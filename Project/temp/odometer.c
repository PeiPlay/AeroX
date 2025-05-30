#include "odometer.h"
#include <math.h>
#include <rtthread.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// 内部函数：角度归一化到 (-PI, PI]
static float __odometer_normalize_angle(float theta)
{
    while(theta > PI)
        theta -= 2 * PI;
    while(theta <= -PI)
        theta += 2 * PI;
    return theta;
}

// 内部函数：查找数组中第一个空闲槽位，返回索引，若无空闲槽位返回 0xFF
static uint8_t __find_free_slot(odometer_t *meter)
{
    for(uint8_t i = 0; i < ODOMETER_MAX_COORDINATE; i++)
    {
        if(meter->coordinates[i].id == 0)
            return i;
    }
    return 0xFF;
}

// 内部函数：根据ID查找坐标系槽位，返回索引，若未找到返回 0xFF
static uint8_t __find_slot_by_id(odometer_t *meter, uint64_t id)
{
    if(id == 0) return 0xFF; // ID为0是无效的
    
    for(uint8_t i = 0; i < ODOMETER_MAX_COORDINATE; i++)
    {
        if(meter->coordinates[i].id == id)
            return i;
    }
    return 0xFF;
}

/*
 * 初始化里程计结构体：
 *  - 将所有槽位标记为无效(ID=0)
 *  - 初始化next_id为1
 *  - 创建互斥锁
 */
void odometer_init(odometer_t *meter)
{
    for (uint8_t i = 0; i < ODOMETER_MAX_COORDINATE; i++)
    {
        meter->coordinates[i].id = 0;
    }
    meter->coordinate_count = 0;
    meter->next_id = 1; // 初始ID从1开始，0表示无效
    
    char lock_name[32];
    rt_snprintf(lock_name, sizeof(lock_name), "odo_lock_%p", meter);
    meter->lock = rt_mutex_create(lock_name, RT_IPC_FLAG_FIFO);
}

/*
 * 增量更新接口：
 * 遍历整个数组，仅对有效的坐标系(ID非0)更新其位置和角度。
 */
void odometer_update(odometer_t *meter, float delta_x, float delta_y, float delta_theta)
{
    float sin_theta, cos_theta;
    float global_delta_x, global_delta_y;

    delta_theta = __odometer_normalize_angle(delta_theta);
    
    for (uint8_t i = 0; i < ODOMETER_MAX_COORDINATE; i++)
    {
        if(meter->coordinates[i].id != 0)
        {
            sin_theta = sinf(meter->coordinates[i].curr_theta);
            cos_theta = cosf(meter->coordinates[i].curr_theta);
          
            global_delta_x = delta_x * cos_theta + delta_y * sin_theta;
            global_delta_y = -delta_x * sin_theta + delta_y * cos_theta;
          
            meter->coordinates[i].curr_x += global_delta_x;
            meter->coordinates[i].curr_y += global_delta_y;
            meter->coordinates[i].curr_theta = __odometer_normalize_angle(meter->coordinates[i].curr_theta + delta_theta);
        }
    }
}

void odometer_update_absolute_theta(odometer_t *meter, float delta_x, float delta_y, float theta)
{
    float sin_theta, cos_theta;
    float global_delta_x, global_delta_y;

    theta = __odometer_normalize_angle(theta);
    
    for (uint8_t i = 0; i < ODOMETER_MAX_COORDINATE; i++)
    {
        if(meter->coordinates[i].id != 0)
        {
            sin_theta = sinf(meter->coordinates[i].curr_theta);
            cos_theta = cosf(meter->coordinates[i].curr_theta);
          
            global_delta_x = delta_x * cos_theta + delta_y * sin_theta;
            global_delta_y = -delta_x * sin_theta + delta_y * cos_theta;
          
            meter->coordinates[i].curr_x += global_delta_x;
            meter->coordinates[i].curr_y += global_delta_y;
            meter->coordinates[i].curr_theta = theta;
        }
    }
}

/*
 * 添加绝对坐标系接口（线程安全）：
 * 查找第一个空闲槽位，将传入的 (x, y, theta) 写入，并分配唯一ID，
 * 同时更新有效坐标系计数，成功时返回true并设置out_id，失败返回false。
 */
bool odometer_add_coordinate_absolute(odometer_t *meter, float x, float y, float theta, uint64_t *out_id)
{
    bool success = false;
    uint8_t index;
    
    rt_mutex_take(meter->lock, RT_WAITING_FOREVER);
    
    if(meter->coordinate_count >= ODOMETER_MAX_COORDINATE)
    {
        rt_mutex_release(meter->lock);
        return false;
    }
    
    index = __find_free_slot(meter);
    if(index == 0xFF)
    {
        rt_mutex_release(meter->lock);
        return false;
    }
    
    // 分配唯一ID
    uint64_t new_id = meter->next_id++;
    if(meter->next_id == 0) meter->next_id = 1; // 避免溢出后分配到ID=0

    meter->coordinates[index].id = new_id;
    meter->coordinate_count++;
    
    meter->coordinates[index].curr_x = x;
    meter->coordinates[index].curr_y = y;
    meter->coordinates[index].curr_theta = __odometer_normalize_angle(theta);

    if(out_id) *out_id = new_id; // 返回分配的ID
    success = true;
    
    rt_mutex_release(meter->lock);
    return success;
}

/*
 * 添加相对坐标系接口（线程安全）：
 * 将输入的 (x, y, theta)（当前车体坐标系下的值），作为新坐标系原点
 * 进而推算出车体当前相对于新坐标系的坐标，并分配唯一ID。
 */
bool odometer_add_coordinate_relative(odometer_t *meter, float x, float y, float theta, uint64_t *out_id)
{
    bool success = false;
    uint8_t index;
    
    rt_mutex_take(meter->lock, RT_WAITING_FOREVER);
    
    if(meter->coordinate_count >= ODOMETER_MAX_COORDINATE)
    {
        rt_mutex_release(meter->lock);
        return false;
    }

    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    float new_x = - x * cos_theta + y * sin_theta;
    float new_y = x * sin_theta + y * cos_theta;
    float new_theta = __odometer_normalize_angle(-theta);
    
    index = __find_free_slot(meter);
    if(index == 0xFF)
    {
        rt_mutex_release(meter->lock);
        return false;
    }
    
    // 分配唯一ID
    uint64_t new_id = meter->next_id++;
    if(meter->next_id == 0) meter->next_id = 1; // 避免溢出后分配到ID=0
    meter->coordinates[index].id = new_id;
    meter->coordinate_count++;
    
    meter->coordinates[index].curr_x = new_x;
    meter->coordinates[index].curr_y = new_y;
    meter->coordinates[index].curr_theta = new_theta;

    if(out_id) *out_id = new_id; // 返回分配的ID
    success = true;
    
    rt_mutex_release(meter->lock);
    return success;
}

/*
 * 删除坐标系接口（线程安全）：
 * 若找到对应ID的坐标系，则将其ID置为0并更新计数，同时将传入的ID指针置零。
 */
bool odometer_delete_coordinate(odometer_t *meter, uint64_t *id)
{
    bool success = false;
    
    if(id == NULL || *id == 0)
        return false;
    
    rt_mutex_take(meter->lock, RT_WAITING_FOREVER);
    
    uint8_t index = __find_slot_by_id(meter, *id);
    if(index != 0xFF)
    {
        meter->coordinates[index].id = 0; // 标记为未使用
		meter->coordinates[index].curr_theta = 0;
		meter->coordinates[index].curr_x = 0;
		meter->coordinates[index].curr_y = 0;
        meter->coordinate_count--;
        *id = 0; // 将传入的ID指针置零
        success = true;
    }
    
    rt_mutex_release(meter->lock);
    return success;
}

/*
 * 坐标转换接口：
 * 将全局坐标 (x, y, theta) 转换到指定坐标系下的机器人局部坐标，
 * 输出为 robot_x, robot_y, robot_theta。
 */
void odometer_transform_to_robot(odometer_t *meter, uint64_t id,
                                  float x, float y, float theta,
                                  float *robot_x, float *robot_y, float *robot_theta)
{
    uint8_t index = __find_slot_by_id(meter, id);
    if(index == 0xFF)
        return;
      
    float curr_x = meter->coordinates[index].curr_x;
    float curr_y = meter->coordinates[index].curr_y;
    float curr_theta = meter->coordinates[index].curr_theta;
  
    float dx = x - curr_x;
    float dy = y - curr_y;
  
    float sin_theta_val = sinf(curr_theta);
    float cos_theta_val = cosf(curr_theta);
  
    *robot_x = dx * cos_theta_val + dy * sin_theta_val;
    *robot_y = -dx * sin_theta_val + dy * cos_theta_val;
    *robot_theta = __odometer_normalize_angle(theta - curr_theta);
}

/*
 * 获取指定坐标系下的当前位置
 */
void odometer_get_position(odometer_t *meter, uint64_t id,
                            float *x, float *y, float *theta)
{
    uint8_t index = __find_slot_by_id(meter, id);
    if(index == 0xFF)
        return;
      
    *x = meter->coordinates[index].curr_x;
    *y = meter->coordinates[index].curr_y;
    *theta = meter->coordinates[index].curr_theta;
}

/*
 * 获取指定坐标系下某点相对于车体的距离
 */
float odometer_get_distance(odometer_t *meter, uint64_t id, 
                            float x, float y, float theta)
{
    theta = __odometer_normalize_angle(theta);
    float robot_x, robot_y, robot_theta;
    odometer_transform_to_robot(meter, id, x, y, theta, &robot_x, &robot_y, &robot_theta);
    return sqrtf(robot_x * robot_x + robot_y * robot_y);
}


#ifndef __ODOMETER_H_
#define __ODOMETER_H_

#include "zf_common_headfile.h"
#include <stdint.h>
#include <rtthread.h>

/**
 * @brief 最大支持的坐标系数量
 * @note 可以根据实际内存需求调整此值
 */
#define ODOMETER_MAX_COORDINATE 20

/**
 * @file odometer.h
 * @brief 多坐标系里程计模块
 * 
 * @details
 * 本模块提供多坐标系下的里程计功能，支持：
 * - 多坐标系管理（最多支持20个同时存在的坐标系）
 * - 相对/绝对坐标系创建
 * - 坐标系之间的位置变换
 * - 线程安全的坐标系管理
 * 
 * 坐标系约定：
 * 1. 机器人局部坐标系：
 *    - Y轴：指向车头（前方）为正方向
 *    - X轴：指向车体右侧为正方向
 *    - 角度：逆时针为正，范围(-PI, PI]
 * 2. 全局坐标系：
 *    - 当机器人局部坐标系与全局坐标系重合时，车头指向全局Y轴正方向
 * 
 * 使用流程：
 * 1. 定义odometer_t结构体变量
 * 2. 调用odometer_init()初始化
 * 3. 添加所需的坐标系（absolute或relative）
 * 4. 定期调用odometer_update()更新位置
 * 5. 使用transform或get_position获取位置信息
 * 
 * 注意事项：
 * 1. 所有角度均使用弧度制
 * 2. 确保update的调用频率足够高，以保证位置计算精度
 * 3. 坐标系的增删操作是线程安全的，但更新操作不是
 */

/*
  里程计模块说明：
    - 坐标系约定：机器人局部坐标中，车头（前方）为 y 轴正方向，右侧为 x 轴正方向；
    - 当局部坐标系与全局坐标系对齐时（角度为 0），机器人车头指向全局 y 轴正方向。
    - 每个坐标系采用一个唯一ID标识，ID为0表示未使用。
    - 添加和删除操作通过互斥锁保护，update函数每次遍历整个数组，仅更新有效槽位。
*/

// 每个坐标系数据结构使用64位唯一ID标识
typedef struct
{
    float curr_x;       // 相对于该坐标系原点的 x 坐标
    float curr_y;       // 相对于该坐标系原点的 y 坐标
    float curr_theta;   // 相对于该坐标系原点的角度（弧度制，归一化到 (-PI, PI]）
    uint64_t id;        // 唯一ID，0表示未使用
} odometer_coordinate_t;

typedef struct
{
    odometer_coordinate_t coordinates[ODOMETER_MAX_COORDINATE];  // 坐标系数组
    uint8_t coordinate_count;  // 有效坐标系数量
    uint64_t next_id;          // 下一个要分配的唯一ID
    rt_mutex_t lock;           // 互斥锁，保护添加/删除操作
} odometer_t;

typedef uint64_t odometer_coordinate_index_t; // 坐标系索引类型，使用64位ID表示

/**
 * @brief 初始化里程计模块
 * @param meter 里程计结构体指针
 * @note 使用其他函数之前必须先调用此函数进行初始化
 */
void odometer_init(odometer_t *meter);

/**
 * @brief 增量更新所有坐标系的位置
 * @param meter 里程计结构体指针
 * @param delta_x 局部坐标系X方向位移增量
 * @param delta_y 局部坐标系Y方向位移增量
 * @param delta_theta 局部坐标系角度增量(弧度)
 * @note 应定期调用此函数更新位置信息
 */
void odometer_update(odometer_t *meter, float delta_x, float delta_y, float delta_theta);

void odometer_update_absolute_theta(odometer_t *meter, float delta_x, float delta_y, float theta);

/**
 * @brief 添加一个绝对坐标系
 * @param meter 里程计结构体指针
 * @param x 全局坐标系下的X坐标
 * @param y 全局坐标系下的Y坐标
 * @param theta 全局坐标系下的角度(弧度)
 * @param out_id 输出参数：成功时返回分配的坐标系ID
 * @return 成功返回true，失败返回false
 */
bool odometer_add_coordinate_absolute(odometer_t *meter, float x, float y, float theta, uint64_t *out_id);

/**
 * @brief 添加一个相对坐标系
 * @param meter 里程计结构体指针
 * @param x 当前局部坐标系下的X坐标
 * @param y 当前局部坐标系下的Y坐标
 * @param theta 当前局部坐标系下的角度(弧度)
 * @param out_id 输出参数：成功时返回分配的坐标系ID
 * @return 成功返回true，失败返回false
 */
bool odometer_add_coordinate_relative(odometer_t *meter, float x, float y, float theta, uint64_t *out_id);

/**
 * @brief 删除指定的坐标系
 * @param meter 里程计结构体指针
 * @param id 指向要删除的坐标系ID的指针，成功删除后会将其置为0
 * @return 成功返回true，失败返回false
 */
bool odometer_delete_coordinate(odometer_t *meter, uint64_t *id);

/**
 * @brief 将全局坐标转换为指定坐标系下的局部坐标
 * @param meter 里程计结构体指针
 * @param id 目标坐标系ID
 * @param x 全局X坐标
 * @param y 全局Y坐标
 * @param theta 全局角度(弧度)
 * @param robot_x 输出：局部X坐标
 * @param robot_y 输出：局部Y坐标
 * @param robot_theta 输出：局部角度(弧度)
 */
void odometer_transform_to_robot(odometer_t *meter, uint64_t id,
                                float x, float y, float theta,
                                float *robot_x, float *robot_y, float *robot_theta);

/**
 * @brief 获取指定坐标系下的当前位置
 * @param meter 里程计结构体指针
 * @param id 目标坐标系ID
 * @param x 输出：当前X坐标
 * @param y 输出：当前Y坐标
 * @param theta 输出：当前角度(弧度)
 */
void odometer_get_position(odometer_t *meter, uint64_t id,
                          float *x, float *y, float *theta);

/**
 * @brief 计算在指定坐标系下，某点相对于车体的直线距离
 * @param meter 里程计结构体指针
 * @param id 目标坐标系ID
 * @param x 全局X坐标
 * @param y 全局Y坐标
 * @param theta 全局角度(弧度)
 * @return 返回相对于车体的直线距离
 */
float odometer_get_distance(odometer_t *meter, uint64_t id, 
    float x, float y, float theta);

#endif // __ODOMETER_H_

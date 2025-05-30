#include "taskCommuCheck.h"
#include "config.h"
#include "cmsis_os.h" // For osDelay
#include "lidar.h"
#include "taskNrfResponse.h"

// ========== 轮询周期配置 ==========
#define COMMU_CHECK_POLL_PERIOD_MS  5    // 5ms轮询周期

// ========== NRF连接检测配置 ==========
static const uint64_t NRF_DISCONNECT_TIMEOUT_MS = 250;   // 250ms 未收到包则掉线
static const uint64_t NRF_MAX_PACKET_INTERVAL_MS = 250;  // 恢复期间包之间最大间隔250ms
static const uint8_t NRF_MIN_RECOVERY_PACKETS = 2;       // 至少连续2个包满足条件才能恢复

// ========== Lidar连接检测配置 ==========
static const uint64_t LIDAR_DISCONNECT_TIMEOUT_MS = 500;   // 500ms 未收到包则掉线
static const uint64_t LIDAR_MAX_PACKET_INTERVAL_MS = 500;  // 恢复期间包之间最大间隔500ms
static const uint8_t LIDAR_MIN_RECOVERY_PACKETS = 1;        // 至少连续1个包满足条件才能恢复

// ========== 全局变量定义 ==========
ground_station_rx_data_t ground_station_rx_data; // NRF接收数据结构体
ground_station_status_t ground_station_status; // NRF状态数据结构体
lidar_status_t lidar_status; // Lidar状态数据结构体

// ========== 内部辅助函数 ==========
static void update_recovery_state(connection_recovery_state_t* recovery, 
                                 uint64_t max_interval_ms)
{
    if (recovery->consecutive_packets == 0) // 新的恢复尝试开始
    {
        recovery->consecutive_packets = 1;
        recovery->time_since_last_packet_ms = 0;
    }
    else // 继续当前的恢复尝试
    {
        if (recovery->time_since_last_packet_ms <= max_interval_ms)
        {
            recovery->consecutive_packets++;
            recovery->time_since_last_packet_ms = 0;
        }
        else // 包间隔过长，重新开始新的序列
        {
            recovery->consecutive_packets = 1;
            recovery->time_since_last_packet_ms = 0;
        }
    }
}

// ========== NRF通信回调函数 ==========
void ground_station_rx_callback(uint8_t channel, uint8_t* data, uint8_t len)
{
    if(len != sizeof(ground_station_rx_data_t)) return;
    ground_station_rx_data = *(ground_station_rx_data_t*)data;

    // 仅设置接收标志位
    ground_station_status.rx_flag = 1;
}

// ========== Lidar通信回调函数 ==========
void lidar_pose_rx_callback(const LidarPoseData* pose_data)
{
    if (!pose_data || !pose_data->valid) return;
    
    // 仅设置标志位和增加计数
    lidar_status.pose_rx_flag = 1;
    lidar_status.pose_count++;
}

void lidar_imu_rx_callback(const LidarImuData* imu_data)
{
    if (!imu_data || !imu_data->valid) return;
    
    // 仅设置标志位和增加计数
    lidar_status.imu_rx_flag = 1;
    lidar_status.imu_count++;
}

// ========== 连接状态检查函数 ==========
static void check_nrf_connection_status(void)
{
    static uint8_t dummy_packet[4] = {0,0,0,0};
    
    if (ground_station_status.is_connected)
    {
        // 已连接状态：检查是否收到数据
        if (ground_station_status.rx_flag)
        {
            ground_station_status.rx_flag = 0; // 清除标志位
            // 收到数据，重置时间计数器（通过recovery结构体维护）
            ground_station_status.recovery.time_since_last_packet_ms = 0;
        }
        else
        {
            // 未收到数据，累加时间
            ground_station_status.recovery.time_since_last_packet_ms += COMMU_CHECK_POLL_PERIOD_MS;
            if (ground_station_status.recovery.time_since_last_packet_ms > NRF_DISCONNECT_TIMEOUT_MS)
            {
                ground_station_status.is_connected = 0;
                ground_station_status.recovery.consecutive_packets = 0; // 重置恢复状态
                ground_station_status.recovery.time_since_last_packet_ms = 0;
            }
        }
    }
    else // 未连接状态：检查恢复条件
    {
        if (ground_station_status.rx_flag)
        {
            ground_station_status.rx_flag = 0; // 清除标志位
            update_recovery_state(&ground_station_status.recovery, NRF_MAX_PACKET_INTERVAL_MS);
        }
        else
        {
            // 未收到数据，累加恢复状态的时间
            ground_station_status.recovery.time_since_last_packet_ms += COMMU_CHECK_POLL_PERIOD_MS;
        }
        
        if (ground_station_status.recovery.consecutive_packets >= NRF_MIN_RECOVERY_PACKETS)
        {
            ground_station_status.is_connected = 1;
            ground_station_status.recovery.consecutive_packets = 0; // 清除恢复状态
            ground_station_status.recovery.time_since_last_packet_ms = 0;
        }
        
        taskENTER_CRITICAL();
        nrf.transmit(dummy_packet, sizeof(dummy_packet)); // 发送虚拟包以维持连接
        taskEXIT_CRITICAL();
    }
}

static void check_lidar_connection_status(void)
{
    static uint8_t rate_calc_counter = 0;
    static uint64_t last_pose_count = 0;
    static uint64_t last_imu_count = 0;
    
    // 每200次循环（1秒）计算一次速率 (200 * 5ms = 1000ms)
    rate_calc_counter++;
    if (rate_calc_counter >= (1000 / COMMU_CHECK_POLL_PERIOD_MS))
    {
        rate_calc_counter = 0;
        
        // 计算pose速率
        uint64_t pose_diff = lidar_status.pose_count - last_pose_count;
        lidar_status.pose_rate = (float)pose_diff;
        last_pose_count = lidar_status.pose_count;
        
        // 计算IMU速率
        uint64_t imu_diff = lidar_status.imu_count - last_imu_count;
        lidar_status.imu_rate = (float)imu_diff;
        last_imu_count = lidar_status.imu_count;
    }

    // 处理位姿数据标志位
    if (lidar_status.pose_rx_flag)
    {
        lidar_status.pose_rx_flag = 0;
        lidar_status.pose_time_since_last_ms = 0;
        
        if (!lidar_status.is_connected)
        {
            update_recovery_state(&lidar_status.recovery, LIDAR_MAX_PACKET_INTERVAL_MS);
        }
    }
    else
    {
        lidar_status.pose_time_since_last_ms += COMMU_CHECK_POLL_PERIOD_MS;
    }

    // 处理IMU数据标志位
    if (lidar_status.imu_rx_flag)
    {
        lidar_status.imu_rx_flag = 0;
        lidar_status.imu_time_since_last_ms = 0;
        
        if (!lidar_status.is_connected)
        {
            update_recovery_state(&lidar_status.recovery, LIDAR_MAX_PACKET_INTERVAL_MS);
        }
    }
    else
    {
        lidar_status.imu_time_since_last_ms += COMMU_CHECK_POLL_PERIOD_MS;
    }

    if (lidar_status.is_connected)
    {
        // 已连接状态：检查是否超时断开
        // 检查位姿数据和IMU数据，任一超时则认为断开
        if (lidar_status.pose_time_since_last_ms > LIDAR_DISCONNECT_TIMEOUT_MS || 
            lidar_status.imu_time_since_last_ms > LIDAR_DISCONNECT_TIMEOUT_MS)
        {
            lidar_status.is_connected = 0;
            lidar_status.recovery.consecutive_packets = 0; // 重置恢复状态
            lidar_status.recovery.time_since_last_packet_ms = 0;
        }
    }
    else // 未连接状态：检查恢复条件
    {
        // 更新恢复状态的时间计数器
        if (!lidar_status.pose_rx_flag && !lidar_status.imu_rx_flag)
        {
            lidar_status.recovery.time_since_last_packet_ms += COMMU_CHECK_POLL_PERIOD_MS;
        }
        
        if (lidar_status.recovery.consecutive_packets >= LIDAR_MIN_RECOVERY_PACKETS)
        {
            lidar_status.is_connected = 1;
            lidar_status.recovery.consecutive_packets = 0; // 清除恢复状态
            lidar_status.recovery.time_since_last_packet_ms = 0;
        }
    }
}

void check_user(void);
// ========== 通信检查主任务函数 ==========
void taskCommuCheck(void *argument)
{
    // 进入主循环
    while (1)
    {
        // 检查各组件连接状态
        check_nrf_connection_status();
        check_lidar_connection_status();
        
        // TODO: 在这里可以添加其他通信组件的连接检查
        // 例如：检查其他传感器、模块的连接状态
        
        check_user(); // 用户自定义检查函数
        osDelay(COMMU_CHECK_POLL_PERIOD_MS); // 每5ms检查一次状态
    }
}

void check_user(void)
{
    NRF_SET_CHASSIS_AZIMUTH_BIT(0, !LIDAR_IS_CONNECTED);
}
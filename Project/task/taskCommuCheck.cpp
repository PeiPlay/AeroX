#include "taskCommuCheck.h"
#include "config.h"
#include "time_utils.h"
#include "cmsis_os.h" // For osDelay
#include "lidar.h"
#include "taskNrfResponse.h"

// ========== NRF连接检测配置 ==========
static const timestamp_t NRF_DISCONNECT_TIMEOUT = TimeStamp_FromMilliseconds(250);   // 250ms 未收到包则掉线
static const timestamp_t NRF_RECOVERY_WINDOW = TimeStamp_FromMilliseconds(500);      // 500ms 恢复窗口期
static const timestamp_t NRF_MAX_PACKET_INTERVAL = TimeStamp_FromMilliseconds(250);  // 恢复期间包之间最大间隔250ms
static const uint8_t NRF_MIN_RECOVERY_PACKETS = 2;                                   // 至少连续2个包满足条件才能恢复

// ========== Lidar连接检测配置 ==========
static const timestamp_t LIDAR_DISCONNECT_TIMEOUT = TimeStamp_FromMilliseconds(300);   // 300ms 未收到包则掉线
static const timestamp_t LIDAR_RECOVERY_WINDOW = TimeStamp_FromMilliseconds(600);      // 600ms 恢复窗口期
static const timestamp_t LIDAR_MAX_PACKET_INTERVAL = TimeStamp_FromMilliseconds(300);  // 恢复期间包之间最大间隔300ms
static const uint8_t LIDAR_MIN_RECOVERY_PACKETS = 3;                                   // 至少连续3个包满足条件才能恢复

// ========== 全局变量定义 ==========
ground_station_rx_data_t ground_station_rx_data; // NRF接收数据结构体
ground_station_status_t ground_station_status; // NRF状态数据结构体
lidar_status_t lidar_status; // Lidar状态数据结构体

// ========== 内部辅助函数 ==========
static void update_recovery_state(connection_recovery_state_t* recovery, 
                                 timestamp_t current_ts, 
                                 timestamp_t max_interval)
{
    if (recovery->consecutive_packets == 0) // 新的恢复尝试开始
    {
        recovery->consecutive_packets = 1;
        recovery->first_packet_ts = current_ts;
        recovery->last_packet_ts = current_ts;
    }
    else // 继续当前的恢复尝试
    {
        timestamp_t interval = current_ts - recovery->last_packet_ts;
        if (interval <= max_interval)
        {
            recovery->consecutive_packets++;
            recovery->last_packet_ts = current_ts;
        }
        else // 包间隔过长，重新开始新的序列
        {
            recovery->consecutive_packets = 1;
            recovery->first_packet_ts = current_ts;
            recovery->last_packet_ts = current_ts;
        }
    }
}

static bool check_recovery_conditions(const connection_recovery_state_t* recovery,
                                     timestamp_t current_ts,
                                     uint8_t min_packets,
                                     timestamp_t recovery_window)
{
    if (recovery->consecutive_packets >= min_packets)
    {
        // 检查整个恢复序列是否在窗口期内完成
        timestamp_t total_duration = recovery->last_packet_ts - recovery->first_packet_ts;
        return (total_duration <= recovery_window);
    }
    return false;
}

static bool is_recovery_expired(const connection_recovery_state_t* recovery,
                               timestamp_t current_ts,
                               timestamp_t recovery_window)
{
    if (recovery->consecutive_packets > 0)
    {
        timestamp_t elapsed = current_ts - recovery->first_packet_ts;
        return (elapsed > recovery_window);
    }
    return false;
}

// ========== NRF通信回调函数 ==========
void ground_station_rx_callback(uint8_t channel, uint8_t* data, uint8_t len)
{
    if(len != sizeof(ground_station_rx_data_t)) return;
    ground_station_rx_data = *(ground_station_rx_data_t*)data;

    timestamp_t current_ts = TimeUtils_GetGlobalTick();
    ground_station_status.last_rx_timestamp = current_ts;

    if (!ground_station_status.is_connected)
    {
        update_recovery_state(&ground_station_status.recovery, current_ts, NRF_MAX_PACKET_INTERVAL);
    }
}

// ========== Lidar通信回调函数 ==========
void lidar_pose_rx_callback(const LidarPoseData* pose_data)
{
    if (!pose_data || !pose_data->valid) return;
    
    timestamp_t current_ts = TimeUtils_GetGlobalTick();
    lidar_status.last_pose_timestamp = current_ts;

    if (!lidar_status.is_connected)
    {
        update_recovery_state(&lidar_status.recovery, current_ts, LIDAR_MAX_PACKET_INTERVAL);
    }
}

void lidar_imu_rx_callback(const LidarImuData* imu_data)
{
    if (!imu_data || !imu_data->valid) return;
    
    timestamp_t current_ts = TimeUtils_GetGlobalTick();
    lidar_status.last_imu_timestamp = current_ts;

    // IMU数据也会触发连接恢复逻辑，与pose数据共享恢复状态
    if (!lidar_status.is_connected)
    {
        update_recovery_state(&lidar_status.recovery, current_ts, LIDAR_MAX_PACKET_INTERVAL);
    }
}

// ========== 连接状态检查函数 ==========
static void check_nrf_connection_status(timestamp_t current_ts)
{
    static uint8_t dummy_packet[4] = {0,0,0,0};
    
    if (ground_station_status.is_connected)
    {
        // 已连接状态：检查是否超时断开
        timestamp_t time_since_last_rx = current_ts - ground_station_status.last_rx_timestamp;
        if (time_since_last_rx > NRF_DISCONNECT_TIMEOUT)
        {
            ground_station_status.is_connected = 0;
            ground_station_status.recovery.consecutive_packets = 0; // 重置恢复状态
        }
    }
    else // 未连接状态：检查恢复条件
    {
        if (check_recovery_conditions(&ground_station_status.recovery, current_ts, 
                                    NRF_MIN_RECOVERY_PACKETS, NRF_RECOVERY_WINDOW))
        {
            ground_station_status.is_connected = 1;
            ground_station_status.recovery.consecutive_packets = 0; // 清除恢复状态
        }
        else if (is_recovery_expired(&ground_station_status.recovery, current_ts, NRF_RECOVERY_WINDOW))
        {
            ground_station_status.recovery.consecutive_packets = 0; // 恢复序列超时，作废
        }
        taskENTER_CRITICAL();
        nrf.transmit(dummy_packet, sizeof(dummy_packet)); // 发送虚拟包以维持连接
        taskEXIT_CRITICAL();
    }
}

static void check_lidar_connection_status(timestamp_t current_ts)
{
    if (lidar_status.is_connected)
    {
        // 已连接状态：检查是否超时断开
        // 检查位姿数据和IMU数据，任一超时则认为断开
        timestamp_t pose_elapsed = current_ts - lidar_status.last_pose_timestamp;
        timestamp_t imu_elapsed = current_ts - lidar_status.last_imu_timestamp;
        
        if (pose_elapsed > LIDAR_DISCONNECT_TIMEOUT || imu_elapsed > LIDAR_DISCONNECT_TIMEOUT)
        {
            lidar_status.is_connected = 0;
            lidar_status.recovery.consecutive_packets = 0; // 重置恢复状态
        }
    }
    else // 未连接状态：检查恢复条件
    {
        if (check_recovery_conditions(&lidar_status.recovery, current_ts, 
                                    LIDAR_MIN_RECOVERY_PACKETS, LIDAR_RECOVERY_WINDOW))
        {
            lidar_status.is_connected = 1;
            lidar_status.recovery.consecutive_packets = 0; // 清除恢复状态
        }
        else if (is_recovery_expired(&lidar_status.recovery, current_ts, LIDAR_RECOVERY_WINDOW))
        {
            lidar_status.recovery.consecutive_packets = 0; // 恢复序列超时，作废
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
        timestamp_t current_ts = TimeUtils_GetGlobalTick();

        // 检查各组件连接状态
        check_nrf_connection_status(current_ts);
        check_lidar_connection_status(current_ts);
        
        // TODO: 在这里可以添加其他通信组件的连接检查
        // 例如：检查其他传感器、模块的连接状态
        
        check_user(); // 用户自定义检查函数
        osDelay(10); // 每10ms检查一次状态
    }
}

void check_user(void)
{
    NRF_SET_CHASSIS_AZIMUTH_BIT(0, !LIDAR_IS_CONNECTED);
}
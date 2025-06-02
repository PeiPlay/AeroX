#include "taskManager.h"
#include "config.h"
#include "taskCommuCheck.h"  // 包含回调函数声明
extern "C" {  // 添加 extern "C" 块
__weak void taskManager_Init(void* argument)
{
    lidar.init();
    osDelay(5);
    lidar.start();
    osDelay(5);
    hc12.init();
    //upt201.init();
    //osDelay(5);
    //upt201.start();
    taskCommuCheck_Init(argument);
}

// 通信检查初始化函数，统一管理NRF等通信组件的初始化
void taskCommuCheck_Init(void* argument)
{
    nrf.init();
    ground_station_status.is_connected = 0; // 初始状态为未连接
    ground_station_status.rx_flag = 0; // 初始化接收标志位
    ground_station_status.recovery.time_since_last_packet_ms = 0;
    ground_station_status.recovery.consecutive_packets = 0;
    
    // 初始化Lidar状态
    lidar_status.is_connected = 0; // 初始状态为未连接
    lidar_status.pose_rx_flag = 0; // 初始化位姿接收标志位
    lidar_status.imu_rx_flag = 0;  // 初始化IMU接收标志位
    lidar_status.pose_count = 0;
    lidar_status.imu_count = 0;
    lidar_status.pose_rate = 0.0f;
    lidar_status.imu_rate = 0.0f;
    lidar_status.pose_time_since_last_ms = 0;
    lidar_status.imu_time_since_last_ms = 0;
    lidar_status.recovery.time_since_last_packet_ms = 0;
    lidar_status.recovery.consecutive_packets = 0;
    
    // 设置激光雷达回调函数
    lidar.setPoseRxCallback(lidar_pose_rx_callback);
    lidar.setImuRxCallback(lidar_imu_rx_callback);
}
}  // 结束 extern "C" 块


#include "taskManager.h"
#include "config.h"
#include "taskCommuCheck.h"  // 包含回调函数声明
extern "C" {  // 添加 extern "C" 块
__weak void taskManager_Init(void* argument)
{
    lidar.init();
    osDelay(5);
    lidar.start();
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
    ground_station_status.last_rx_timestamp = 0;
    ground_station_status.recovery.first_packet_ts = 0;
    ground_station_status.recovery.last_packet_ts = 0;
    ground_station_status.recovery.consecutive_packets = 0;
    
    // 初始化Lidar状态
    lidar_status.is_connected = 0; // 初始状态为未连接
    lidar_status.last_pose_timestamp = 0;
    lidar_status.last_imu_timestamp = 0;
    lidar_status.recovery.first_packet_ts = 0;
    lidar_status.recovery.last_packet_ts = 0;
    lidar_status.recovery.consecutive_packets = 0;
    
    // 设置激光雷达回调函数
    lidar.setPoseRxCallback(lidar_pose_rx_callback);
    lidar.setImuRxCallback(lidar_imu_rx_callback);
}
}  // 结束 extern "C" 块


#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// 滤波器配置
#define LIDAR_POSITION_FILTER_SIZE 10  // 位置滑动均值滤波器大小，可修改
#define LIDAR_VELOCITY_LOWPASS_ALPHA 0.4f  // 速度一阶低通滤波器系数初始值 (0-1, 越小滤波越强)

// DMA接收配置
#define LIDAR_DMA_BUFFER_SIZE 32      // DMA接收缓冲区大小

// Lidar通信协议常量
#define LIDAR_HEADER1         0x3F // '?'
#define LIDAR_HEADER2         0x21 // '!'
#define LIDAR_FOOTER          0x21 // '!'
#define LIDAR_CMD_POSE        0x01 // 雷达位姿数据命令码
#define LIDAR_CMD_IMU         0x02 // IMU数据命令码
#define LIDAR_PAYLOAD_SIZE    12   // 3个float，每个4字节
#define LIDAR_PACKET_BASE_SIZE 4   // HEADER1, HEADER2, CMD, FOOTER
#define LIDAR_PACKET_TOTAL_SIZE (LIDAR_PACKET_BASE_SIZE + LIDAR_PAYLOAD_SIZE) // 16 bytes

// 雷达位姿数据结构
struct LidarPoseData {
    float x;       // X坐标 (地面坐标系)
    float y;       // Y坐标 (地面坐标系)
    float z;       // Z坐标 (地面坐标系)
    bool valid;    // 数据有效性标志
};

// 雷达速度数据结构
struct LidarVelocityData {
    float vx_filtered; // X方向滤波后速度 (m/s) (地面坐标系)
    float vy_filtered; // Y方向滤波后速度 (m/s) (地面坐标系)
    float vz_filtered; // Z方向滤波后速度 (m/s) (地面坐标系)
    bool valid;        // 速度数据有效性标志
};

// IMU姿态数据结构
struct LidarImuData {
    float roll;    // Roll角
    float pitch;   // Pitch角
    float yaw;     // Yaw角
    bool valid;    // 数据有效性标志
};

// Lidar数据接收回调函数类型定义
typedef void (*lidar_pose_rx_callback_t)(const struct LidarPoseData* pose_data);
typedef void (*lidar_imu_rx_callback_t)(const struct LidarImuData* imu_data);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Lidar {
public:
    Lidar(UART_HandleTypeDef* huart, float pose_frequency_hz = 50.0f);
    ~Lidar();

    bool init();
    bool start();
    void stop();

    // 设置位姿数据频率 (需要在start()之前调用)
    void setPoseFrequency(float frequency_hz);
    
    // 获取当前设置的频率
    float getPoseFrequency() const { return pose_frequency_hz_; }

    LidarPoseData getPoseData() const;
    LidarVelocityData getVelocityData() const;
    LidarImuData getImuData() const;

    bool isRunning() const { return running_; }

    // 设置外部回调函数
    void setPoseRxCallback(lidar_pose_rx_callback_t callback);
    void setImuRxCallback(lidar_imu_rx_callback_t callback);
    
    // 外部接口：用于通信检查任务
    void setDataValid(bool pose_valid, bool imu_valid);
    bool isPoseDataValid() const;
    bool isImuDataValid() const;

    // 设置速度低通滤波器系数 (0-1, 越小滤波越强)
    void setVelocityLowpassAlpha(float alpha);
    
    // 获取当前速度低通滤波器系数
    float getVelocityLowpassAlpha() const { return velocity_lowpass_alpha_; }

    // DMA接收完成回调
    void dmaRxCallback(UART_HandleTypeDef *huart);

private:
    UART_HandleTypeDef* huart_;
    volatile bool running_;

    // 位姿数据频率配置 (Hz)
    float pose_frequency_hz_;

    // DMA接收缓冲区
    uint8_t dma_rx_buffer_[LIDAR_DMA_BUFFER_SIZE];

    LidarPoseData pose_data_;
    LidarVelocityData velocity_data_;
    LidarImuData imu_data_;

    // 速度计算相关
    LidarPoseData last_pose_;
    bool velocity_initialized_;

    // 一阶低通滤波器 - 用于速度滤波
    float velocity_lowpass_alpha_;  // 低通滤波器系数
    float vx_lowpass_prev_;
    float vy_lowpass_prev_;
    float vz_lowpass_prev_;
    bool velocity_lowpass_initialized_;

    // 滑动均值滤波器 - 用于位置滤波    
    float x_filter_buffer_[LIDAR_POSITION_FILTER_SIZE];
    float y_filter_buffer_[LIDAR_POSITION_FILTER_SIZE];
    float z_filter_buffer_[LIDAR_POSITION_FILTER_SIZE];
    uint8_t filter_index_;
    uint8_t filter_count_;
    uint32_t pose_packet_count_;
    uint32_t imu_packet_count_;
    uint32_t error_count_;

    // 回调函数指针
    lidar_pose_rx_callback_t pose_rx_callback_;
    lidar_imu_rx_callback_t imu_rx_callback_;

    enum ParseState {
        STATE_WAIT_HEADER1,
        STATE_WAIT_HEADER2,
        STATE_WAIT_COMMAND,
        STATE_RECEIVE_PAYLOAD,
        STATE_WAIT_FOOTER
    };

    ParseState state_;
    uint8_t current_cmd_;
    uint8_t payload_[LIDAR_PAYLOAD_SIZE];
    uint8_t payload_index_;

    bool processByte(uint8_t byte);
    bool parsePosePacket();
    bool parseImuPacket();
    
    // DMA相关函数
    bool startDmaReceive();
    uint16_t getNextReceiveLength() const;
    void processReceivedData(uint8_t* pData, uint16_t Size);

    // 速度计算和滤波
    void calculateVelocity();
    void updatePositionFilter(float x, float y, float z);
    float getFilteredPosition(const float* buffer) const;
    float applyLowpassFilter(float current_value, float previous_filtered, float alpha) const;

    // 辅助函数：从字节数组转换到float
    static float bytesToFloat(const uint8_t* bytes);
};
#endif //

#endif // LIDAR_H

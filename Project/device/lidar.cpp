#include "lidar.h"
#include "cmsis_os.h"
#include <string.h>

// 构造函数
Lidar::Lidar(UART_HandleTypeDef* huart, float pose_frequency_hz)
    : huart_(huart),
      running_(false),
      pose_frequency_hz_(pose_frequency_hz),
      pose_packet_count_(0),
      imu_packet_count_(0),
      error_count_(0),
      state_(STATE_WAIT_HEADER1),
      current_cmd_(0),
      payload_index_(0),
      velocity_initialized_(false),
      filter_index_(0),
      filter_count_(0),
      pose_rx_callback_(nullptr),
      imu_rx_callback_(nullptr),
      velocity_lowpass_alpha_(LIDAR_VELOCITY_LOWPASS_ALPHA),
      vx_lowpass_prev_(0.0f),
      vy_lowpass_prev_(0.0f),
      vz_lowpass_prev_(0.0f),
      velocity_lowpass_initialized_(false) {
    memset(&pose_data_, 0, sizeof(LidarPoseData));
    memset(&velocity_data_, 0, sizeof(LidarVelocityData));
    memset(&imu_data_, 0, sizeof(LidarImuData));
    memset(&last_pose_, 0, sizeof(LidarPoseData));
    memset(x_filter_buffer_, 0, sizeof(x_filter_buffer_));
    memset(y_filter_buffer_, 0, sizeof(y_filter_buffer_));
    memset(z_filter_buffer_, 0, sizeof(z_filter_buffer_));
    memset(dma_rx_buffer_, 0, sizeof(dma_rx_buffer_));
    
    pose_data_.valid = false;
    velocity_data_.valid = false;
    imu_data_.valid = false;
}

// 析构函数
Lidar::~Lidar() {
    stop();
}

// 初始化设备
bool Lidar::init() {
    state_ = STATE_WAIT_HEADER1;
    current_cmd_ = 0;
    payload_index_ = 0;
    
    pose_packet_count_ = 0;
    imu_packet_count_ = 0;
    error_count_ = 0;
    
    pose_data_.valid = false;
    imu_data_.valid = false;
    
    return true;
}

// 启动设备
bool Lidar::start() {
    if (running_) {
        return true;
    }
    running_ = true;
    return startDmaReceive();
}

// 停止设备
void Lidar::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    HAL_UART_DMAStop(huart_);
}

// 获取雷达位姿数据
LidarPoseData Lidar::getPoseData() const {
    return pose_data_;
}

// 获取IMU数据
LidarImuData Lidar::getImuData() const {
    return imu_data_;
}

// 获取雷达速度数据
LidarVelocityData Lidar::getVelocityData() const {
    return velocity_data_;
}

// 设置位姿数据接收回调函数
void Lidar::setPoseRxCallback(lidar_pose_rx_callback_t callback) {
    pose_rx_callback_ = callback;
}

// 设置IMU数据接收回调函数
void Lidar::setImuRxCallback(lidar_imu_rx_callback_t callback) {
    imu_rx_callback_ = callback;
}

// 外部接口：设置数据有效性
void Lidar::setDataValid(bool pose_valid, bool imu_valid) {
    pose_data_.valid = pose_valid;
    imu_data_.valid = imu_valid;
}

// 外部接口：检查位姿数据有效性
bool Lidar::isPoseDataValid() const {
    return pose_data_.valid;
}

// 外部接口：检查IMU数据有效性
bool Lidar::isImuDataValid() const {
    return imu_data_.valid;
}

// 设置位姿数据频率
void Lidar::setPoseFrequency(float frequency_hz) {
    if (frequency_hz > 0.0f && frequency_hz <= 1000.0f) { // 限制频率范围
        pose_frequency_hz_ = frequency_hz;
    }
}

// 设置速度低通滤波器系数
void Lidar::setVelocityLowpassAlpha(float alpha) {
    if (alpha >= 0.0f && alpha <= 1.0f) {
        velocity_lowpass_alpha_ = alpha;
    }
}

// 启动DMA接收
bool Lidar::startDmaReceive() {
    if (!running_) {
        return false;
    }
    
    // 清除可能存在的错误标志
    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    
    // 根据当前状态确定接收长度
    uint16_t receive_length = getNextReceiveLength();
    
    // 启动DMA接收
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart_, dma_rx_buffer_, receive_length);
    return (status == HAL_OK);
}

// 根据当前解析状态确定下一次DMA接收的数据长度
uint16_t Lidar::getNextReceiveLength() const {
    switch (state_) {
        case STATE_WAIT_HEADER1:
        case STATE_WAIT_HEADER2:
        case STATE_WAIT_COMMAND:
            return 1; // 逐字节接收控制字节
            
        case STATE_RECEIVE_PAYLOAD:
            return LIDAR_PAYLOAD_SIZE - payload_index_; // 接收剩余载荷数据
            
        case STATE_WAIT_FOOTER:
            return 1; // 接收尾部字节
            
        default:
            return 1;
    }
}

// DMA接收完成回调
void Lidar::dmaRxCallback(UART_HandleTypeDef *huart) {
    if (huart != huart_ || !running_) {
        return;
    }
    
    // 获取本次接收的数据长度
    uint16_t received_length = getNextReceiveLength();
    
    // 处理接收到的数据
    processReceivedData(dma_rx_buffer_, received_length);
    
    // 启动下一次DMA接收
    if (running_) {
        HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart_, dma_rx_buffer_, getNextReceiveLength());
        if (status != HAL_OK) {
            error_count_++;
            startDmaReceive();
        }
    }
}

// 处理批量接收的数据
void Lidar::processReceivedData(uint8_t* pData, uint16_t Size) {
    for (uint16_t i = 0; i < Size; ++i) {
        if (!processByte(pData[i])) {
            // 解析错误时的处理
            break;
        }
    }
}

// 处理单个字节的状态机
bool Lidar::processByte(uint8_t byte) {
    switch (state_) {
        case STATE_WAIT_HEADER1:
            if (byte == LIDAR_HEADER1) {
                state_ = STATE_WAIT_HEADER2;
            }
            break;
            
        case STATE_WAIT_HEADER2:
            if (byte == LIDAR_HEADER2) {
                state_ = STATE_WAIT_COMMAND;
            } else {
                state_ = STATE_WAIT_HEADER1;
                return false; // 触发重新开始DMA接收
            }
            break;
            
        case STATE_WAIT_COMMAND:
            if (byte == LIDAR_CMD_POSE || byte == LIDAR_CMD_IMU) {
                current_cmd_ = byte;
                payload_index_ = 0;
                state_ = STATE_RECEIVE_PAYLOAD;
            } else {
                state_ = STATE_WAIT_HEADER1;
                return false; // 触发重新开始DMA接收
            }
            break;
            
        case STATE_RECEIVE_PAYLOAD:
            payload_[payload_index_++] = byte;
            if (payload_index_ >= LIDAR_PAYLOAD_SIZE) {
                state_ = STATE_WAIT_FOOTER;
            }
            break;
            
        case STATE_WAIT_FOOTER:
            if (byte == LIDAR_FOOTER) {
                // 解析完整数据包
                if (current_cmd_ == LIDAR_CMD_POSE) {
                    parsePosePacket();
                } else if (current_cmd_ == LIDAR_CMD_IMU) {
                    parseImuPacket();
                }
            } else {
                error_count_++;
            }
            state_ = STATE_WAIT_HEADER1;
            break;
    }
    
    return true;
}

// 解析雷达位姿数据包
bool Lidar::parsePosePacket() {
    if (payload_index_ != LIDAR_PAYLOAD_SIZE) {
        error_count_++;
        return false;
    }

    // 解析位置数据
    LidarPoseData new_pose;
    new_pose.x = bytesToFloat(&payload_[0]);
    new_pose.y = bytesToFloat(&payload_[4]);
    new_pose.z = bytesToFloat(&payload_[8]);
    new_pose.valid = true;

    // 更新位置滤波器
    updatePositionFilter(new_pose.x, new_pose.y, new_pose.z);

    // 获取滤波后的位置
    float filtered_x = getFilteredPosition(x_filter_buffer_);
    float filtered_y = getFilteredPosition(y_filter_buffer_);
    float filtered_z = getFilteredPosition(z_filter_buffer_);

    // 使用固定频率计算速度
    if (velocity_initialized_ && pose_data_.valid && filter_count_ >= 2) {
        // 使用位姿数据频率计算时间间隔
        float dt_seconds = 1.0f / pose_frequency_hz_;
        
        // 计算原始速度
        float vx_raw = (filtered_x - last_pose_.x) / dt_seconds;
        float vy_raw = (filtered_y - last_pose_.y) / dt_seconds;
        float vz_raw = (filtered_z - last_pose_.z) / dt_seconds;

        // 应用一阶低通滤波器
        if (!velocity_lowpass_initialized_) {
            // 第一次初始化，直接使用原始速度
            velocity_data_.vx_filtered = vx_raw;
            velocity_data_.vy_filtered = vy_raw;
            velocity_data_.vz_filtered = vz_raw;
            velocity_lowpass_initialized_ = true;
        } else {
            // 应用低通滤波
            velocity_data_.vx_filtered = applyLowpassFilter(vx_raw, vx_lowpass_prev_, velocity_lowpass_alpha_);
            velocity_data_.vy_filtered = applyLowpassFilter(vy_raw, vy_lowpass_prev_, velocity_lowpass_alpha_);
            velocity_data_.vz_filtered = applyLowpassFilter(vz_raw, vz_lowpass_prev_, velocity_lowpass_alpha_);
        }

        // 保存当前滤波后的速度用于下次滤波
        vx_lowpass_prev_ = velocity_data_.vx_filtered;
        vy_lowpass_prev_ = velocity_data_.vy_filtered;
        vz_lowpass_prev_ = velocity_data_.vz_filtered;
        
        velocity_data_.valid = true;
    }

    // 保存上一次滤波后的位置
    last_pose_.x = filtered_x;
    last_pose_.y = filtered_y;
    last_pose_.z = filtered_z;
    last_pose_.valid = true;
    
    // 更新当前位置数据（使用原始位置）
    pose_data_ = new_pose;
    pose_packet_count_++;

    if (!velocity_initialized_) {
        velocity_initialized_ = true;
    }

    // 调用位姿数据接收回调函数
    if (pose_rx_callback_) {
        pose_rx_callback_(&pose_data_);
    }

    return true;
}

void Lidar::updatePositionFilter(float x, float y, float z) {
    // 更新滤波器缓冲区
    x_filter_buffer_[filter_index_] = x;
    y_filter_buffer_[filter_index_] = y;
    z_filter_buffer_[filter_index_] = z;

    // 更新索引和计数
    filter_index_ = (filter_index_ + 1) % LIDAR_POSITION_FILTER_SIZE;
    if (filter_count_ < LIDAR_POSITION_FILTER_SIZE) {
        filter_count_++;
    }
}

float Lidar::getFilteredPosition(const float* buffer) const {
    if (filter_count_ == 0) {
        return 0.0f;
    }

    // 如果滤波器大小小于5，使用原来的简单平均值
    if (LIDAR_POSITION_FILTER_SIZE < 5 || filter_count_ < 5) {
        float sum = 0.0f;
        for (uint8_t i = 0; i < filter_count_; i++) {
            sum += buffer[i];
        }
        return sum / filter_count_;
    }

    // 当滤波器大小大于等于5时，去掉最大值和最小值后求平均
    float min_val = buffer[0];
    float max_val = buffer[0];
    float sum = 0.0f;

    // 找到最大值和最小值，同时计算总和
    for (uint8_t i = 0; i < filter_count_; i++) {
        float val = buffer[i];
        sum += val;
        if (val < min_val) {
            min_val = val;
        }
        if (val > max_val) {
            max_val = val;
        }
    }

    // 去掉一个最大值和一个最小值
    sum -= max_val;
    sum -= min_val;

    // 返回剩余数据的平均值
    return sum / (filter_count_ - 2);
}

// 解析IMU数据包
bool Lidar::parseImuPacket() {
    imu_data_.roll = bytesToFloat(&payload_[0]);
    imu_data_.pitch = bytesToFloat(&payload_[4]);
    imu_data_.yaw = bytesToFloat(&payload_[8]);
    imu_data_.valid = true;
    
    imu_packet_count_++;

    // 调用IMU数据接收回调函数
    if (imu_rx_callback_) {
        imu_rx_callback_(&imu_data_);
    }

    return true;
}

// 字节数组到float转换 (小端序)
float Lidar::bytesToFloat(const uint8_t* bytes) {
    float val;
    memcpy(&val, bytes, sizeof(float));
    return val;
}

// 一阶低通滤波器实现
float Lidar::applyLowpassFilter(float current_value, float previous_filtered, float alpha) const {
    // 一阶低通滤波器公式: y[n] = α * x[n] + (1-α) * y[n-1]
    // α是滤波系数，范围0-1，越小滤波效果越强
    return alpha * current_value + (1.0f - alpha) * previous_filtered;
}

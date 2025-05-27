#include "lidar.h"
#include "cmsis_os.h"
#include <string.h>
#include "time_utils.h"

// 构造函数
Lidar::Lidar(UART_HandleTypeDef* huart)
    : huart_(huart),
      running_(false),
      pose_packet_count_(0),
      imu_packet_count_(0),
      error_count_(0),
      state_(STATE_WAIT_HEADER1),
      current_cmd_(0),
      payload_index_(0),
      last_pose_timestamp_(0),
      velocity_initialized_(false),
      filter_index_(0),
      filter_count_(0),
      pose_rx_callback_(nullptr),
      imu_rx_callback_(nullptr) {
    memset(&pose_data_, 0, sizeof(LidarPoseData));
    memset(&velocity_data_, 0, sizeof(LidarVelocityData));
    memset(&imu_data_, 0, sizeof(LidarImuData));
    memset(&last_pose_, 0, sizeof(LidarPoseData));
    memset(vx_filter_buffer_, 0, sizeof(vx_filter_buffer_));
    memset(vy_filter_buffer_, 0, sizeof(vy_filter_buffer_));
    memset(vz_filter_buffer_, 0, sizeof(vz_filter_buffer_));
    
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
    return startRxInterrupt(); // 启动单字节接收
}

// 停止设备
void Lidar::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    // 根据实际使用的接收方式停止
    // HAL_UART_AbortReceive_IT(huart_); // 若使用HAL_UART_Receive_IT
    HAL_UART_Abort(huart_); // 通用停止，包括DMA
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

// 重新启动单字节中断接收
bool Lidar::startRxInterrupt() {
    if (!running_) {
        return false;
    }
    // 清除可能存在的错误标志和中断标志
    __HAL_UART_CLEAR_OREFLAG(huart_);
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    // 开启UART的接收非空中断 (RXNE)
    // HAL_UART_Receive_IT 会自动使能RXNEIE
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
    return (status == HAL_OK);
}

// 单字节接收完成回调，在HAL_UART_RxCpltCallback中调用
void Lidar::byteRxCallback(UART_HandleTypeDef *huart) {
    if (huart != huart_ || !running_) {
        // if (running_ && huart == huart_) { // 如果是当前UART且在运行，则尝试重启接收
        //      startRxInterrupt();
        // }
        return;
    }
    
    processByte(rx_byte_);
    
    if (running_) {
        // 继续下一次单字节接收
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
        if (status != HAL_OK) {
            // 错误处理，例如记录错误，尝试重启
            error_count_++;
            // 尝试延迟后重启
            // osDelay(10);
            // startRxInterrupt();
        }
    }
}


// 批量数据接收回调（例如UART IDLE中断），在UART IDLE中断处理函数中调用
void Lidar::rxCallback(UART_HandleTypeDef *huart, uint8_t* pData, uint16_t Size) {
    if (huart != huart_ || !running_) {
        return;
    }
    processReceivedData(pData, Size);
    // 如果使用DMA + IDLE, 在这里重新启动DMA接收
    // HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer_dma_, DMA_BUFFER_SIZE);
    // __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
}

// 处理批量接收的数据
void Lidar::processReceivedData(uint8_t* pData, uint16_t Size) {
    for (uint16_t i = 0; i < Size; ++i) {
        if (!processByte(pData[i])) {
            // 如果processByte指示了一个解析错误并重置了状态机，
            // 可能需要根据具体情况决定是否继续处理剩余字节。
            // 当前实现是逐字节处理。
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
            }
            break;
            
        case STATE_WAIT_COMMAND:
            if (byte == LIDAR_CMD_POSE || byte == LIDAR_CMD_IMU) {
                current_cmd_ = byte;
                payload_index_ = 0;
                state_ = STATE_RECEIVE_PAYLOAD;
            } else {
                state_ = STATE_WAIT_HEADER1;
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

    // 获取当前时间戳
    timestamp_t current_timestamp = utils::time::TimeStamp::now();

    // 计算速度
    if (velocity_initialized_ && pose_data_.valid) {
        float dt_seconds = utils::time::TimeStamp::toSecondsFloat(
            utils::time::TimeStamp::diff(current_timestamp, last_pose_timestamp_));
        
        if (dt_seconds > 0.001f) { // 防止除零，最小时间间隔1ms
            float vx_raw = (new_pose.x - last_pose_.x) / dt_seconds;
            float vy_raw = (new_pose.y - last_pose_.y) / dt_seconds;
            float vz_raw = (new_pose.z - last_pose_.z) / dt_seconds;

            // 更新滤波器
            updateVelocityFilter(vx_raw, vy_raw, vz_raw);

            // 保存速度数据
            velocity_data_.vx_raw = vx_raw;
            velocity_data_.vy_raw = vy_raw;
            velocity_data_.vz_raw = vz_raw;
            velocity_data_.vx_filtered = getFilteredVelocity(vx_filter_buffer_);
            velocity_data_.vy_filtered = getFilteredVelocity(vy_filter_buffer_);
            velocity_data_.vz_filtered = getFilteredVelocity(vz_filter_buffer_);
            velocity_data_.valid = true;
        }
    }

    // 保存上一次位置和时间戳
    last_pose_ = pose_data_;
    last_pose_timestamp_ = current_timestamp;    // 更新当前位置数据
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

void Lidar::updateVelocityFilter(float vx, float vy, float vz) {
    // 更新滤波器缓冲区
    vx_filter_buffer_[filter_index_] = vx;
    vy_filter_buffer_[filter_index_] = vy;
    vz_filter_buffer_[filter_index_] = vz;

    // 更新索引和计数
    filter_index_ = (filter_index_ + 1) % LIDAR_VELOCITY_FILTER_SIZE;
    if (filter_count_ < LIDAR_VELOCITY_FILTER_SIZE) {
        filter_count_++;
    }
}

float Lidar::getFilteredVelocity(const float* buffer) const {
    if (filter_count_ == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < filter_count_; i++) {
        sum += buffer[i];
    }
    return sum / filter_count_;
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

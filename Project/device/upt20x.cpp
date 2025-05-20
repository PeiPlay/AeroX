#include "upt20x.h"
#include "cmsis_os.h"  // 用于系统时间获取

// 构造函数
UPT20X::UPT20X(UART_HandleTypeDef* huart)
    : huart_(huart),
      running_(false),
      packet_count_(0),
      error_count_(0),
      last_update_time_(0),
      last_stats_update_time_(0),
      update_freq_(0.0f),
      state_(STATE_WAIT_HEADER),
      payload_index_(0),
      calculated_checksum_(0),
      watchdog_(nullptr)
{
    // 清零光流数据
    memset(&flow_data_, 0, sizeof(OpticalFlowData));
}

// 析构函数
UPT20X::~UPT20X() {
    stop();
    
    // 释放看门狗资源
    stopWatchdog();
    if (watchdog_) {
        delete watchdog_;
        watchdog_ = nullptr;
    }
}

// 初始化设备
bool UPT20X::init() {
    // 初始化成员变量
    state_ = STATE_WAIT_HEADER;
    payload_index_ = 0;
    calculated_checksum_ = 0;
    
    // 重置统计数据
    packet_count_ = 0;
    error_count_ = 0;
    last_update_time_ = osKernelGetTickCount();
    last_stats_update_time_ = last_update_time_;
    update_freq_ = 0.0f;
    
    return true;
}

// 启动设备
bool UPT20X::start() {
    if (running_) {
        return true;  // 已经在运行
    }
    
    running_ = true;
    
    // 启动首次中断接收
    return startRxInterrupt();
}

// 停止设备
void UPT20X::stop() {
    if (!running_) {
        return;  // 已经停止
    }
    
    running_ = false;
    
    // 中止串口接收
    HAL_UART_AbortReceive(huart_);
}

// 获取最新光流数据
OpticalFlowData UPT20X::getData() const {
    return flow_data_;  // 直接返回，由中断上下文更新
}

// 复位累计位移
void UPT20X::resetAccumulation() {
    // 此函数保留为空，维持接口一致性
}

// 重新启动中断接收
bool UPT20X::startRxInterrupt() {
    if (!running_) {
        return false;
    }
    __HAL_UART_CLEAR_IDLEFLAG(huart_); // 清除空闲中断标志位
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE); // 开启UART的空闲中断

    // 启动单字节接收
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
    return (status == HAL_OK);
}

// 接收回调函数，在HAL_UART_RxCpltCallback中调用
void UPT20X::rxCallback(UART_HandleTypeDef *huart) {
    // 检查是否是本模块的串口触发了中断
    if (huart != huart_) {
        return;
    }
    
    // 处理接收到的字节
    processByte(rx_byte_);
    __HAL_UART_CLEAR_IDLEFLAG(huart_); // 清除空闲中断标志位

    // 定期更新统计信息（大约每秒一次）
    uint32_t now = osKernelGetTickCount();
    if (now - last_stats_update_time_ >= 1000) {
        updateStatistics();
        last_stats_update_time_ = now;
    }
    
    // 继续下一次接收
    if (running_) {
        startRxInterrupt();
    }
}

// 处理单个字节
bool UPT20X::processByte(uint8_t byte) {
    __HAL_UART_CLEAR_FLAG(huart_, UART_FLAG_RXNE);

    // 基于当前状态处理字节
    switch (state_) {
        case STATE_WAIT_HEADER:
            if (byte == UPT20X_HEADER) {
                state_ = STATE_WAIT_LENGTH;
            }
            break;
            
        case STATE_WAIT_LENGTH:
            if (byte == UPT20X_LENGTH) {
                state_ = STATE_RECEIVE_PAYLOAD;
                payload_index_ = 0;
                calculated_checksum_ = 0;  // 重置校验和
            } else {
                // 长度错误，返回等待包头状态
                state_ = STATE_WAIT_HEADER;
                error_count_++;
            }
            break;
            
        case STATE_RECEIVE_PAYLOAD:
            // 保存有效负载并更新校验和
            payload_[payload_index_] = byte;
            calculated_checksum_ ^= byte;  // XOR校验
            
            payload_index_++;
            if (payload_index_ >= UPT20X_LENGTH) {
                state_ = STATE_WAIT_CHECKSUM;
            }
            break;
            
        case STATE_WAIT_CHECKSUM:
            // 检验校验和
            if (byte == calculated_checksum_) {
                state_ = STATE_WAIT_FOOTER;
            } else {
                // 校验和错误，返回等待包头状态
                state_ = STATE_WAIT_HEADER;
                error_count_++;
            }
            break;
            
        case STATE_WAIT_FOOTER:
            if (byte == UPT20X_FOOTER) {
                // 成功接收一个完整数据包
                if (parsePacket()) {
                    packet_count_++;
                    last_update_time_ = osKernelGetTickCount();
                    return true;
                } else {
                    error_count_++;
                }
            } else {
                error_count_++;
            }
            
            // 无论包尾是否正确，都返回等待包头状态
            state_ = STATE_WAIT_HEADER;
            break;
    }
    
    return false;
}

// 解析数据包
bool UPT20X::parsePacket() {
    // 从有效负载中提取数据
    int16_t flow_x_integral = payload_[0] | (payload_[1] << 8);  // 小端序
    int16_t flow_y_integral = payload_[2] | (payload_[3] << 8);
    uint16_t integration_timespan = payload_[4] | (payload_[5] << 8);
    uint16_t laser_distance = payload_[6] | (payload_[7] << 8);
    uint8_t valid = payload_[8];
    uint8_t confidence = payload_[9];
    
    // 更新光流数据
    flow_data_.flow_x = flow_x_integral / 10000.0f;
    flow_data_.flow_y = flow_y_integral / 10000.0f;
    flow_data_.integration_time = integration_timespan;
    flow_data_.distance = laser_distance;
    flow_data_.valid = (valid == UPT20X_VALID_FLAG);
    flow_data_.confidence = confidence;
    
    // 如果数据有效，喂狗
    if (flow_data_.valid && watchdog_ && watchdog_->isActive()) {
        watchdog_->feed();
    }
    
    return true;
}

// 更新统计数据
void UPT20X::updateStatistics() {
    // 计算每秒更新频率
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed = now - last_update_time_;
    
    // 防止除零
    if (elapsed > 0) {
        update_freq_ = 1000.0f / elapsed;
    } else {
        update_freq_ = 0.0f;
    }
}

// 配置看门狗
bool UPT20X::configWatchdog(uint32_t timeout_ms, std::function<void()> callback) {
    // 如果已经存在看门狗，先释放资源
    if (watchdog_) {
        stopWatchdog();
        delete watchdog_;
        watchdog_ = nullptr;
    }
    
    // 设置回调函数
    if (callback) {
        watchdog_callback_ = callback;
    } else {
        // 使用默认回调
        watchdog_callback_ = [this]() { this->defaultWatchdogCallback(); };
    }
    
    // 创建新的看门狗实例
    watchdog_ = new (std::nothrow) Watchdog(timeout_ms, watchdog_callback_);
    
    // 检查是否成功创建
    return (watchdog_ != nullptr);
}

// 启动看门狗监控
bool UPT20X::startWatchdog(WatchdogManager& manager) {
    if (!watchdog_) {
        // 如果未配置看门狗，使用默认参数配置一个
        if (!configWatchdog(2000)) {
            return false;
        }
    }
    
    // 启动看门狗并喂一次狗
    bool result = watchdog_->start(manager);
    if (result) {
        watchdog_->feed();
    }
    
    return result;
}

// 停止看门狗监控
void UPT20X::stopWatchdog() {
    if (watchdog_) {
        watchdog_->stop();
    }
}

// 默认看门狗超时回调
void UPT20X::defaultWatchdogCallback() {
    // 记录错误
    error_count_++;
    
    // 尝试重启设备
    if (running_) {
        // 停止当前运行
        HAL_UART_AbortReceive(huart_);
        
        // 等待一会儿
        osDelay(100);
        
        // 重置状态机
        state_ = STATE_WAIT_HEADER;
        payload_index_ = 0;
        calculated_checksum_ = 0;
        
        // 重新启动接收
        startRxInterrupt();
    }
}

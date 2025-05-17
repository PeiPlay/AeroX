#include "up_t201.h"

// 构造函数
UPT201::UPT201(UART_HandleTypeDef* huart, uint32_t stack_size, osPriority_t priority)
    : serial_(huart),
      thread_id_(nullptr),
      stack_size_(stack_size),
      priority_(priority),
      running_(false),
      packet_count_(0),
      error_count_(0),
      last_update_time_(0),
      update_freq_(0.0f),
      state_(STATE_WAIT_HEADER),
      payload_index_(0),
      calculated_checksum_(0),
      data_mutex_(nullptr),
      data_available_sem_(nullptr),
      watchdog_(nullptr)  // 初始化看门狗指针为空
{
    // 清零光流数据
    memset(&flow_data_, 0, sizeof(OpticalFlowData));
}

// 析构函数
UPT201::~UPT201() {
    stop();
    
    // 释放看门狗资源
    stopWatchdog();
    if (watchdog_) {
        delete watchdog_;
        watchdog_ = nullptr;
    }
    
    if (data_mutex_) {
        osMutexDelete(data_mutex_);
    }
    
    if (data_available_sem_) {
        osSemaphoreDelete(data_available_sem_);
    }
}

// 初始化设备
bool UPT201::init() {
    // 初始化串口流
    if (!serial_.init()) {
        return false;
    }
    
    // 创建互斥量
    osMutexAttr_t mutex_attr = {
        "UPT201Mutex",      // 名称
        osMutexPrioInherit, // 属性: 优先级继承，防止优先级反转
        NULL,               // 内存区块
        0                   // 大小
    };
    
    data_mutex_ = osMutexNew(&mutex_attr);
    if (!data_mutex_) {
        return false;
    }
    
    // 创建数据可用信号量
    osSemaphoreAttr_t sem_attr = {
        "UPT201DataSem",    // 名称
        0,                  // 属性
        NULL,               // 内存区块
        0                   // 大小
    };
    
    data_available_sem_ = osSemaphoreNew(1, 0, &sem_attr); // 初始计数为0，最大为1
    if (!data_available_sem_) {
        osMutexDelete(data_mutex_);
        data_mutex_ = nullptr;
        return false;
    }
    
    return true;
}

// 启动设备
bool UPT201::start() {
    if (running_) {
        return true;  // 已经在运行
    }
    
    // 启动串口接收
    if (!serial_.begin()) {
        return false;
    }
    
    // 重置状态机
    state_ = STATE_WAIT_HEADER;
    payload_index_ = 0;
    calculated_checksum_ = 0;
    
    // 重置统计数据
    packet_count_ = 0;
    error_count_ = 0;
    last_update_time_ = osKernelGetTickCount();
    update_freq_ = 0.0f;
    
    // 创建处理线程 - 修正栈大小和优先级设置
    osThreadAttr_t thread_attr;
    memset(&thread_attr, 0, sizeof(thread_attr));  // 确保完全初始化
    
    thread_attr.name = "UPT201Thread";
    thread_attr.stack_size = stack_size_ / 4;  // 转换为word单位
    thread_attr.priority = priority_;
    
    running_ = true;  // 设置运行标志
    
    thread_id_ = osThreadNew(threadFunction, this, &thread_attr);
    if (!thread_id_) {
        running_ = false;
        serial_.stop();
        return false;
    }
    
    return true;
}

// 停止设备
void UPT201::stop() {
    if (!running_) {
        return;  // 已经停止
    }
    
    running_ = false;  // 清除运行标志，线程将会退出
    
    // 等待线程完全终止
    if (thread_id_ != nullptr) {
        // 给线程一定时间退出
        osDelay(100);
        thread_id_ = nullptr;
    }
    
    // 停止串口接收
    serial_.stop();
}

// 获取最新光流数据
OpticalFlowData UPT201::getData() const {
    OpticalFlowData data;
    
    // 使用互斥锁保护数据访问
    if (osMutexAcquire(data_mutex_, osWaitForever) == osOK) {
        data = flow_data_;  // 拷贝最新数据
        osMutexRelease(data_mutex_);
    } else {
        memset(&data, 0, sizeof(OpticalFlowData));  // 出错时返回全零数据
    }
    
    return data;
}

// 获取数据更新频率
float UPT201::getUpdateFrequency() const {
    return update_freq_;
}

// 线程函数（静态，需要调用实例方法）
void UPT201::threadFunction(void* argument) {
    UPT201* instance = static_cast<UPT201*>(argument);
    if (instance) {
        instance->processSerialData();
    }
    osThreadExit();  // 确保线程正常退出
}

// 处理串口数据
void UPT201::processSerialData() {
    uint8_t byte;
    uint32_t last_statistics_time = osKernelGetTickCount();
    
    while (running_) {
        // 读取一个字节的数据（带100ms超时，确保可以检查running_标志）
        if (!serial_.read(&byte, 100)) {
            continue;  // 超时，继续检查running_标志
        }
        
        // 基于当前状态处理字节
        switch (state_) {
            case STATE_WAIT_HEADER:
                if (byte == UP_T201_HEADER) {
                    state_ = STATE_WAIT_LENGTH;
                }
                break;
                
            case STATE_WAIT_LENGTH:
                if (byte == UP_T201_LENGTH) {
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
                if (payload_index_ >= UP_T201_LENGTH) {
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
                if (byte == UP_T201_FOOTER) {
                    // 成功接收一个完整数据包
                    if (parsePacket()) {
                        packet_count_++;
                        last_update_time_ = osKernelGetTickCount();
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
        
        // 每秒更新一次统计数据
        uint32_t now = osKernelGetTickCount();
        if (now - last_statistics_time >= 1000) {
            updateStatistics();
            last_statistics_time = now;
        }
    }
}

// 解析数据包
bool UPT201::parsePacket() {
    // 从有效负载中提取数据
    int16_t flow_x_integral = payload_[0] | (payload_[1] << 8);  // 小端序
    int16_t flow_y_integral = payload_[2] | (payload_[3] << 8);
    uint16_t integration_timespan = payload_[4] | (payload_[5] << 8);
    uint16_t laser_distance = payload_[6] | (payload_[7] << 8);
    uint8_t valid = payload_[8];
    uint8_t confidence = payload_[9];
    
    // 创建新的光流数据
    OpticalFlowData new_data;
    
    // 将整型值转换为浮点型（radians）
    new_data.flow_x = flow_x_integral / 10000.0f;
    new_data.flow_y = flow_y_integral / 10000.0f;
    new_data.integration_time = integration_timespan;
    new_data.distance = laser_distance;
    new_data.valid = (valid == UP_T201_VALID_FLAG);
    new_data.confidence = confidence;
    
    // 如果数据有效，喂狗
    if (new_data.valid && watchdog_ && watchdog_->isActive()) {
        watchdog_->feed();
    }
    
    // 互斥访问，更新共享数据
    if (osMutexAcquire(data_mutex_, osWaitForever) == osOK) {
        flow_data_ = new_data;
        osMutexRelease(data_mutex_);
        
        // 释放信号量通知等待的线程有新数据
        if (new_data.valid) {
            osSemaphoreRelease(data_available_sem_);
        }
        
        return true;
    }
    
    return false;
}

// 等待新的光流数据
bool UPT201::waitForData(uint32_t timeout_ms) {
    if (!running_ || !data_available_sem_) {
        return false;
    }
    
    // 等待数据可用信号量
    return (osSemaphoreAcquire(data_available_sem_, timeout_ms) == osOK);
}

// 复位累计位移
void UPT201::resetAccumulation() {
    // 这个函数可以为空，因为光流模块本身会处理积分
    // 但保留此函数作为接口，让用户可以在需要时重置光流计算
}

// 更新统计数据
void UPT201::updateStatistics() {
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

// 看门狗相关函数实现

// 配置看门狗
bool UPT201::configWatchdog(uint32_t timeout_ms, std::function<void()> callback) {
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
bool UPT201::startWatchdog(WatchdogManager& manager) {
    if (!watchdog_) {
        // 如果未配置看门狗，使用默认参数配置一个
        // 默认2秒超时，使用默认回调
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
void UPT201::stopWatchdog() {
    if (watchdog_) {
        watchdog_->stop();
    }
}

// 默认看门狗超时回调
void UPT201::defaultWatchdogCallback() {
    // 在这里处理超时情况，比如尝试重启设备或记录错误
    // 注意：这个回调在看门狗管理器线程中执行，不要执行长时间阻塞操作
    
    // 记录错误
    error_count_++;
    
    // 尝试重启设备
    if (running_) {
        // 停止当前运行
        serial_.stop();
        
        // 等待一会儿
        osDelay(100);
        
        // 重新启动串口
        serial_.begin();
        
        // 重置状态机
        state_ = STATE_WAIT_HEADER;
        payload_index_ = 0;
        calculated_checksum_ = 0;
    }
}

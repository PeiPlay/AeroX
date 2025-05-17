#ifndef SERIAL_STREAM_H
#define SERIAL_STREAM_H
#include "main.h" 
#include "cmsis_os.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// 阻塞回调函数类型定义，当用户消耗太慢时调用
typedef void (*BlockingCallback)(void* context);
#pragma pack(push, 1)
template<size_t BUFFER_SIZE = 128>
class SerialStream {
public:
    // 构造函数 - 仅设置参数，不分配资源
    SerialStream(UART_HandleTypeDef* huart);
    ~SerialStream();
    
    // 初始化方法 - 在RTOS内核启动后调用，用于初始化资源
    bool init();
    
    // 启动DMA接收
    bool begin();
    
    // 停止接收
    void stop();
    
    // 注册阻塞回调函数（可选）
    void registerBlockingCallback(BlockingCallback callback, void* context = nullptr);
    
    // 获取UART句柄（用于ISR中）
    UART_HandleTypeDef* getUartHandle() const { return huart_; }
    
    // UART空闲中断回调（在ISR中调用）
    // 添加UART句柄参数，用于确保回调的是正确的串口
    void idleCallback(UART_HandleTypeDef *huart);
    
    // 等待新数据可用（带超时，FreeRTOS任务中使用）
    bool waitForData(uint32_t timeout_ms);
    
    // 从用户缓冲区读取一个字节（主要用户接口）
    // 会一直阻塞等待直到有可用数据
    uint8_t read();
    
    // 重载版本：将数据读入指定缓冲区，带超时
    // 返回值: true表示成功读取数据，false表示超时
    bool read(uint8_t* data, uint32_t timeout_ms);
    
    // 判断是否有可读数据
    bool available();
    
private:
    UART_HandleTypeDef* huart_;  // UART句柄
    
    // 双缓冲区 - 静态分配
    uint8_t rx_buffer_[2][BUFFER_SIZE];  // 双缓冲区数组
    uint32_t buffer_len_[2];             // 每个缓冲区中有效数据长度
    
    volatile uint8_t recv_buffer_idx_;   // 当前接收缓冲区索引
    volatile uint8_t user_buffer_idx_;   // 当前用户读取缓冲区索引
    
    // 用户读取位置
    volatile uint32_t read_pos_;
    
    // 数据可用信号量
    osSemaphoreId_t data_ready_sem_;
    
    // 阻塞回调
    BlockingCallback blocking_callback_;
    void* blocking_context_;
    
    // 初始化状态
    bool initialized_;
    
    // 内部方法
    void switchBuffers();         // 切换双缓冲区
    bool isUserBufferReady() const;  // 检查用户缓冲区是否就绪（已消耗完）
};

// 模板类实现部分
template<size_t BUFFER_SIZE>
SerialStream<BUFFER_SIZE>::SerialStream(UART_HandleTypeDef* huart)
    : huart_(huart),
      buffer_len_{0, 0},
      recv_buffer_idx_(0), 
      user_buffer_idx_(1), 
      read_pos_(0),
      data_ready_sem_(nullptr),
      blocking_callback_(nullptr), 
      blocking_context_(nullptr),
      initialized_(false) {
    // 构造函数仅设置初始参数
    memset(rx_buffer_[0], 0, BUFFER_SIZE);
    memset(rx_buffer_[1], 0, BUFFER_SIZE);
}

template<size_t BUFFER_SIZE>
SerialStream<BUFFER_SIZE>::~SerialStream() {
    stop();
    
    if (initialized_) {
        // 删除信号量
        if (data_ready_sem_) osSemaphoreDelete(data_ready_sem_);
    }
}

template<size_t BUFFER_SIZE>
bool SerialStream<BUFFER_SIZE>::init() {
    if (initialized_ || huart_ == nullptr) {
        return false;
    }
    
    // 创建信号量
    osSemaphoreAttr_t sem_attr = {
        "SerialStreamSem", // 名称
        0,                 // 属性
        NULL,              // 内存区块
        0                  // 大小
    };
    data_ready_sem_ = osSemaphoreNew(1, 0, &sem_attr); // 初始计数为0，最大为1
    
    if (!data_ready_sem_) {
        return false;
    }
    
    // 设置初始状态
    buffer_len_[0] = 0;
    buffer_len_[1] = 0;
    recv_buffer_idx_ = 0;
    user_buffer_idx_ = 1;
    read_pos_ = 0;
    
    initialized_ = true;
    return true;
}

template<size_t BUFFER_SIZE>
bool SerialStream<BUFFER_SIZE>::begin() {
    if (!initialized_) {
        return false;
    }
    
    // 重置状态
    recv_buffer_idx_ = 0;
    user_buffer_idx_ = 1;  // 初始时用户没有可读数据
    read_pos_ = 0;
    buffer_len_[0] = 0;
    buffer_len_[1] = 0;
    
    // 清空信号量
    while(osSemaphoreAcquire(data_ready_sem_, 0) == osOK);
    
    HAL_UART_AbortReceive(huart_);

    taskENTER_CRITICAL();

    HAL_UART_AbortReceive(huart_);
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    HAL_UART_DMAStop(huart_);
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    HAL_UART_AbortReceive(huart_);
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer_[recv_buffer_idx_], BUFFER_SIZE);
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE); // 启用IDLE中断

    taskEXIT_CRITICAL();
    return true;
}

template<size_t BUFFER_SIZE>
void SerialStream<BUFFER_SIZE>::stop() {
    // 禁用IDLE中断
    __HAL_UART_DISABLE_IT(huart_, UART_IT_IDLE);
    
    // 停止DMA接收
    HAL_UART_AbortReceive(huart_);
}

template<size_t BUFFER_SIZE>
void SerialStream<BUFFER_SIZE>::registerBlockingCallback(BlockingCallback callback, void* context) {
    blocking_callback_ = callback;
    blocking_context_ = context;
}

template<size_t BUFFER_SIZE>
bool SerialStream<BUFFER_SIZE>::isUserBufferReady() const {
    // 用户缓冲区已就绪 = 当前读取位置已达到缓冲区末尾
    return read_pos_ >= buffer_len_[user_buffer_idx_];
}

template<size_t BUFFER_SIZE>
void SerialStream<BUFFER_SIZE>::idleCallback(UART_HandleTypeDef *huart) {
    // 检查传入的串口是否与当前实例匹配
    if (!initialized_ || (huart->Instance != huart_->Instance)) {
        return;
    }
    
    // 先停止DMA接收
    HAL_UART_DMAStop(huart_);
    HAL_UART_AbortReceive(huart_);
    __HAL_UART_CLEAR_IDLEFLAG(huart_);

    // 获取已接收的数据长度
    uint32_t recv_len = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    
    if (recv_len == 0) {
        // 没有收到数据，重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer_[recv_buffer_idx_], BUFFER_SIZE);
        __HAL_UART_CLEAR_IDLEFLAG(huart_);
        return;
    }
    
    // 保存接收到的数据长度
    buffer_len_[recv_buffer_idx_] = recv_len;
    
    osStatus_t result = osOK;
    
    if (isUserBufferReady()) {
        // 用户已经消耗完缓冲区，可以切换
        
        // 切换用户缓冲区和接收缓冲区
        switchBuffers();
        
        // 重置用户读取位置
        read_pos_ = 0;
        
        // 通知用户任务有新数据可用
        result = osSemaphoreRelease(data_ready_sem_);
    } else {
        // 用户尚未消耗完缓冲区
        
        if (blocking_callback_ != nullptr) {
            // 调用用户提供的阻塞函数
            blocking_callback_(blocking_context_);
        } else {
            // 没有阻塞回调，则重新接收到当前缓冲区
            buffer_len_[recv_buffer_idx_] = 0;  // 清除旧数据
        }
    }
    
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer_[recv_buffer_idx_], BUFFER_SIZE);
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE); // 启用IDLE中断
}

template<size_t BUFFER_SIZE>
void SerialStream<BUFFER_SIZE>::switchBuffers() {
    // 交换接收缓冲区和用户缓冲区索引
    uint8_t temp = recv_buffer_idx_;
    recv_buffer_idx_ = user_buffer_idx_;
    user_buffer_idx_ = temp;
}

template<size_t BUFFER_SIZE>
bool SerialStream<BUFFER_SIZE>::waitForData(uint32_t timeout_ms) {
    // 等待数据可用信号量
    return osSemaphoreAcquire(data_ready_sem_, timeout_ms) == osOK;
}

template<size_t BUFFER_SIZE>
bool SerialStream<BUFFER_SIZE>::available() {
    // 判断是否有可读数据 = 当前读取位置小于缓冲区数据长度
    return (read_pos_ < buffer_len_[user_buffer_idx_]);
}

template<size_t BUFFER_SIZE>
uint8_t SerialStream<BUFFER_SIZE>::read() {
    if (!initialized_) {
        return 0; // 返回0作为错误值
    }
    
    // 如果没有可用数据，等待信号量（无超时，阻塞直到有数据）
    while (!available()) {
        // 等待数据，永久阻塞
        waitForData(osWaitForever);
    }
    
    // 此时必定有数据可读
    uint8_t result = rx_buffer_[user_buffer_idx_][read_pos_];
    read_pos_++;
    
    return result;
}

template<size_t BUFFER_SIZE>
bool SerialStream<BUFFER_SIZE>::read(uint8_t* data, uint32_t timeout_ms) {
    if (!data) {
        return false;  // 无效参数
    }
    
    // 如果没有可用数据，等待信号量（有超时）
    if (!available()) {
        if (!waitForData(timeout_ms)) {
            return false;  // 等待超时
        }
    }
    
    // 此时必定有数据可读
    *data = rx_buffer_[user_buffer_idx_][read_pos_];
    read_pos_++;
    return true;
}

#pragma pack(pop)
#endif // SERIAL_STREAM_H

#ifndef UP_T201_H
#define UP_T201_H

#include "serial_stream.h"
#include "cmsis_os.h"
#include <stdint.h>
#include "watchdog.h" // 添加看门狗头文件

// 数据包格式常量
#define UP_T201_HEADER         0xFE
#define UP_T201_FOOTER         0x55
#define UP_T201_LENGTH         0x0A
#define UP_T201_VALID_FLAG     0xF5
#define UP_T201_INVALID_FLAG   0x00
#define UP_T201_PACKET_SIZE    14

// 光流数据结构
struct OpticalFlowData {
    float flow_x;              // X方向光流位移，单位：radians
    float flow_y;              // Y方向光流位移，单位：radians
    uint32_t integration_time; // 积分时间，单位：微秒
    uint16_t distance;         // 距离，单位：毫米
    bool valid;                // 数据有效性标志
    uint8_t confidence;        // 置信度，百分比
    
    // 计算实际位移（需要乘以高度）
    float getDisplacementX(float height_mm) const {
        return (valid) ? (flow_x * height_mm) : 0.0f;
    }
    
    float getDisplacementY(float height_mm) const {
        return (valid) ? (flow_y * height_mm) : 0.0f;
    }
};

class UPT201 {
public:
    // 构造函数 - 仅设置参数，不分配资源
    // 增加默认栈大小至1024字节，确保足够空间
    UPT201(UART_HandleTypeDef* huart, uint32_t stack_size = 1024, osPriority_t priority = osPriorityNormal);
    ~UPT201();
    
    // 初始化方法 - 在RTOS内核启动后调用，用于初始化资源
    bool init();
    
    // 启动设备
    bool start();
    
    // 停止设备
    void stop();
    
    // 获取最新的光流数据
    OpticalFlowData getData() const;
    
    // 检查模块是否运行中
    bool isRunning() const { return running_; }
    
    // 获取数据更新频率（Hz）
    float getUpdateFrequency() const;

    // 等待新的光流数据（带超时，单位ms）
    // 返回值: true表示有新数据，false表示超时
    bool waitForData(uint32_t timeout_ms = osWaitForever);
    
    // 复位累计位移（用于外部调用重置光流位移计算）
    void resetAccumulation();

    void uartIdleCallback(UART_HandleTypeDef *huart) {
        serial_.idleCallback(huart);  // 调用串口流的空闲回调
    }
    
    // 新增: 看门狗相关功能
    /**
     * @brief 配置看门狗监控
     * @param timeout_ms 超时时间（毫秒）
     * @param callback 超时回调函数，默认为nullptr（使用内部默认回调）
     * @return 是否成功配置
     */
    bool configWatchdog(uint32_t timeout_ms, std::function<void()> callback = nullptr);
    
    /**
     * @brief 获取看门狗实例，用于外部管理
     * @return 看门狗指针，如未配置则返回nullptr
     */
    Watchdog* getWatchdog() const { return watchdog_; }
    
    /**
     * @brief 启动看门狗监控
     * @param manager 看门狗管理器
     * @return 是否成功启动
     */
    bool startWatchdog(WatchdogManager& manager);
    
    /**
     * @brief 停止看门狗监控
     */
    void stopWatchdog();
    
    /**
     * @brief 检查看门狗是否活动
     * @return 看门狗是否处于活动状态
     */
    bool isWatchdogActive() const { return watchdog_ && watchdog_->isActive(); }
    
private:
    // 串口通信
    SerialStream<128> serial_;
    
    // 线程参数
    osThreadId_t thread_id_;
    uint32_t stack_size_;
    osPriority_t priority_;
    volatile bool running_;
    
    // 数据缓存和处理
    OpticalFlowData flow_data_;
    
    // 统计数据
    uint32_t packet_count_;
    uint32_t error_count_;
    uint32_t last_update_time_;
    float update_freq_;
    
    // 解析状态机状态
    enum ParseState {
        STATE_WAIT_HEADER,
        STATE_WAIT_LENGTH,
        STATE_RECEIVE_PAYLOAD,
        STATE_WAIT_CHECKSUM,
        STATE_WAIT_FOOTER
    };
    
    ParseState state_;
    uint8_t payload_[10];  // 数据包有效负载
    uint8_t payload_index_;
    uint8_t calculated_checksum_;
    
    // 内部方法
    static void threadFunction(void* argument);  // 线程函数
    void processSerialData();   // 处理串口数据
    bool parsePacket();         // 解析数据包
    void updateStatistics();    // 更新统计数据
    
    // 对最新数据的互斥保护
    osMutexId_t data_mutex_;
    
    // 新数据可用信号量
    osSemaphoreId_t data_available_sem_;
    
    // 看门狗相关成员
    Watchdog* watchdog_;                       // 看门狗实例
    std::function<void()> watchdog_callback_;  // 自定义看门狗回调
    
    // 默认看门狗回调处理
    void defaultWatchdogCallback();
};

#endif // UP_T201_H

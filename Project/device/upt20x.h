#ifndef UPT20X_H
#define UPT20X_H
#include "main.h"
#include <stdint.h>
#include "watchdog.h"
#include <functional>

// 数据包格式常量
#define UPT20X_HEADER         0xFE
#define UPT20X_FOOTER         0x55
#define UPT20X_LENGTH         0x0A
#define UPT20X_VALID_FLAG     0xF5
#define UPT20X_INVALID_FLAG   0x00
#define UPT20X_PACKET_SIZE    14

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

class UPT20X {
public:
    // 构造函数，传入UART句柄
    UPT20X(UART_HandleTypeDef* huart);
    ~UPT20X();
    
    // 初始化
    bool init();
    
    // 启动接收
    bool start();
    
    // 停止接收
    void stop();
    
    // 获取最新的光流数据
    OpticalFlowData getData() const;
    
    // 检查模块是否运行中
    bool isRunning() const { return running_; }
    
    // 获取数据更新频率（Hz）
    float getUpdateFrequency() const { return update_freq_; }
    
    // 复位累计位移
    void resetAccumulation();
    
    // 接收回调函数，在HAL_UART_RxCpltCallback中调用
    void rxCallback(UART_HandleTypeDef *huart);
    
    // 看门狗相关功能
    bool configWatchdog(uint32_t timeout_ms, std::function<void()> callback = nullptr);
    Watchdog* getWatchdog() const { return watchdog_; }
    bool startWatchdog(WatchdogManager& manager);
    void stopWatchdog();
    bool isWatchdogActive() const { return watchdog_ && watchdog_->isActive(); }
    
private:
    // 串口控制
    UART_HandleTypeDef* huart_;
    volatile bool running_;
    
    // 接收缓冲区
    uint8_t rx_byte_;  // 单字节接收缓冲区
    
    // 数据缓存和处理
    OpticalFlowData flow_data_;
    
    // 统计数据
    uint32_t packet_count_;
    uint32_t error_count_;
    uint32_t last_update_time_;
    uint32_t last_stats_update_time_;
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
    bool processByte(uint8_t byte);  // 处理接收到的字节
    bool parsePacket();              // 解析数据包
    void updateStatistics();         // 更新统计数据
    
    // 重新启动接收
    bool startRxInterrupt();
    
    // 看门狗相关成员
    Watchdog* watchdog_;                       // 看门狗实例
    std::function<void()> watchdog_callback_;  // 自定义看门狗回调
    
    // 默认看门狗回调处理
    void defaultWatchdogCallback();
};

#endif // UPT20X_H

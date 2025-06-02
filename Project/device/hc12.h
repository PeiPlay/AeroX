#ifndef __HC12_H
#define __HC12_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* HC-12工作模式枚举 */
typedef enum {
    HC12_MODE_FU1 = 1,    // 较省电模式，空闲3.6mA
    HC12_MODE_FU2 = 2,    // 省电模式，空闲80μA
    HC12_MODE_FU3 = 3,    // 全速模式，空闲16mA(默认)
    HC12_MODE_FU4 = 4     // 超远距离模式，空闲16mA
} HC12_Mode_t;

/* HC-12发射功率等级枚举 */
typedef enum {
    HC12_POWER_1 = 1,     // -1dBm
    HC12_POWER_2 = 2,     // +2dBm
    HC12_POWER_3 = 3,     // +5dBm
    HC12_POWER_4 = 4,     // +8dBm
    HC12_POWER_5 = 5,     // +11dBm
    HC12_POWER_6 = 6,     // +14dBm
    HC12_POWER_7 = 7,     // +17dBm
    HC12_POWER_8 = 8      // +20dBm(默认)
} HC12_Power_t;

/* HC-12波特率枚举 */
typedef enum {
    HC12_BAUD_1200 = 1200,
    HC12_BAUD_2400 = 2400,
    HC12_BAUD_4800 = 4800,
    HC12_BAUD_9600 = 9600,      // 默认
    HC12_BAUD_19200 = 19200,
    HC12_BAUD_38400 = 38400,
    HC12_BAUD_57600 = 57600,
    HC12_BAUD_115200 = 115200
} HC12_Baud_t;

/* HC-12串口格式 */
typedef struct {
    uint8_t data_bits;      // 数据位：8
    char parity;           // 校验位：'N'无校验, 'O'奇校验, 'E'偶校验
    uint8_t stop_bits;     // 停止位：1,2,3(1.5位)
} HC12_UartFormat_t;

/* HC-12配置结构体 */
typedef struct {
    HC12_Mode_t mode;           // 工作模式
    HC12_Baud_t baud_rate;      // 波特率
    uint8_t channel;            // 通信频道(1-127)
    HC12_Power_t power;         // 发射功率等级
    HC12_UartFormat_t format;   // 串口格式
} HC12_Config_t;

/**
 * @brief HC-12无线串口通信模块类
 */
class HC12 {
private:
    UART_HandleTypeDef *huart;      // UART句柄
    GPIO_TypeDef *set_port;         // SET引脚端口
    uint16_t set_pin;               // SET引脚号
    HC12_Config_t config;           // 当前配置
    bool is_in_at_mode;             // 是否处于AT模式
    uint8_t rx_buffer[256];         // 接收缓冲区
    uint16_t rx_index;              // 接收索引
    
    // 私有成员函数
    HAL_StatusTypeDef sendATCommand(const char *cmd, char *response, uint16_t timeout);
    
public:
    // 构造函数和析构函数
    HC12(UART_HandleTypeDef *huart, GPIO_TypeDef *set_port, uint16_t set_pin);
    ~HC12();
    
    // 初始化和基本操作
    HAL_StatusTypeDef init();
    HAL_StatusTypeDef enterATMode();
    HAL_StatusTypeDef exitATMode();
    
    // 参数设置函数
    HAL_StatusTypeDef setMode(HC12_Mode_t mode);
    HAL_StatusTypeDef setBaudRate(HC12_Baud_t baud);
    HAL_StatusTypeDef setChannel(uint8_t channel);
    HAL_StatusTypeDef setPower(HC12_Power_t power);
    HAL_StatusTypeDef setUartFormat(const HC12_UartFormat_t &format);
    
    // 参数查询函数
    HAL_StatusTypeDef getVersion(char *version);
    HAL_StatusTypeDef getAllParams(HC12_Config_t *config);
    const HC12_Config_t& getCurrentConfig() const;
    
    // 系统控制函数
    HAL_StatusTypeDef restoreDefault();
    HAL_StatusTypeDef sleep();
    
    // 数据传输函数
    HAL_StatusTypeDef transmitData(const uint8_t *data, uint16_t size);
    HAL_StatusTypeDef receiveData(uint8_t *data, uint16_t *size, uint32_t timeout);
    
    // 中断处理
    void irqHandler();
    
    // 状态查询
    bool isInATMode() const;
    bool isInitialized() const;
    
    // 静态常量定义
    static const uint16_t AT_TIMEOUT = 1000;
    static const uint16_t SET_DELAY = 40;
    static const uint16_t EXIT_DELAY = 80;
    static const uint16_t RESET_DELAY = 200;
};

#endif /* __HC12_H */

#include "hc12.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"

/**
 * @brief 构造函数
 * @param huart: UART句柄
 * @param set_port: SET引脚端口
 * @param set_pin: SET引脚号
 */
HC12::HC12(UART_HandleTypeDef *huart, GPIO_TypeDef *set_port, uint16_t set_pin)
    : huart(huart)
    , set_port(set_port)
    , set_pin(set_pin)
    , is_in_at_mode(false)
    , rx_index(0)
{
    // 设置默认配置
    config.mode = HC12_MODE_FU3;
    config.baud_rate = HC12_BAUD_9600;
    config.channel = 1;
    config.power = HC12_POWER_8;
    config.format.data_bits = 8;
    config.format.parity = 'N';
    config.format.stop_bits = 1;
    
    // 清空接收缓冲区
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * @brief 析构函数
 */
HC12::~HC12()
{
    // 嵌入式系统通常不需要特殊的析构处理
}

/**
 * @brief HC-12模块初始化
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::init()
{
    if (!huart || !set_port) {
        return HAL_ERROR;
    }
    
    // SET引脚置高电平，退出AT模式
    HAL_GPIO_WritePin(set_port, set_pin, GPIO_PIN_SET);
    //osDelay(RESET_DELAY);
    
    return HAL_OK;
}

/**
 * @brief 进入AT指令模式
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::enterATMode()
{
    if (!huart) {
        return HAL_ERROR;
    }
    
    // SET引脚置低电平
    HAL_GPIO_WritePin(set_port, set_pin, GPIO_PIN_RESET);
    osDelay(SET_DELAY);
    
    is_in_at_mode = true;
    return HAL_OK;
}

/**
 * @brief 退出AT指令模式
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::exitATMode()
{
    if (!huart) {
        return HAL_ERROR;
    }
    
    // SET引脚置高电平
    HAL_GPIO_WritePin(set_port, set_pin, GPIO_PIN_SET);
    osDelay(EXIT_DELAY);
    
    is_in_at_mode = false;
    return HAL_OK;
}

/**
 * @brief 发送AT指令并接收响应(私有函数)
 * @param cmd: AT指令字符串
 * @param response: 响应缓冲区
 * @param timeout: 超时时间(ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::sendATCommand(const char *cmd, char *response, uint16_t timeout)
{
    if (!cmd || !is_in_at_mode) {
        return HAL_ERROR;
    }
    
    // 发送AT指令
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t*)cmd, 
                                                strlen(cmd), AT_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }
    
    // 接收响应
    if (response) {
        uint8_t rx_byte;
        uint16_t rx_count = 0;
        uint32_t start_tick = HAL_GetTick();
        
        while ((HAL_GetTick() - start_tick) < timeout) {
            if (HAL_UART_Receive(huart, &rx_byte, 1, 1) == HAL_OK) {
                response[rx_count++] = rx_byte;
                if (rx_byte == '\n' || rx_count >= 63) {
                    response[rx_count] = '\0';
                    break;
                }
            }
        }
        
        if (rx_count == 0) {
            return HAL_TIMEOUT;
        }
    }
    
    return HAL_OK;
}

/**
 * @brief 设置工作模式
 * @param mode: 工作模式
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::setMode(HC12_Mode_t mode)
{
    char cmd[16];
    char response[64];
    
    // 进入AT模式
    enterATMode();
    
    // 发送设置模式指令
    snprintf(cmd, sizeof(cmd), "AT+FU%d", mode);
    HAL_StatusTypeDef status = sendATCommand(cmd, response, AT_TIMEOUT);
    
    if (status == HAL_OK) {
        config.mode = mode;
    }
    
    // 退出AT模式
    exitATMode();
    
    return status;
}

/**
 * @brief 设置波特率
 * @param baud: 波特率
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::setBaudRate(HC12_Baud_t baud)
{
    char cmd[16];
    char response[64];
    
    // 进入AT模式
    enterATMode();
    
    // 发送设置波特率指令
    snprintf(cmd, sizeof(cmd), "AT+B%d", baud);
    HAL_StatusTypeDef status = sendATCommand(cmd, response, AT_TIMEOUT);
    
    if (status == HAL_OK) {
        config.baud_rate = baud;
    }
    
    // 退出AT模式
    exitATMode();
    
    return status;
}

/**
 * @brief 设置通信频道
 * @param channel: 频道号(1-127)
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::setChannel(uint8_t channel)
{
    if (channel < 1 || channel > 127) {
        return HAL_ERROR;
    }
    
    char cmd[16];
    char response[64];
    
    // 进入AT模式
    enterATMode();
    
    // 发送设置频道指令
    snprintf(cmd, sizeof(cmd), "AT+C%03d", channel);
    HAL_StatusTypeDef status = sendATCommand(cmd, response, AT_TIMEOUT);
    
    if (status == HAL_OK) {
        config.channel = channel;
    }
    
    // 退出AT模式
    exitATMode();
    
    return status;
}

/**
 * @brief 设置发射功率
 * @param power: 功率等级
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::setPower(HC12_Power_t power)
{
    char cmd[16];
    char response[64];
    
    // 进入AT模式
    enterATMode();
    
    // 发送设置功率指令
    snprintf(cmd, sizeof(cmd), "AT+P%d", power);
    HAL_StatusTypeDef status = sendATCommand(cmd, response, AT_TIMEOUT);
    
    if (status == HAL_OK) {
        config.power = power;
    }
    
    // 退出AT模式
    exitATMode();
    
    return status;
}

/**
 * @brief 设置串口格式
 * @param format: 串口格式
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::setUartFormat(const HC12_UartFormat_t &format)
{
    char cmd[16];
    char response[64];
    
    // 进入AT模式
    enterATMode();
    
    // 发送设置串口格式指令
    snprintf(cmd, sizeof(cmd), "AT+U%d%c%d", format.data_bits, format.parity, format.stop_bits);
    HAL_StatusTypeDef status = sendATCommand(cmd, response, AT_TIMEOUT);
    
    if (status == HAL_OK) {
        config.format = format;
    }
    
    // 退出AT模式
    exitATMode();
    
    return status;
}

/**
 * @brief 获取固件版本
 * @param version: 版本字符串缓冲区
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::getVersion(char *version)
{
    if (!version) {
        return HAL_ERROR;
    }
    
    // 进入AT模式
    enterATMode();
    
    HAL_StatusTypeDef status = sendATCommand("AT+V", version, AT_TIMEOUT);
    
    // 退出AT模式
    exitATMode();
    
    return status;
}

/**
 * @brief 获取所有参数
 * @param config: 配置结构体指针
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::getAllParams(HC12_Config_t *config)
{
    if (!config) {
        return HAL_ERROR;
    }
    
    char response[256];
    
    // 进入AT模式
    enterATMode();
    
    HAL_StatusTypeDef status = sendATCommand("AT+RX", response, AT_TIMEOUT);
    
    if (status == HAL_OK) {
        *config = this->config;
    }
    
    // 退出AT模式
    exitATMode();
    
    return status;
}

/**
 * @brief 获取当前配置
 * @retval 当前配置的引用
 */
const HC12_Config_t& HC12::getCurrentConfig() const
{
    return config;
}

/**
 * @brief 恢复出厂默认设置
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::restoreDefault()
{
    char response[64];
    
    // 进入AT模式
    enterATMode();
    
    HAL_StatusTypeDef status = sendATCommand("AT+DEFAULT", response, AT_TIMEOUT);
    
    if (status == HAL_OK) {
        // 恢复默认配置
        config.mode = HC12_MODE_FU3;
        config.baud_rate = HC12_BAUD_9600;
        config.channel = 1;
        config.power = HC12_POWER_8;
    }
    
    // 退出AT模式
    exitATMode();
    
    return status;
}

/**
 * @brief 进入睡眠模式
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::sleep()
{
    char response[64];
    
    // 进入AT模式
    enterATMode();
    
    HAL_StatusTypeDef status = sendATCommand("AT+SLEEP", response, AT_TIMEOUT);
    
    // 退出AT模式
    exitATMode();
    
    return status;
}

/**
 * @brief 发送数据(透传模式)
 * @param data: 发送数据
 * @param size: 数据长度
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::transmitData(const uint8_t *data, uint16_t size)
{
    if (!data || size == 0 || is_in_at_mode) {
        return HAL_ERROR;
    }
    
    return HAL_UART_Transmit(huart, (uint8_t*)data, size, 100);
}

/**
 * @brief 接收数据(透传模式)
 * @param data: 接收缓冲区
 * @param size: 接收到的数据长度
 * @param timeout: 超时时间
 * @retval HAL状态
 */
HAL_StatusTypeDef HC12::receiveData(uint8_t *data, uint16_t *size, uint32_t timeout)
{
    if (!data || !size || is_in_at_mode) {
        return HAL_ERROR;
    }
    
    *size = 0;
    uint32_t start_tick = HAL_GetTick();
    
    while ((HAL_GetTick() - start_tick) < timeout) {
        uint8_t rx_byte;
        if (HAL_UART_Receive(huart, &rx_byte, 1, 1) == HAL_OK) {
            data[(*size)++] = rx_byte;
            if (*size >= 255) break;  // 防止缓冲区溢出
        }
    }
    
    return (*size > 0) ? HAL_OK : HAL_TIMEOUT;
}

/**
 * @brief UART中断处理函数
 * @retval 无
 */
void HC12::irqHandler()
{
    // 在这里可以处理UART接收中断
    // 将接收到的数据存储到环形缓冲区中
    uint8_t rx_data;
    if (HAL_UART_Receive(huart, &rx_data, 1, 0) == HAL_OK) {
        if (rx_index < sizeof(rx_buffer)) {
            rx_buffer[rx_index++] = rx_data;
        }
    }
}

/**
 * @brief 检查是否处于AT模式
 * @retval true: 处于AT模式, false: 处于透传模式
 */
bool HC12::isInATMode() const
{
    return is_in_at_mode;
}

/**
 * @brief 检查模块是否已初始化
 * @retval true: 已初始化, false: 未初始化
 */
bool HC12::isInitialized() const
{
    return (huart != nullptr && set_port != nullptr);
}

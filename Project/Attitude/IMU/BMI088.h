/**
 * @file BMI088.h
 * @brief BMI088 IMU传感器驱动类
 * @details 实现了IMU接口，封装了BMI088传感器的访问
 */

#ifndef BMI088_H
#define BMI088_H

#include "Attitude.h"
#include "BMI088_def.h"
#include "main.h"
#include <stdint.h>

// 使用SPI进行通信
#define BMI088_USE_SPI

/**
 * @brief BMI088 配置结构体
 */
typedef struct 
{
    SPI_HandleTypeDef *hspi; // SPI句柄
    struct 
    {
        GPIO_TypeDef *port; // 片选信号端口
        uint16_t pin;       // 片选信号引脚
    } ce_acc, ce_gyro; // 片选信号结构体
    uint8_t gyroRange;    // 陀螺仪量程设置
    uint8_t accelRange;   // 加速度计量程设置
} BMI088Config_t;


/**
 * @brief BMI088 IMU传感器类
 * @details 实现IMU抽象接口，提供对BMI088传感器的访问
 */
class BMI088 : public IMU
{
public:
    /**
     * @brief 构造函数
     * @param config BMI088配置结构体引用
     */
    BMI088(const BMI088Config_t& config);

    /**
     * @brief 析构函数
     */
    virtual ~BMI088() = default;

    /**
     * @brief 初始化BMI088传感器
     * @return 返回初始化是否成功，true表示成功
     */
    virtual bool init() override;

    /**
     * @brief 读取IMU传感器数据
     * @param gyro 陀螺仪数据（角速度），单位：rad/s
     * @param accel 加速度计数据，单位：m/s^2
     */
    virtual void read(float gyro[3], float accel[3]) override;

    /**
     * @brief 校准陀螺仪零偏
     * @param sampleCount 采样次数，默认为500
     */
    void calibrateGyro(uint32_t sampleCount = 500);

private:
    BMI088Config_t _config; // BMI088 配置

    // 传感器灵敏度
    float _gyroSensitivity;
    float _accelSensitivity;

    // 校准值
    float _gyroOffset[3];
    bool _isCalibrated;

    // BMI088内部寄存器初始化配置
    static const uint8_t ACCEL_CONFIG_NUM = 6;
    static const uint8_t GYRO_CONFIG_NUM = 6;
    uint8_t _accelConfig[ACCEL_CONFIG_NUM][3];
    uint8_t _gyroConfig[GYRO_CONFIG_NUM][3];

    // 初始化加速度计
    bool initAccel();

    // 初始化陀螺仪
    bool initGyro();

    // 底层通信函数
    void delay_us(uint16_t us);
    void delay_ms(uint16_t ms);

    // SPI通信相关函数
#ifdef BMI088_USE_SPI
    void accelChipSelect(bool select);
    void gyroChipSelect(bool select);
    void writeSingleReg(uint8_t reg, uint8_t data);
    void readSingleReg(uint8_t reg, uint8_t *data);
    void readMultiRegs(uint8_t reg, uint8_t *data, uint8_t len);
    uint8_t spiTransfer(uint8_t data);

    // 加速度计和陀螺仪SPI操作宏
    void accelWriteSingleReg(uint8_t reg, uint8_t data);
    void accelReadSingleReg(uint8_t reg, uint8_t &data);
    void accelReadMultiRegs(uint8_t reg, uint8_t *data, uint8_t len);
    void gyroWriteSingleReg(uint8_t reg, uint8_t data);
    void gyroReadSingleReg(uint8_t reg, uint8_t &data);
    void gyroReadMultiRegs(uint8_t reg, uint8_t *data, uint8_t len);
#endif

    // 设置传感器灵敏度
    void setSensitivity();
};

#endif // BMI088_H
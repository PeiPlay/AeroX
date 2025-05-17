/**
 * @file BMI088.cpp
 * @brief BMI088 IMU传感器驱动类实现
 */

#include "BMI088.h"
#include "main.h"
#include "time_utils.h"

/**
 * @brief 构造函数
 */
BMI088::BMI088(const BMI088Config_t& config)
    : _config(config), // 使用初始化列表拷贝配置
      _isCalibrated(false)
{
    // 初始化零偏值为0
    _gyroOffset[0] = 0.0f;
    _gyroOffset[1] = 0.0f;
    _gyroOffset[2] = 0.0f;

    // 设置加速度计配置参数
    _accelConfig[0][0] = BMI088_ACC_PWR_CTRL;
    _accelConfig[0][1] = BMI088_ACC_ENABLE_ACC_ON;
    _accelConfig[0][2] = BMI088_ACC_PWR_CTRL_ERROR;

    _accelConfig[1][0] = BMI088_ACC_PWR_CONF;
    _accelConfig[1][1] = BMI088_ACC_PWR_ACTIVE_MODE;
    _accelConfig[1][2] = BMI088_ACC_PWR_CONF_ERROR;

    _accelConfig[2][0] = BMI088_ACC_CONF;
    _accelConfig[2][1] = BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set;
    _accelConfig[2][2] = BMI088_ACC_CONF_ERROR;

    _accelConfig[3][0] = BMI088_ACC_RANGE;
    _accelConfig[3][1] = _config.accelRange; // 使用 _config 访问
    _accelConfig[3][2] = BMI088_ACC_RANGE_ERROR;

    _accelConfig[4][0] = BMI088_INT1_IO_CTRL;
    _accelConfig[4][1] = BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW;
    _accelConfig[4][2] = BMI088_INT1_IO_CTRL_ERROR;

    _accelConfig[5][0] = BMI088_INT_MAP_DATA;
    _accelConfig[5][1] = BMI088_ACC_INT1_DRDY_INTERRUPT;
    _accelConfig[5][2] = BMI088_INT_MAP_DATA_ERROR;

    // 设置陀螺仪配置参数
    _gyroConfig[0][0] = BMI088_GYRO_RANGE;
    _gyroConfig[0][1] = _config.gyroRange; // 使用 _config 访问
    _gyroConfig[0][2] = BMI088_GYRO_RANGE_ERROR;

    _gyroConfig[1][0] = BMI088_GYRO_BANDWIDTH;
    _gyroConfig[1][1] = BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set;
    _gyroConfig[1][2] = BMI088_GYRO_BANDWIDTH_ERROR;

    _gyroConfig[2][0] = BMI088_GYRO_LPM1;
    _gyroConfig[2][1] = BMI088_GYRO_NORMAL_MODE;
    _gyroConfig[2][2] = BMI088_GYRO_LPM1_ERROR;

    _gyroConfig[3][0] = BMI088_GYRO_CTRL;
    _gyroConfig[3][1] = BMI088_DRDY_ON;
    _gyroConfig[3][2] = BMI088_GYRO_CTRL_ERROR;

    _gyroConfig[4][0] = BMI088_GYRO_INT3_INT4_IO_CONF;
    _gyroConfig[4][1] = BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW;
    _gyroConfig[4][2] = BMI088_GYRO_INT3_INT4_IO_CONF_ERROR;

    _gyroConfig[5][0] = BMI088_GYRO_INT3_INT4_IO_MAP;
    _gyroConfig[5][1] = BMI088_GYRO_DRDY_IO_INT3;
    _gyroConfig[5][2] = BMI088_GYRO_INT3_INT4_IO_MAP_ERROR;

    // 设置传感器灵敏度
    setSensitivity();
}

/**
 * @brief 设置传感器灵敏度
 */
void BMI088::setSensitivity()
{
    // 根据量程设置加速度计灵敏度
    switch (_config.accelRange) // 使用 _config 访问
    {
    case BMI088_ACC_RANGE_3G:
        _accelSensitivity = BMI088_ACCEL_3G_SEN;
        break;
    case BMI088_ACC_RANGE_6G:
        _accelSensitivity = BMI088_ACCEL_6G_SEN;
        break;
    case BMI088_ACC_RANGE_12G:
        _accelSensitivity = BMI088_ACCEL_12G_SEN;
        break;
    case BMI088_ACC_RANGE_24G:
        _accelSensitivity = BMI088_ACCEL_24G_SEN;
        break;
    default:
        _accelSensitivity = BMI088_ACCEL_3G_SEN;
        break;
    }

    // 根据量程设置陀螺仪灵敏度
    switch (_config.gyroRange) // 使用 _config 访问
    {
    case BMI088_GYRO_2000:
        _gyroSensitivity = BMI088_GYRO_2000_SEN;
        break;
    case BMI088_GYRO_1000:
        _gyroSensitivity = BMI088_GYRO_1000_SEN;
        break;
    case BMI088_GYRO_500:
        _gyroSensitivity = BMI088_GYRO_500_SEN;
        break;
    case BMI088_GYRO_250:
        _gyroSensitivity = BMI088_GYRO_250_SEN;
        break;
    case BMI088_GYRO_125:
        _gyroSensitivity = BMI088_GYRO_125_SEN;
        break;
    default:
        _gyroSensitivity = BMI088_GYRO_2000_SEN;
        break;
    }
}

/**
 * @brief 初始化BMI088传感器
 */
bool BMI088::init()
{
    // 初始化GPIO相关引脚
    accelChipSelect(false); // 先使CS引脚为高，取消选中
    gyroChipSelect(false);

    // 分别初始化加速度计和陀螺仪
    bool accelInitResult = initAccel();
    bool gyroInitResult = initGyro();

    if(accelInitResult && gyroInitResult)
    {
        calibrateGyro();
        return true;
    }

    return false;
}

/**
 * @brief 初始化加速度计
 */
bool BMI088::initAccel()
{
    uint8_t res = 0;
    uint8_t regData = 0;

    // 检查通信
    accelReadSingleReg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accelReadSingleReg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 软复位加速度计
    accelWriteSingleReg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    // 复位后检查通信是否正常
    accelReadSingleReg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accelReadSingleReg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 检查ID是否正确
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return false;
    }

    // 配置加速度计寄存器
    for (uint8_t i = 0; i < ACCEL_CONFIG_NUM; i++)
    {
        accelWriteSingleReg(_accelConfig[i][0], _accelConfig[i][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        accelReadSingleReg(_accelConfig[i][0], regData);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (regData != _accelConfig[i][1])
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 初始化陀螺仪
 */
bool BMI088::initGyro()
{
    uint8_t res = 0;
    uint8_t regData = 0;

    // 检查通信
    gyroReadSingleReg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    gyroReadSingleReg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 软复位陀螺仪
    gyroWriteSingleReg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    // 复位后检查通信是否正常
    gyroReadSingleReg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    gyroReadSingleReg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 检查ID是否正确
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return false;
    }

    // 配置陀螺仪寄存器
    for (uint8_t i = 0; i < GYRO_CONFIG_NUM; i++)
    {
        gyroWriteSingleReg(_gyroConfig[i][0], _gyroConfig[i][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        gyroReadSingleReg(_gyroConfig[i][0], regData);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (regData != _gyroConfig[i][1])
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief 读取IMU传感器数据
 */
void BMI088::read(float gyro[3], float accel[3])
{
    uint8_t buf[8] = {0};
    int16_t rawData;

    // 读取加速度计数据
    accelReadMultiRegs(BMI088_ACCEL_XOUT_L, buf, 6);

    // 转换加速度计数据
    rawData = (int16_t)((buf[1] << 8) | buf[0]);
    accel[0] = rawData * _accelSensitivity;

    rawData = (int16_t)((buf[3] << 8) | buf[2]);
    accel[1] = rawData * _accelSensitivity;

    rawData = (int16_t)((buf[5] << 8) | buf[4]);
    accel[2] = rawData * _accelSensitivity;

    float gyroData[3] = {0};

    // 读取陀螺仪数据
    gyroReadMultiRegs(BMI088_GYRO_X_L, buf, 6);

    // 转换陀螺仪数据
    rawData = (int16_t)((buf[1] << 8) | buf[0]);
    gyroData[0] = rawData * _gyroSensitivity;

    rawData = (int16_t)((buf[3] << 8) | buf[2]);
    gyroData[1] = rawData * _gyroSensitivity;

    rawData = (int16_t)((buf[5] << 8) | buf[4]);
    gyroData[2] = rawData * _gyroSensitivity;

    // 应用陀螺仪零偏校准值（如果已校准）
    if (_isCalibrated)
    {
        gyro[0] = gyroData[0] - _gyroOffset[0];
        gyro[1] = gyroData[1] - _gyroOffset[1];
        gyro[2] = gyroData[2] - _gyroOffset[2];
    }
    else
    {
        gyro[0] = gyroData[0];
        gyro[1] = gyroData[1];
        gyro[2] = gyroData[2];
    }
}

/**
 * @brief 校准陀螺仪零偏
 */
void BMI088::calibrateGyro(uint32_t sampleCount)
{
    float gyroBuf[3] = {0};
    float accelBuf[3] = {0};

    // 重置零偏值
    _gyroOffset[0] = 0.0f;
    _gyroOffset[1] = 0.0f;
    _gyroOffset[2] = 0.0f;

    for(uint32_t i = 0; i < sampleCount / 2; i++)
    {
        read(gyroBuf, accelBuf); // 忽略前几次数据
        delay_ms(2); // 每次采样间隔2ms
    }

    // 采集多次样本并求平均值
    for (uint32_t i = 0; i < sampleCount; i++)
    {
        read(gyroBuf, accelBuf); // 不考虑零偏的原始读数

        _gyroOffset[0] += gyroBuf[0];
        _gyroOffset[1] += gyroBuf[1];
        _gyroOffset[2] += gyroBuf[2];

        delay_ms(2); // 每次采样间隔2ms
    }

    // 计算平均值作为零偏
    _gyroOffset[0] /= (float)sampleCount;
    _gyroOffset[1] /= (float)sampleCount;
    _gyroOffset[2] /= (float)sampleCount;

    _isCalibrated = true;
}

/**
 * @brief 毫秒级延时函数
 */
void BMI088::delay_ms(uint16_t ms)
{
    utils::time::Delay::milliseconds(ms); // 使用时间工具类进行延时
}

/**
 * @brief 微秒级延时函数
 */
void BMI088::delay_us(uint16_t us)
{
    utils::time::Delay::microseconds(us); // 使用时间工具类进行延时
}

#ifdef BMI088_USE_SPI
/**
 * @brief 设置加速度计片选信号
 */
void BMI088::accelChipSelect(bool select)
{
    if (select)
    {
        HAL_GPIO_WritePin(_config.ce_acc.port, _config.ce_acc.pin, GPIO_PIN_RESET); // 使用 _config 访问
    }
    else
    {
        HAL_GPIO_WritePin(_config.ce_acc.port, _config.ce_acc.pin, GPIO_PIN_SET); // 使用 _config 访问
    }
}

/**
 * @brief 设置陀螺仪片选信号
 */
void BMI088::gyroChipSelect(bool select)
{
    if (select)
    {
        HAL_GPIO_WritePin(_config.ce_gyro.port, _config.ce_gyro.pin, GPIO_PIN_RESET); // 使用 _config 访问
    }
    else
    {
        HAL_GPIO_WritePin(_config.ce_gyro.port, _config.ce_gyro.pin, GPIO_PIN_SET); // 使用 _config 访问
    }
}

/**
 * @brief SPI单字节传输
 */
uint8_t BMI088::spiTransfer(uint8_t data)
{
    uint8_t rxData;
    HAL_SPI_TransmitReceive(_config.hspi, &data, &rxData, 1, 1000); // 使用 _config 访问
    return rxData;
}

/**
 * @brief 向单个寄存器写入数据
 */
void BMI088::writeSingleReg(uint8_t reg, uint8_t data)
{
    spiTransfer(reg);
    spiTransfer(data);
}

/**
 * @brief 从单个寄存器读取数据
 */
void BMI088::readSingleReg(uint8_t reg, uint8_t *data)
{
    spiTransfer(reg | 0x80);   // 读取操作需要将最高位置1
    *data = spiTransfer(0x55); // 发送随意数据以读取返回值
}

/**
 * @brief 从多个连续寄存器读取数据
 */
void BMI088::readMultiRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    spiTransfer(reg | 0x80);

    while (len--)
    {
        *data = spiTransfer(0x55);
        data++;
    }
}

/**
 * @brief 向加速度计寄存器写入数据
 */
void BMI088::accelWriteSingleReg(uint8_t reg, uint8_t data)
{
    accelChipSelect(true);
    writeSingleReg(reg, data);
    accelChipSelect(false);
}

/**
 * @brief 从加速度计寄存器读取数据
 */
void BMI088::accelReadSingleReg(uint8_t reg, uint8_t &data)
{
    accelChipSelect(true);
    spiTransfer(reg | 0x80);
    spiTransfer(0x55); // 加速度计需要一个额外的读取周期
    data = spiTransfer(0x55);
    accelChipSelect(false);
}

/**
 * @brief 从加速度计多个连续寄存器读取数据
 */
void BMI088::accelReadMultiRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    accelChipSelect(true);
    spiTransfer(reg | 0x80);
    spiTransfer(0x55); // 加速度计需要一个额外的读取周期

    while (len--)
    {
        *data = spiTransfer(0x55);
        data++;
    }

    accelChipSelect(false);
}

/**
 * @brief 向陀螺仪寄存器写入数据
 */
void BMI088::gyroWriteSingleReg(uint8_t reg, uint8_t data)
{
    gyroChipSelect(true);
    writeSingleReg(reg, data);
    gyroChipSelect(false);
}

/**
 * @brief 从陀螺仪寄存器读取数据
 */
void BMI088::gyroReadSingleReg(uint8_t reg, uint8_t &data)
{
    gyroChipSelect(true);
    readSingleReg(reg, &data);
    gyroChipSelect(false);
}

/**
 * @brief 从陀螺仪多个连续寄存器读取数据
 */
void BMI088::gyroReadMultiRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    gyroChipSelect(true);
    readMultiRegs(reg, data, len);
    gyroChipSelect(false);
}
#endif // BMI088_USE_SPI
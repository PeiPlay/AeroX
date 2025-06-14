/**
 * @file BMI088_def.h
 * @brief BMI088寄存器定义
 * @details 定义了BMI088传感器的寄存器地址和配置参数
 */

#ifndef BMI088_DEF_H
#define BMI088_DEF_H

// BMI088加速度计寄存器定义
#define BMI088_ACC_CHIP_ID 0x00       // 加速度计身份标识寄存器
#define BMI088_ACC_CHIP_ID_VALUE 0x1E // 加速度计身份标识值

#define BMI088_ACC_ERR_REG 0x02 // 加速度计错误寄存器
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
#define BMI088_FATAL_ERROR_SHFITS 0x0
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

#define BMI088_ACC_STATUS 0x03 // 加速度计状态寄存器
#define BMI088_ACCEL_DRDY_SHFITS 0x7
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

#define BMI088_ACCEL_XOUT_L 0x12 // 加速度计X轴低字节
#define BMI088_ACCEL_XOUT_M 0x13 // 加速度计X轴高字节
#define BMI088_ACCEL_YOUT_L 0x14 // 加速度计Y轴低字节
#define BMI088_ACCEL_YOUT_M 0x15 // 加速度计Y轴高字节
#define BMI088_ACCEL_ZOUT_L 0x16 // 加速度计Z轴低字节
#define BMI088_ACCEL_ZOUT_M 0x17 // 加速度计Z轴高字节

#define BMI088_SENSORTIME_DATA_L 0x18 // 传感器时间寄存器低字节
#define BMI088_SENSORTIME_DATA_M 0x19 // 传感器时间寄存器中字节
#define BMI088_SENSORTIME_DATA_H 0x1A // 传感器时间寄存器高字节

#define BMI088_ACC_INT_STAT_1 0x1D // 加速度计中断状态寄存器1
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

#define BMI088_TEMP_M 0x22 // 温度寄存器中字节
#define BMI088_TEMP_L 0x23 // 温度寄存器低字节

#define BMI088_ACC_CONF 0x40          // 加速度计配置寄存器
#define BMI088_ACC_CONF_MUST_Set 0x80 // 加速度计配置必须设置位
#define BMI088_ACC_BWP_SHFITS 0x4
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)

#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)

#define BMI088_ACC_RANGE 0x41 // 加速度计量程寄存器
#define BMI088_ACC_RANGE_SHFITS 0x0
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)

#define BMI088_INT1_IO_CTRL 0x53 // 中断1 IO控制寄存器
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)

#define BMI088_INT_MAP_DATA 0x58 // 中断映射数据寄存器
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)

#define BMI088_ACC_SELF_TEST 0x6D // 加速度计自检寄存器
#define BMI088_ACC_SELF_TEST_OFF 0x00
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09

#define BMI088_ACC_PWR_CONF 0x7C // 加速度计电源配置寄存器
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00

#define BMI088_ACC_PWR_CTRL 0x7D // 加速度计电源控制寄存器
#define BMI088_ACC_ENABLE_ACC_OFF 0x00
#define BMI088_ACC_ENABLE_ACC_ON 0x04

#define BMI088_ACC_SOFTRESET 0x7E // 加速度计软复位寄存器
#define BMI088_ACC_SOFTRESET_VALUE 0xB6

// BMI088陀螺仪寄存器定义
#define BMI088_GYRO_CHIP_ID 0x00       // 陀螺仪身份标识寄存器
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F // 陀螺仪身份标识值

#define BMI088_GYRO_X_L 0x02 // 陀螺仪X轴低字节
#define BMI088_GYRO_X_H 0x03 // 陀螺仪X轴高字节
#define BMI088_GYRO_Y_L 0x04 // 陀螺仪Y轴低字节
#define BMI088_GYRO_Y_H 0x05 // 陀螺仪Y轴高字节
#define BMI088_GYRO_Z_L 0x06 // 陀螺仪Z轴低字节
#define BMI088_GYRO_Z_H 0x07 // 陀螺仪Z轴高字节

#define BMI088_GYRO_INT_STAT_1 0x0A // 陀螺仪中断状态寄存器1
#define BMI088_GYRO_DYDR_SHFITS 0x7
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

#define BMI088_GYRO_RANGE 0x0F // 陀螺仪量程寄存器
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)

#define BMI088_GYRO_BANDWIDTH 0x10 // 陀螺仪带宽寄存器
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80
#define BMI088_GYRO_2000_532_HZ 0x00
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02
#define BMI088_GYRO_400_47_HZ 0x03
#define BMI088_GYRO_200_23_HZ 0x04
#define BMI088_GYRO_100_12_HZ 0x05
#define BMI088_GYRO_200_64_HZ 0x06
#define BMI088_GYRO_100_32_HZ 0x07

#define BMI088_GYRO_LPM1 0x11 // 陀螺仪低功耗模式1寄存器
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SUSPEND_MODE 0x80
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20

#define BMI088_GYRO_SOFTRESET 0x14 // 陀螺仪软复位寄存器
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

#define BMI088_GYRO_CTRL 0x15 // 陀螺仪控制寄存器
#define BMI088_DRDY_OFF 0x00
#define BMI088_DRDY_ON 0x80

#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16 // 陀螺仪中断3和中断4 IO配置寄存器
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)

#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18 // 陀螺仪中断3和中断4映射寄存器
#define BMI088_GYRO_DRDY_IO_OFF 0x00
#define BMI088_GYRO_DRDY_IO_INT3 0x01
#define BMI088_GYRO_DRDY_IO_INT4 0x80
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4)

// 公共定义
#define BMI088_TEMP_FACTOR 0.125f // 温度系数
#define BMI088_TEMP_OFFSET 23.0f  // 温度偏移

// 加速度计灵敏度
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

// 陀螺仪灵敏度
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

// 通信延时定义
#define BMI088_COM_WAIT_SENSOR_TIME 150
#define BMI088_LONG_DELAY_TIME 80

// 错误代码定义
enum BMI088_ERROR_CODE
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,
    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF
};

#endif // BMI088_DEF_H
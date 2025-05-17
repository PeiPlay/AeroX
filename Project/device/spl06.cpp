#include "spl06.h"
#include "cmsis_os.h"
#include <cstring> // 用于 memset
#include <cmath>   // 用于 fabsf, powf, abs

// SPI 读写标志
constexpr uint8_t SPL_WRITE = 0x00;
constexpr uint8_t SPL_READ = 0x80;

// 气压测量速率 (samples/sec)
constexpr uint8_t PM_RATE_1 = (0 << 4);
constexpr uint8_t PM_RATE_2 = (1 << 4);
constexpr uint8_t PM_RATE_4 = (2 << 4);
constexpr uint8_t PM_RATE_8 = (3 << 4);
constexpr uint8_t PM_RATE_16 = (4 << 4);
constexpr uint8_t PM_RATE_32 = (5 << 4);
constexpr uint8_t PM_RATE_64 = (6 << 4);
constexpr uint8_t PM_RATE_128 = (7 << 4);

// 气压过采样率 (times)
constexpr uint8_t PM_PRC_1 = 0;   // Single, kP=524288, 3.6ms
constexpr uint8_t PM_PRC_2 = 1;   // 2 times, kP=1572864, 5.2ms
constexpr uint8_t PM_PRC_4 = 2;   // 4 times, kP=3670016, 8.4ms
constexpr uint8_t PM_PRC_8 = 3;   // 8 times, kP=7864320, 14.8ms
constexpr uint8_t PM_PRC_16 = 4;  // 16 times, kP=253952, 27.6ms
constexpr uint8_t PM_PRC_32 = 5;  // 32 times, kP=516096, 53.2ms
constexpr uint8_t PM_PRC_64 = 6;  // 64 times, kP=1040384, 104.4ms
constexpr uint8_t PM_PRC_128 = 7; // 128 times, kP=2088960, 206.8ms

// 温度测量速率 (samples/sec)
constexpr uint8_t TMP_RATE_1 = (0 << 4);
constexpr uint8_t TMP_RATE_2 = (1 << 4);
constexpr uint8_t TMP_RATE_4 = (2 << 4);
constexpr uint8_t TMP_RATE_8 = (3 << 4);
constexpr uint8_t TMP_RATE_16 = (4 << 4);
constexpr uint8_t TMP_RATE_32 = (5 << 4);
constexpr uint8_t TMP_RATE_64 = (6 << 4);
constexpr uint8_t TMP_RATE_128 = (7 << 4);

// 温度过采样率 (times)
constexpr uint8_t TMP_PRC_1 = 0;   // Single, kT=524288
constexpr uint8_t TMP_PRC_2 = 1;   // 2 times, kT=1572864
constexpr uint8_t TMP_PRC_4 = 2;   // 4 times, kT=3670016
constexpr uint8_t TMP_PRC_8 = 3;   // 8 times, kT=7864320
constexpr uint8_t TMP_PRC_16 = 4;  // 16 times, kT=253952
constexpr uint8_t TMP_PRC_32 = 5;  // 32 times, kT=516096
constexpr uint8_t TMP_PRC_64 = 6;  // 64 times, kT=1040384
constexpr uint8_t TMP_PRC_128 = 7; // 128 times, kT=2088960

constexpr uint8_t TMP_EXTER_SEN = (1 << 7); // 使用 MEMS 温度传感器
constexpr uint8_t TMP_INTER_SEN = 0;       // 使用 ASIC 温度传感器

// SPL06_MEAS_CFG 寄存器位定义
constexpr uint8_t MEAS_COEF_RDY = 0x80;    // 校准系数准备就绪
constexpr uint8_t MEAS_SENSOR_RDY = 0x40;  // 传感器准备就绪
constexpr uint8_t MEAS_TMP_RDY = 0x20;     // 温度数据准备就绪
constexpr uint8_t MEAS_PRS_RDY = 0x10;     // 气压数据准备就绪

// 测量控制模式
constexpr uint8_t MEAS_CTRL_Standby = 0x00;             // 待机模式
constexpr uint8_t MEAS_CTRL_PressMeasure = 0x01;        // 单次气压测量
constexpr uint8_t MEAS_CTRL_TempMeasure = 0x02;         // 单次温度测量
constexpr uint8_t MEAS_CTRL_ContinuousPress = 0x05;     // 连续气压测量
constexpr uint8_t MEAS_CTRL_ContinuousTemp = 0x06;      // 连续温度测量
constexpr uint8_t MEAS_CTRL_ContinuousPressTemp = 0x07; // 连续气压和温度测量

// FIFO_STS 寄存器位定义
constexpr uint8_t SPL06_FIFO_FULL = 0x02;  // FIFO 满
constexpr uint8_t SPL06_FIFO_EMPTY = 0x01; // FIFO 空

// INT_STS 寄存器位定义
constexpr uint8_t SPL06_INT_FIFO_FULL = 0x04; // FIFO 满中断
constexpr uint8_t SPL06_INT_TMP = 0x02;       // 温度就绪中断
constexpr uint8_t SPL06_INT_PRS = 0x01;       // 气压就绪中断

// 寄存器地址
constexpr uint8_t SPL06_PSR_B2 = 0x00;    // 气压数据 MSB
constexpr uint8_t SPL06_PSR_B1 = 0x01;    // 气压数据 LSB
constexpr uint8_t SPL06_PSR_B0 = 0x02;    // 气压数据 XLSB
constexpr uint8_t SPL06_TMP_B2 = 0x03;    // 温度数据 MSB
constexpr uint8_t SPL06_TMP_B1 = 0x04;    // 温度数据 LSB
constexpr uint8_t SPL06_TMP_B0 = 0x05;    // 温度数据 XLSB
constexpr uint8_t SPL06_PSR_CFG = 0x06;   // 气压配置寄存器
constexpr uint8_t SPL06_TMP_CFG = 0x07;   // 温度配置寄存器
constexpr uint8_t SPL06_MEAS_CFG = 0x08;  // 测量配置寄存器
constexpr uint8_t SPL06_CFG_REG = 0x09;   // 配置寄存器 (中断和FIFO设置)
constexpr uint8_t SPL06_INT_STS = 0x0A;   // 中断状态寄存器
constexpr uint8_t SPL06_FIFO_STS = 0x0B;  // FIFO 状态寄存器
constexpr uint8_t SPL06_RESET = 0x0C;     // 软复位寄存器
constexpr uint8_t SPL06_DEV_ID = 0x0D;    // 器件ID寄存器
constexpr uint8_t SPL06_COEF = 0x10;      // 校准系数寄存器起始地址
constexpr uint8_t SPL06_COEF_SRCE = 0x28; // 温度传感器源寄存器 (0x07 bit7)

// CFG_REG 位定义
constexpr uint8_t SPL06_CFG_T_SHIFT = 0x08; // 温度结果位移 (当过采样率 > 8 时需要)
constexpr uint8_t SPL06_CFG_P_SHIFT = 0x04; // 气压结果位移 (当过采样率 > 8 时需要)

// 高度计算常量
constexpr float CONST_PF = 0.1902630958f; // (1/5.25588f) 压力因子
constexpr float FIX_TEMP = 20.0f;         // 用于高度计算的固定温度 (°C)

// 中值滤波常量
constexpr int32_t MAX_DELTA_ERROR = 10000; // 滤波器允许的最大差值 (Pa * 10)，用于检测突变

// --- 辅助函数 ---
/**
* @brief 计算浮点数的绝对值
* @param x 输入浮点数
* @return 绝对值
*/
inline float MyFP32Abs(float x) { return fabsf(x); }

/**
* @brief 计算32位有符号整数的绝对值
* @param x 输入整数
* @return 绝对值
*/
inline int32_t MyINT32SAbs(int32_t x) { return abs(x); }

/**
* @brief 快速计算3个整数的中值
* @param arr 包含3个整数的数组
* @return 中值
*/
int32_t QuickMedian3_INT32S(int32_t* arr) {
   // 简单的3元素中值查找
   int32_t a = arr[0], b = arr[1], c = arr[2];
   if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
   if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
   return c;
}


SPL06::SPL06(const SPL06Config_t& config) : _config(config) {
   // 构造函数: 使用初始化列表拷贝配置，并初始化其他成员
   memset(&caliPara_, 0, sizeof(caliPara_));
   memset(&info_, 0, sizeof(info_)); // Status 标志位在结构体定义中初始化
   baroGndPressure_ = 101325.0f;
   baroGndAltitude_ = 0.0f;
   baroCaliTimeout_ = 0;
   filterIndex_ = 0;
   filterIsReady_ = false;
   memset(filterBuff_, 0, sizeof(filterBuff_));
}

bool SPL06::init() {
   info_.Status.InitOK = 0; // 假设初始化失败，直到成功

   chipSelect(false); // 取消片选
   delayMs(2); // 等待传感器启动
   softReset();
   delayMs(100); // 复位后等待，传感器内部初始化

   uint8_t id = spiReadReg(SPL06_DEV_ID);

   if (id != 0x10) { // 检查器件ID是否正确
       // 器件ID不正确，初始化失败
       return false;
   }
   
   uint8_t coef[18];
   spiReadContinuous(SPL06_COEF, coef, 18); // 读取校准系数

   // 计算校准系数
   caliPara_.C0 = ((int16_t)coef[0] << 4) | ((coef[1] & 0xF0) >> 4);
   caliPara_.C0 = (caliPara_.C0 & 0x0800) ? (0xF000 | caliPara_.C0) : caliPara_.C0; // 符号扩展

   caliPara_.C1 = ((int16_t)(coef[1] & 0x0F) << 8) | coef[2];
   caliPara_.C1 = (caliPara_.C1 & 0x0800) ? (0xF000 | caliPara_.C1) : caliPara_.C1; // 符号扩展

   caliPara_.C00 = ((int32_t)coef[3] << 12) | ((int32_t)coef[4] << 4) | ((int32_t)coef[5] >> 4);
   caliPara_.C00 = (caliPara_.C00 & 0x080000) ? (0xFFF00000 | caliPara_.C00) : caliPara_.C00; // 符号扩展

   caliPara_.C10 = ((int32_t)(coef[5] & 0x0F) << 16) | ((int32_t)coef[6] << 8) | coef[7];
   caliPara_.C10 = (caliPara_.C10 & 0x080000) ? (0xFFF00000 | caliPara_.C10) : caliPara_.C10; // 符号扩展

   caliPara_.C01 = ((int16_t)coef[8] << 8) | coef[9];
   caliPara_.C11 = ((int16_t)coef[10] << 8) | coef[11];
   caliPara_.C20 = ((int16_t)coef[12] << 8) | coef[13];
   caliPara_.C21 = ((int16_t)coef[14] << 8) | coef[15];
   caliPara_.C30 = ((int16_t)coef[16] << 8) | coef[17];

   // 定义将要使用的配置
   uint8_t press_rate = PM_RATE_8;
   uint8_t press_oversampling = PM_PRC_32;
   uint8_t temp_rate = TMP_RATE_8;
   uint8_t temp_oversampling = TMP_PRC_2;

   // 1. 配置气压和温度的专用寄存器 (PSR_CFG, TMP_CFG)
   //    并确定是否需要位移
   spiWriteReg(SPL06_PSR_CFG, press_rate | press_oversampling);
   // 根据过采样率设置定标因子 kP (从原 configPressure 移入)
   switch (press_oversampling) {
       case PM_PRC_1: caliPara_.kP = 524288.0f; break;
       case PM_PRC_2: caliPara_.kP = 1572864.0f; break;
       case PM_PRC_4: caliPara_.kP = 3670016.0f; break;
       case PM_PRC_8: caliPara_.kP = 7864320.0f; break;
       case PM_PRC_16: caliPara_.kP = 253952.0f; break;
       case PM_PRC_32: caliPara_.kP = 516096.0f; break;
       case PM_PRC_64: caliPara_.kP = 1040384.0f; break;
       case PM_PRC_128: caliPara_.kP = 2088960.0f; break;
       default: caliPara_.kP = 516096.0f; break; // Default to a common value
   }

   spiWriteReg(SPL06_TMP_CFG, temp_rate | temp_oversampling | TMP_EXTER_SEN);
   // 根据过采样率设置定标因子 kT (从原 configTemperature 移入)
    switch (temp_oversampling) {
       case TMP_PRC_1: caliPara_.kT = 524288.0f; break;
       case TMP_PRC_2: caliPara_.kT = 1572864.0f; break;
       case TMP_PRC_4: caliPara_.kT = 3670016.0f; break;
       case TMP_PRC_8: caliPara_.kT = 7864320.0f; break;
       case TMP_PRC_16: caliPara_.kT = 253952.0f; break;
       case TMP_PRC_32: caliPara_.kT = 516096.0f; break;
       case TMP_PRC_64: caliPara_.kT = 1040384.0f; break;
       case TMP_PRC_128: caliPara_.kT = 2088960.0f; break;
       default: caliPara_.kT = 1572864.0f; break; // Default to a common value
   }

   // 2. 构建并写入 CFG_REG (0x09)
   uint8_t cfg_reg_val = 0;
   // 尝试像官方驱动一样设置 INT_HL (bit 7)
   // 如果不希望改变中断行为，可以注释掉下面这行，则 cfg_reg_val 初始化为0，与您之前的行为一致（除了合并写入）
   //cfg_reg_val |= (1 << 7); // INT_HL = 1 (Active High interrupt)

   if (press_oversampling > PM_PRC_8) {
       cfg_reg_val |= SPL06_CFG_P_SHIFT; // Set pressure shift bit
   }
   if (temp_oversampling > TMP_PRC_8) {
       cfg_reg_val |= SPL06_CFG_T_SHIFT; // Set temperature shift bit
   }
   spiWriteReg(SPL06_CFG_REG, cfg_reg_val);

   // 在启动测量前，检查传感器和校准系数是否就绪
   uint8_t meas_cfg_before_startup = spiReadReg(SPL06_MEAS_CFG);
   if (!((meas_cfg_before_startup & MEAS_COEF_RDY) && (meas_cfg_before_startup & MEAS_SENSOR_RDY))) {
       // 传感器或校准系数未就绪，无法启动测量
       return false;
   }

   // 启动连续测量模式
   uint8_t target_mode = MEAS_CTRL_ContinuousPressTemp;
   startup(target_mode); 
   startup(target_mode); 

    startup(target_mode); 

    startup(target_mode); 



   // **关键检查**: 验证测量模式是否已正确设置
   delayMs(1); // 短暂延时
   uint8_t meas_cfg_after_startup = spiReadReg(SPL06_MEAS_CFG);

   if ((meas_cfg_after_startup & 0x07) != target_mode) {
       // 测量模式设置失败。
       return false; 
   }

   // 等待第一次读数稳定
   delayMs(150); 

   info_.Status.InitOK = 1; // 标记初始化成功
   return true;
}

void SPL06::update() {
   if (!info_.Status.InitOK) {
       return; // 如果未初始化成功，则不执行更新
   }

   uint8_t meas_status = spiReadReg(SPL06_MEAS_CFG);
   bool new_data_read = false;

   // 检查气压数据是否就绪
   if (meas_status & MEAS_PRS_RDY) {
       info_.RawPressure = readRawPressure();
       new_data_read = true;
   }

   // 检查温度数据是否就绪
   if (meas_status & MEAS_TMP_RDY) {
       info_.RawTemperature = readRawTemperature();
       new_data_read = true;
   }

   // 如果在本次调用中没有读取到任何新数据（例如，标志位均未置位），
   // 并且原始值仍为初始的0，则后续计算无意义，可能导致输出持续为0。
   // 这种情况下，可以考虑提前返回，或至少意识到数据是陈旧的。
   // 对于“始终为0”的问题，这意味着 PRS_RDY 和 TMP_RDY 可能从未被设置。
   // if (!new_data_read && info_.RawPressure == 0 && info_.RawTemperature == 0) {
   //     // 可能是传感器未正确启动测量，或update调用过快
   //     return;
   // }


   // 缩放原始数据
   float rawTempScaled = (float)info_.RawTemperature / caliPara_.kT;
   float rawPressScaled = (float)info_.RawPressure / caliPara_.kP;

   // 计算补偿后的温度
   info_.Temperature = 0.5f * caliPara_.C0 + rawTempScaled * caliPara_.C1;

   // 计算补偿后的气压
   float qua2 = caliPara_.C10 + rawPressScaled * (caliPara_.C20 + rawPressScaled * caliPara_.C30);
   float qua3 = rawTempScaled * rawPressScaled * (caliPara_.C11 + rawPressScaled * caliPara_.C21);
   // 先保存未滤波的值
   info_.UnfilteredPressure = caliPara_.C00 + rawPressScaled * qua2 + rawTempScaled * caliPara_.C01 + qua3;

   // 应用中值滤波 (使用整数运算并保留一位小数)
   // 注意：滤波函数处理的是乘以10后的整数值
   int32_t pressureInt = static_cast<int32_t>(info_.UnfilteredPressure * 10.0f);
   pressureInt = medianFilter(pressureInt);
   // 将滤波后的结果存回 info_.Pressure
   info_.Pressure = static_cast<float>(pressureInt) / 10.0f;

   // 校准地面气压或计算相对高度 (使用滤波后的气压值)
   if (!info_.Status.IsStable) {
       calibrateGroundPressure(info_.Pressure); // 校准地面气压
       info_.Altitude = 0.0f; // 校准期间，相对高度为0
   } else {
       // 计算相对于校准后地面的高度
       info_.Altitude = pressureToAltitude(info_.Pressure) - baroGndAltitude_;
   }
}

// --- 私有辅助方法实现 ---

/**
* @brief 毫秒延时
* @param ms 延时毫秒数
*/
void SPL06::delayMs(uint32_t ms) {
   // 使用 HAL 库提供的延时函数
   // 如果配置结构体中提供了延时函数指针，则使用它
   // if (_config.delayMs) { _config.delayMs(ms); } else { HAL_Delay(ms); }
   osDelay(ms); // 使用 FreeRTOS 的延时函数
}

/**
* @brief 控制片选引脚
* @param select true: 使能片选 (通常为低电平), false: 禁用片选 (通常为高电平)
*/
void SPL06::chipSelect(bool select) {
   HAL_GPIO_WritePin(_config.csPin.port, _config.csPin.pin, select ? GPIO_PIN_RESET : GPIO_PIN_SET);

}

/**
* @brief 通过SPI向单个寄存器写入数据
* @param reg 寄存器地址
* @param data 要写入的数据
*/
void SPL06::spiWriteReg(uint8_t reg, uint8_t data) {
    uint8_t txBuff[2];
    uint8_t rxBuff[2]; // 伪接收缓冲区

    txBuff[0] = reg;
    txBuff[1] = data; // 发送数据

    chipSelect(true); // 使能片选
    // 使用配置中提供的SPI句柄进行传输
    HAL_SPI_TransmitReceive(_config.hspi, txBuff, rxBuff, 2, 1000);
    chipSelect(false); // 禁用片选
}

/**
* @brief 通过SPI从单个寄存器读取数据
* @param reg 寄存器地址
* @return 读取到的数据
*/
uint8_t SPL06::spiReadReg(uint8_t reg) {
   uint8_t txBuff[2]; // 初始化发送缓冲区
   uint8_t rxBuff[2];

    txBuff[0] = reg | SPL_READ; // 设置读标志位

   chipSelect(true); // 使能片选
   HAL_SPI_TransmitReceive(_config.hspi, txBuff, rxBuff, 2, 1000);
   chipSelect(false); // 禁用片选

   return rxBuff[1]; // 返回接收到的数据字节
}

/**
* @brief 通过SPI连续读取多个寄存器的数据
* @param reg 起始寄存器地址
* @param pRxBuff 接收数据缓冲区指针
* @param len 要读取的字节数
*/
void SPL06::spiReadContinuous(uint8_t reg, uint8_t* pRxBuff, uint16_t len) {
   uint8_t txAddr = reg | SPL_READ; // 设置读标志位

   chipSelect(true); // 使能片选
   HAL_SPI_Transmit(_config.hspi, &txAddr, 1, 100); // 发送寄存器地址
   HAL_SPI_Receive(_config.hspi, pRxBuff, len, 100); // 接收数据
   chipSelect(false); // 禁用片选
}

/**
* @brief 执行软复位
*/
void SPL06::softReset() {
   spiWriteReg(SPL06_RESET, 0x09); // 向复位寄存器写入特定值
}

/**
* @brief 启动传感器测量
* @param mode 测量模式 (MEAS_CTRL_*)
*/
void SPL06::startup(uint8_t mode) {
   spiWriteReg(SPL06_MEAS_CFG, mode); // 设置测量模式
}

/**
* @brief 读取原始气压数据
* @return 24位原始气压值 (有符号)
*/
int32_t SPL06::readRawPressure() {
   uint8_t buff[3];
   spiReadContinuous(SPL06_PSR_B2, buff, 3); // 从气压寄存器读取3字节
   // 组合成32位整数
   int32_t pressure = ((int32_t)buff[0] << 16) | ((int32_t)buff[1] << 8) | buff[2];
   // 进行符号扩展 (24位有符号数)
   pressure = (pressure & 0x800000) ? (0xFF000000 | pressure) : pressure;
   return pressure;
}

/**
* @brief 读取原始温度数据
* @return 24位原始温度值 (有符号)
*/
int32_t SPL06::readRawTemperature() {
   uint8_t buff[3];
   spiReadContinuous(SPL06_TMP_B2, buff, 3); // 从温度寄存器读取3字节
   // 组合成32位整数
   int32_t temper = ((int32_t)buff[0] << 16) | ((int32_t)buff[1] << 8) | buff[2];
   // 进行符号扩展 (24位有符号数)
   temper = (temper & 0x800000) ? (0xFF000000 | temper) : temper;
   return temper;
}

/**
* @brief 将气压值转换为海拔高度
* @param pressure 气压值 (Pa)
* @return 海拔高度 (m)，相对于标准大气压 (101325 Pa)
*/
float SPL06::pressureToAltitude(float pressure) {
   if (pressure <= 0) {
       return 0.0f; // 避免除以零或对非正数取对数
   }
   // 使用标准气压公式 (更常用)
   // Altitude = 44330.0 * (1.0 - pow(pressure / 101325.0, CONST_PF))
   // 使用原始 C 代码中的公式 (带固定温度补偿):
   return ((powf((101325.0f / pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;
}

/**
* @brief 校准地面气压参考值
* @param currentPressure 当前测量的气压值 (Pa)
*/
void SPL06::calibrateGroundPressure(float currentPressure) {
   const float pressureError = currentPressure - baroGndPressure_;
   // 缓慢调整地面气压参考值，使其趋近当前气压
   baroGndPressure_ += pressureError * 0.15f;

   // 检查是否稳定：误差小于当前地面参考压力的 0.005%
   // 并且稳定持续时间超过 250ms
   if (MyFP32Abs(pressureError) < (baroGndPressure_ * 0.00005f)) {
       // 检查系统时钟是否可用且有效 (需要获取系统时钟的函数)
       // uint32_t currentTick = HAL_GetTick(); // 或者 _config.getTick()
       uint32_t currentTick = HAL_GetTick(); // 直接使用 HAL_GetTick
       if (currentTick > baroCaliTimeout_ + 250 || currentTick < baroCaliTimeout_) { // 处理时钟回绕
            // 稳定时间足够长，认为校准完成
            baroGndAltitude_ = pressureToAltitude(baroGndPressure_); // 计算校准后的地面高度
            info_.Status.IsStable = 1; // 标记为稳定
       }
        // 如果稳定但时间未到，则继续等待
   } else {
       // 如果气压变化较大，重置稳定计时器
       // baroCaliTimeout_ = HAL_GetTick(); // 或者 _config.getTick()
       baroCaliTimeout_ = HAL_GetTick(); // 直接使用 HAL_GetTick
       info_.Status.IsStable = 0; // 标记为不稳定
   }
}

/**
* @brief 对气压值应用中值滤波器
* @param newPressure 新的气压读数 (单位: Pa * 10)
* @return 滤波后的气压值 (单位: Pa * 10)
*/
int32_t SPL06::medianFilter(int32_t newPressure) {
   // 注意: 输入和输出的气压值都乘以了 10

   uint32_t nextIndex = filterIndex_ + 1; // 计算下一个缓冲区索引
   if (nextIndex >= MEDIAN_FILTER_LEN) {
       nextIndex = 0; // 循环缓冲区
       filterIsReady_ = true; // 标记缓冲区已满，可以进行滤波
   }

   // 获取上一个有效值的索引
   int32_t lastIndex = (filterIndex_ == 0) ? (MEDIAN_FILTER_LEN - 1) : (filterIndex_ - 1);
   const int32_t lastPressure = filterBuff_[lastIndex]; // 获取上一次滤波结果

   if (filterIsReady_) { // 如果缓冲区已满
       // 检查新值与上一个值的差值，判断是否为突变干扰
       if (MyINT32SAbs(lastPressure - newPressure) < MAX_DELTA_ERROR) {
           // 差值在允许范围内，将新值存入缓冲区
           filterBuff_[filterIndex_] = newPressure;
           filterIndex_ = nextIndex; // 更新索引
           // 返回缓冲区的中值
           return QuickMedian3_INT32S(filterBuff_);
       } else {
           // 检测到突变，忽略当前新值，返回上一次的滤波结果
           return lastPressure;
       }
   } else { // 如果缓冲区未满
       // 将新值存入缓冲区
       filterBuff_[filterIndex_] = newPressure;
       filterIndex_ = nextIndex; // 更新索引
       // 在缓冲区填满之前，直接返回当前值
       return newPressure;
   }
}

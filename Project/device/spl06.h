#ifndef DEVICE_SPL06_H
#define DEVICE_SPL06_H

#include <cstdint>
#include <cmath>
#include "main.h" // 包含HAL库定义 (例如 SPI_HandleTypeDef, GPIO_TypeDef)

/**
* @brief SPL06 配置结构体
*/
typedef struct
{
   SPI_HandleTypeDef *hspi; // SPI句柄指针
   struct
   {
       GPIO_TypeDef *port; // 片选信号端口
       uint16_t pin;       // 片选信号引脚
   } csPin; // 片选引脚信息
} SPL06Config_t;


class SPL06 {
public:
   // --- 公开结构体 ---
   /**
    * @brief SPL06 校准参数结构体
    */
   struct SPLCaliPara_t {
       int16_t C0;
       int16_t C1;
       int32_t C00;
       int32_t C10;
       int16_t C01;
       int16_t C11;
       int16_t C20;
       int16_t C21;
       int16_t C30;
       float kT; // 温度定标因子
       float kP; // 气压定标因子
   };

   /**
    * @brief SPL06 传感器信息结构体
    */
   struct SPLInfo_t {
       /**
        * @brief 状态标志位结构体
        */
       struct Status_t {
           uint8_t InitOK : 1;   // 初始化成功标志
           uint8_t IsStable : 1; // 气压计是否稳定标志
           uint8_t Reserve : 6;  // 保留位
       } Status = {0, 0, 0}; // 初始化标志位

       int32_t RawPressure = 0;    // 原始气压值
       int32_t RawTemperature = 0; // 原始温度值
       float UnfilteredPressure = 0.0f; // 补偿后但未滤波的气压值 (单位: Pa)
       float Pressure = 0.0f;      // 补偿后且滤波后的气压值 (单位: Pa)
       float Temperature = 0.0f;   // 补偿后的温度值 (单位: °C)
       float Altitude = 0.0f;      // 相对高度 (单位: m)
   };

   // --- 公开方法 ---
   /**
    * @brief 构造函数
    * @param config SPL06配置结构体引用
    */
   SPL06(const SPL06Config_t& config);

   /**
    * @brief 初始化SPL06传感器
    * @return 初始化是否成功
    */
   bool init();

   /**
    * @brief 更新传感器读数和计算值
    */
   void update();

   // --- Getters ---
   /**
    * @brief 获取相对高度
    * @return 相对高度 (m)
    */
   float getAltitude() const { return info_.Altitude; }
   /**
    * @brief 获取补偿后且滤波后的气压值
    * @return 滤波后的气压值 (Pa)
    */
   float getPressure() const { return info_.Pressure; }
   /**
    * @brief 获取补偿后但未滤波的气压值
    * @return 未滤波的气压值 (Pa)
    */
   float getUnfilteredPressure() const { return info_.UnfilteredPressure; }
   /**
    * @brief 获取补偿后的温度值
    * @return 温度值 (°C)
    */
   float getTemperature() const { return info_.Temperature; }
   /**
    * @brief 获取原始气压值
    * @return 原始气压值
    */
   int32_t getRawPressure() const { return info_.RawPressure; }
   /**
    * @brief 获取原始温度值
    * @return 原始温度值
    */
   int32_t getRawTemperature() const { return info_.RawTemperature; }
   /**
    * @brief 检查传感器是否初始化成功
    * @return true 如果初始化成功
    */
   bool isOK() const { return info_.Status.InitOK; }
   /**
    * @brief 检查传感器读数是否稳定（用于地面气压校准）
    * @return true 如果读数稳定
    */
   bool isStable() const { return info_.Status.IsStable; }
   /**
    * @brief 获取校准参数
    * @return 校准参数结构体
    */
   SPLCaliPara_t getCalibrationData() const { return caliPara_; }


private:
   // --- 私有成员 ---
   SPL06Config_t _config; // 保存硬件配置
   SPLCaliPara_t caliPara_; // 校准参数
   SPLInfo_t info_;         // 传感器信息

   // 校准和高度计算状态
   float baroGndPressure_ = 101325.0f; // 地面气压参考值 (Pa)
   float baroGndAltitude_ = 0.0f;      // 地面气压对应的高度 (m)
   uint32_t baroCaliTimeout_ = 0;      // 用于校准稳定性的时间戳

   // 中值滤波状态
   static constexpr int MEDIAN_FILTER_LEN = 3; // 滤波器长度
   int32_t filterBuff_[MEDIAN_FILTER_LEN] = {0}; // 滤波缓冲区
   uint32_t filterIndex_ = 0;                   // 当前滤波索引
   bool filterIsReady_ = false;                 // 滤波缓冲区是否已满

   // --- 私有辅助方法 ---
   // 硬件抽象层 (使用配置结构体实现)
   void delayMs(uint32_t ms);
   void chipSelect(bool select); // 控制片选信号
   void spiWriteReg(uint8_t reg, uint8_t data);
   uint8_t spiReadReg(uint8_t reg);
   void spiReadContinuous(uint8_t reg, uint8_t* pRxBuff, uint16_t len);

   // 设备控制
   void softReset();
   void configTemperature(uint8_t rate, uint8_t oversampling);
   void configPressure(uint8_t rate, uint8_t oversampling);
   void startup(uint8_t mode);
   int32_t readRawPressure();
   int32_t readRawTemperature();

   // 数据处理
   float pressureToAltitude(float pressure);
   void calibrateGroundPressure(float currentPressure);
   int32_t medianFilter(int32_t newPressure); // 输入是 气压值 * 10
};

#endif // DEVICE_SPL06_H

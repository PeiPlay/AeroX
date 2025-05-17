
#ifndef _SPL06_001_H_
#define _SPL06_001_H_

#include "cpu.h"
#include <math.h>

/*portable*/
#define SPL_ENABLE()    SPL_CS_GPIO_Port->ODR &= ~SPL_CS_Pin
#define SPL_DISABLE()   SPL_CS_GPIO_Port->ODR |= SPL_CS_Pin
#define SPL_DelayMS(x)  Hal_DelayMs(x)

#define SPL_SPI_TR(tx,rx,len,timeout) \
        HAL_SPI_TransmitReceive(&hspi3,tx,rx,len,timeout)
#define SPL_SPI_TX(tx,len,timeout) \
        HAL_SPI_Transmit(&hspi3,tx,len,timeout)
#define SPL_SPI_RX(rx,len,timeout) \
        HAL_SPI_Receive(&hspi3,rx,len,timeout)
        
/*--------*/

#define SPL_WRITE   0
#define SPL_READ    0x80

//气压测量速率(sample/sec),Background 模式使用
#define  PM_RATE_1          (0<<4)      //1 measurements pr. sec.
#define  PM_RATE_2          (1<<4)      //2 measurements pr. sec.
#define  PM_RATE_4          (2<<4)      //4 measurements pr. sec.           
#define  PM_RATE_8          (3<<4)      //8 measurements pr. sec.
#define  PM_RATE_16         (4<<4)      //16 measurements pr. sec.
#define  PM_RATE_32         (5<<4)      //32 measurements pr. sec.
#define  PM_RATE_64         (6<<4)      //64 measurements pr. sec.
#define  PM_RATE_128        (7<<4)      //128 measurements pr. sec.

//气压重采样速率(times),Background 模式使用
#define PM_PRC_1            0       //Sigle         kP=524288   ,3.6ms
#define PM_PRC_2            1       //2 times       kP=1572864  ,5.2ms
#define PM_PRC_4            2       //4 times       kP=3670016  ,8.4ms
#define PM_PRC_8            3       //8 times       kP=7864320  ,14.8ms
#define PM_PRC_16           4       //16 times      kP=253952   ,27.6ms
#define PM_PRC_32           5       //32 times      kP=516096   ,53.2ms
#define PM_PRC_64           6       //64 times      kP=1040384  ,104.4ms
#define PM_PRC_128          7       //128 times     kP=2088960  ,206.8ms

//温度测量速率(sample/sec),Background 模式使用
#define  TMP_RATE_1         (0<<4)      //1 measurements pr. sec.
#define  TMP_RATE_2         (1<<4)      //2 measurements pr. sec.
#define  TMP_RATE_4         (2<<4)      //4 measurements pr. sec.           
#define  TMP_RATE_8         (3<<4)      //8 measurements pr. sec.
#define  TMP_RATE_16        (4<<4)      //16 measurements pr. sec.
#define  TMP_RATE_32        (5<<4)      //32 measurements pr. sec.
#define  TMP_RATE_64        (6<<4)      //64 measurements pr. sec.
#define  TMP_RATE_128       (7<<4)      //128 measurements pr. sec.

//温度重采样速率(times),Background 模式使用
#define TMP_PRC_1           0       //Sigle
#define TMP_PRC_2           1       //2 times
#define TMP_PRC_4           2       //4 times
#define TMP_PRC_8           3       //8 times
#define TMP_PRC_16          4       //16 times
#define TMP_PRC_32          5       //32 times
#define TMP_PRC_64          6       //64 times
#define TMP_PRC_128         7       //128 times

#define TMP_EXTER_SEN       (1<<7)  //MEMS
#define TMP_INTER_SEN       0       //ASIC

//SPL06_MEAS_CFG
#define MEAS_COEF_RDY       0x80
#define MEAS_SENSOR_RDY     0x40        //传感器初始化完成
#define MEAS_TMP_RDY        0x20        //有新的温度数据
#define MEAS_PRS_RDY        0x10        //有新的气压数据

#define MEAS_CTRL_Standby               0x00    //空闲模式
#define MEAS_CTRL_PressMeasure          0x01    //单次气压测量
#define MEAS_CTRL_TempMeasure           0x02    //单次温度测量
#define MEAS_CTRL_ContinuousPress       0x05    //连续气压测量
#define MEAS_CTRL_ContinuousTemp        0x06    //连续温度测量
#define MEAS_CTRL_ContinuousPressTemp   0x07    //连续气压温度测量

//FIFO_STS
#define SPL06_FIFO_FULL     0x02
#define SPL06_FIFO_EMPTY    0x01

//INT_STS
#define SPL06_INT_FIFO_FULL     0x04
#define SPL06_INT_TMP           0x02
#define SPL06_INT_PRS           0x01

//REGISTER
#define SPL06_CFG_T_SHIFT   0x08    //oversampling times>8时必须使用
#define SPL06_CFG_P_SHIFT   0x04

#define SPL06_PSR_B2     0x00        //气压值
#define SPL06_PSR_B1     0x01
#define SPL06_PSR_B0     0x02
#define SPL06_TMP_B2     0x03        //温度值
#define SPL06_TMP_B1     0x04
#define SPL06_TMP_B0     0x05
		  
#define SPL06_PSR_CFG    0x06        //气压测量配置
#define SPL06_TMP_CFG    0x07        //温度测量配置
#define SPL06_MEAS_CFG   0x08        //测量模式配置
		  
#define SPL06_CFG_REG    0x09
#define SPL06_INT_STS    0x0A
#define SPL06_FIFO_STS   0x0B
		  
#define SPL06_RESET      0x0C
#define SPL06_DEV_ID     0x0D
		  
#define SPL06_COEF       0x10        //-0x21
#define SPL06_COEF_SRCE  0x28

/*BMP280 校准参数*/
typedef struct
{
	INT16S C0;
	INT16S C1;
	INT32S C00;
	INT32S C10;
	INT16S C01;
	INT16S C11;
	INT16S C20;
	INT16S C21;
	INT16S C30;

	FP32 kT;
	FP32 kP;
}SPLCaliPara_t;

typedef struct
{
    struct
    {
        INT8U InitOK : 1;
        INT8U IsStable : 1;
        INT8U Reserve : 7;
    }Status;
    
    INT32S RawPressure;     //读出的芯片原始数据
    INT32S RawTemperature;
    
    FP32 Pressure;
    FP32 Temperature;
    FP32 Altitude;      //m
}SPLInfo_t;

void SPL06Init(void);
void SPL06Update(void);


extern volatile SPLInfo_t  g_SPLCtrlMsg;

#endif



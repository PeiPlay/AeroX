#include "SPL06_001.h"

volatile static SPLCaliPara_t   s_CaliPara;
volatile SPLInfo_t  g_SPLCtrlMsg;

static FP32 s_BaroGndPressure = 101325.0;
static FP32 s_BaroGndAltitude = 0;
static INT32U s_BaroCaliTimeout = 0;

static void SPLWriteReg(INT8U Reg, INT8U Data);
static INT8U SPLReadReg(INT8U Reg);
static void SPLReadContinul(INT8U Reg, INT8U *pRxBuff, INT16U Len);
static void SPLReadContinul(INT8U Reg, INT8U *pRxBuff, INT16U Len);
static void SPLSoftReset(void);

static void SPLCfgTemper(INT8U Rate, INT8U Oversampling, SPLCaliPara_t *CaliPara);
static void SPLCfgPressure(INT8U Rate, INT8U Oversampling, SPLCaliPara_t *CaliPara);
static void SPLStartup(INT8U Mode);
static INT32S SPLGetPressure(void);
static INT32S SPLGetTemper(void);
static FP32 PressureToAltitude(FP32 Pressure);

static INT32S BaroMedianFilter(INT32S NewPressure);
static void BaroCalibration(FP32 Pressure);

#define CONST_PF 0.1902630958	//(1/5.25588f) Pressure factor
#define FIX_TEMP 20				// Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
								// TLDR: Adjusting for temp changes does more harm than good.

#define MEDIAN_FILTER_LEN       3
#define MAX_DELTA_ERROR         1000


void SPL06Init(void)
{
    INT8U Coef[18];
    
    SPL_DelayMS(2);
    SPLSoftReset();
    SPL_DelayMS(100);
    
    INT8U ID = SPLReadReg(SPL06_DEV_ID);
    
    if(ID == 0x10)
    {
        SPLReadContinul(SPL06_COEF, Coef, 18);
        s_CaliPara.C0 = ((INT16S)Coef[0] << 4) + ((Coef[1] & 0xF0) >> 4);
        s_CaliPara.C0 = (s_CaliPara.C0 & 0x0800) ? (0xF000 | s_CaliPara.C0) : s_CaliPara.C0;
        s_CaliPara.C1 = ((INT16S)(Coef[1] & 0x0F) << 8) + Coef[2];
        s_CaliPara.C1 = (s_CaliPara.C1 & 0x0800) ? (0xF000 | s_CaliPara.C1) : s_CaliPara.C1;
        s_CaliPara.C00 = ((INT32S)Coef[3] << 12) + ((INT32S)Coef[4] << 4) + (Coef[5] >> 4);
        s_CaliPara.C00 = (s_CaliPara.C00 & 0x080000) ? (0xFFF00000 | s_CaliPara.C00) : s_CaliPara.C00;
        s_CaliPara.C10 = ((INT32S)(Coef[5] & 0x0F) << 16) + ((INT32S)Coef[6] << 8) + Coef[7];
        s_CaliPara.C10 = (s_CaliPara.C10 & 0x080000) ? (0xFFF00000 | s_CaliPara.C10) : s_CaliPara.C10;
        s_CaliPara.C01 = ((INT16S)Coef[8] << 8) + Coef[9];
        s_CaliPara.C11 = ((INT16S)Coef[10] << 8) + Coef[11];
        s_CaliPara.C20 = ((INT16S)Coef[12] << 8) + Coef[13];
        s_CaliPara.C21 = ((INT16S)Coef[14] << 8) + Coef[15];
        s_CaliPara.C30 = ((INT16S)Coef[16] << 8) + Coef[17];
        
        SPLCfgPressure(PM_RATE_8, PM_PRC_32, (SPLCaliPara_t *)&s_CaliPara);//每秒更新8次，125ms更新一次
        SPLCfgTemper(TMP_RATE_8, TMP_PRC_2, (SPLCaliPara_t *)&s_CaliPara);
        //每次采样3.6ms，配置总采样时间之和小于1s
        
        SPLStartup(MEAS_CTRL_ContinuousPressTemp);
        
        SPL_DelayMS(150);   //等200MS让气压计参数慢慢升上来，避免medianfilter锁死
        
        g_SPLCtrlMsg.Status.InitOK = 1;
    }
    else
    {
        g_SPLCtrlMsg.Status.InitOK = 0;
    }
}

void SPL06Update(void)
{
    FP32 RawTemper,RawPressure;
    FP32 Temp;
    FP32 Qua2,Qua3;
    
    g_SPLCtrlMsg.RawTemperature = RawTemper = (FP32)SPLGetTemper();
    g_SPLCtrlMsg.RawPressure = RawPressure = (FP32)SPLGetPressure();
    
    RawTemper /= s_CaliPara.kT;
    RawPressure /= s_CaliPara.kP;
    
    g_SPLCtrlMsg.Temperature = 0.5f*s_CaliPara.C0 + RawTemper * s_CaliPara.C1;
    
    Qua2 = s_CaliPara.C10 + RawPressure * (s_CaliPara.C20 + RawPressure * s_CaliPara.C30);
    Qua3 = RawTemper * RawPressure * (s_CaliPara.C11 + RawPressure * s_CaliPara.C21);
    g_SPLCtrlMsg.Pressure = s_CaliPara.C00 + RawPressure * Qua2 + RawTemper * s_CaliPara.C01 + Qua3;
    
    g_SPLCtrlMsg.Pressure = BaroMedianFilter(g_SPLCtrlMsg.Pressure * 10.0f) / 10.0f;    //整形计算，保留小数
    
    if(!g_SPLCtrlMsg.Status.IsStable)
    {
        BaroCalibration(g_SPLCtrlMsg.Pressure);
        g_SPLCtrlMsg.Altitude = 0;
    }
    else
    {
        g_SPLCtrlMsg.Altitude = PressureToAltitude(g_SPLCtrlMsg.Pressure) - s_BaroGndAltitude;
    }
}

static void SPLWriteReg(INT8U Reg, INT8U Data)
{
    INT8U s_SendBuff[2];
    INT8U s_ReceiveBuff[2];

    s_SendBuff[0] = Reg;
    s_SendBuff[1] = Data;
    
    SPL_ENABLE();
    SPL_SPI_TR(s_SendBuff, s_ReceiveBuff, 2, 1000);
    SPL_DISABLE();
}

static INT8U SPLReadReg(INT8U Reg)
{
    INT8U Transmit[2]={0};
    INT8U Receive[2]={0};
    
    Transmit[0] = Reg | SPL_READ;
    
    SPL_ENABLE();
    SPL_SPI_TR(Transmit, Receive, 2, 1000);
    SPL_DISABLE();
    
    return Receive[1];
}

static void SPLReadContinul(INT8U Reg, INT8U *pRxBuff, INT16U Len)
{
    INT8U Transmit;
    
    Transmit = Reg | SPL_READ;
    
    SPL_ENABLE();
    SPL_SPI_TX(&Transmit, 1, 10);
    SPL_SPI_RX(pRxBuff, Len, 10);
    SPL_DISABLE();   
}

static void SPLCfgTemper(INT8U Rate, INT8U Oversampling, SPLCaliPara_t *CaliPara)
{
	INT8U temp;
    
	switch (Oversampling)
	{
        case TMP_PRC_1:
            CaliPara->kT = 524288;
            break;
        case TMP_PRC_2:
            CaliPara->kT = 1572864;
            break;
        case TMP_PRC_4:
            CaliPara->kT = 3670016;
            break;
        case TMP_PRC_8:
            CaliPara->kT = 7864320;
            break;
        case TMP_PRC_16:
            CaliPara->kT = 253952;
            break;
        case TMP_PRC_32:
            CaliPara->kT = 516096;
            break;
        case TMP_PRC_64:
            CaliPara->kT = 1040384;
            break;
        case TMP_PRC_128:
            CaliPara->kT = 2088960;
            break;
	}
    SPLWriteReg(SPL06_TMP_CFG, Rate|Oversampling|TMP_EXTER_SEN);
	if (Oversampling > TMP_PRC_8)
	{
        temp = SPLReadReg(SPL06_CFG_REG);
        SPLWriteReg(SPL06_CFG_REG, temp | SPL06_CFG_T_SHIFT);
	}
}

static void SPLCfgPressure(INT8U Rate, INT8U Oversampling, SPLCaliPara_t *CaliPara)
{
	INT8U temp;
    
	switch (Oversampling)
	{
        case PM_PRC_1:
            CaliPara->kP = 524288;
            break;
        case PM_PRC_2:
            CaliPara->kP = 1572864;
            break;
        case PM_PRC_4:
            CaliPara->kP = 3670016;
            break;
        case PM_PRC_8:
            CaliPara->kP = 7864320;
            break;
        case PM_PRC_16:
            CaliPara->kP = 253952;
            break;
        case PM_PRC_32:
            CaliPara->kP = 516096;
            break;
        case PM_PRC_64:
            CaliPara->kP = 1040384;
            break;
        case PM_PRC_128:
            CaliPara->kP = 2088960;
            break;
	}
    SPLWriteReg(SPL06_PSR_CFG, Rate|Oversampling);
	if (Oversampling > PM_PRC_8)
	{
        temp = SPLReadReg(SPL06_CFG_REG);
        SPLWriteReg(SPL06_CFG_REG, temp | SPL06_CFG_P_SHIFT);
	}
}

static void SPLStartup(INT8U Mode)
{
	SPLWriteReg(SPL06_MEAS_CFG, Mode);
}

static void SPLSoftReset(void)
{
    SPLWriteReg(SPL06_RESET, 0x09);
}

static INT32S SPLGetPressure(void)
{
    INT8U Buff[3];
    INT32S Pressure;
    
    SPLReadContinul(SPL06_PSR_B2, Buff, 3);
	Pressure = (INT32S)(Buff[0] << 16) + (INT32S)(Buff[1] << 8) + Buff[2];
	Pressure = (Pressure & 0x800000) ? (0xFF000000 | Pressure) : Pressure;
    
    return Pressure;
}

static INT32S SPLGetTemper(void)
{
    INT8U Buff[3];
    INT32S Temper;
    
    SPLReadContinul(SPL06_TMP_B2, Buff, 3);
	Temper = (INT32S)(Buff[0] << 16) + (INT32S)(Buff[1] << 8) + Buff[2];
	Temper = (Temper & 0x800000) ? (0xFF000000 | Temper) : Temper;
    
    return Temper;    
}

//单位Pa
static FP32 PressureToAltitude(FP32 Pressure)
{
    FP32 Altitude;
    
    if(Pressure > 0)
    {
        //Converts pressure to altitude above sea level (ASL) in meters
        //Altitude = 44330.77*(1-pow(Pressure / 101325, (1/5.256)));
        
        //温度补偿算法
        Altitude = ((pow((101570.0f / Pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;
    }
    else
    {
        Altitude = 0;
    }
    
    return Altitude;
}

//气压计1个标准大气压校准
static void BaroCalibration(FP32 Pressure)
{
	//慢慢收敛校准
    const FP32 PressureError = Pressure - s_BaroGndPressure;
    s_BaroGndPressure += PressureError * 0.15f;

    if (MyFP32Abs(PressureError) < (s_BaroGndPressure * 0.00005f))  // 0.005% calibration error (should give c. 10cm calibration error)
	{
        if ((g_SysTickTime - s_BaroCaliTimeout) > 250) 
		{
            s_BaroGndAltitude = PressureToAltitude(s_BaroGndPressure);
            g_SPLCtrlMsg.Status.IsStable = 1;
        }
    }
    else 
	{
        s_BaroCaliTimeout = g_SysTickTime;
    }
}

static INT32S BaroMedianFilter(INT32S NewPressure)
{
    static INT32S FilterBuff[MEDIAN_FILTER_LEN];
    static INT32U FilterIndex = 0;
    static BOOLEAN FilterIsReady = 0;
    
    INT32S NextIndex = FilterIndex + 1;
    if (NextIndex == MEDIAN_FILTER_LEN) 
	{
        NextIndex = 0;
        FilterIsReady = 1;
    }
    
    INT32S LastIndex = FilterIndex - 1;
    if (LastIndex < 0) 
	{
        LastIndex = MEDIAN_FILTER_LEN - 1;
    }
    
    const INT32S LastPressure = FilterBuff[LastIndex];

    if (FilterIsReady) 
	{
        if (MyINT32SAbs(LastPressure - NewPressure) < MAX_DELTA_ERROR)
		{
            FilterBuff[FilterIndex] = NewPressure;
            FilterIndex = NextIndex;
            return QuickMedian3_INT32S(FilterBuff);
        } 
		else
		{
            // glitch threshold exceeded, so just return previous reading and don't add the glitched reading to the filter array
            return FilterBuff[LastIndex];
        }
    } 
	else 
	{
        FilterBuff[FilterIndex] = NewPressure;
        FilterIndex = NextIndex;
        return NewPressure;
    }
}
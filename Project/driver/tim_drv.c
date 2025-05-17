#include "tim_drv.h"

/**
 * @brief 获取定时器的时钟总线和位宽信息
 * @param TIMx 定时器实例指针
 * @param is_apb1 指向存储是否挂载在APB1总线的变量的指针（1表示APB1，0表示APB2）
 * @param is_16bits 指向存储是否是16位定时器的变量的指针（1表示16位，0表示32位）
 * @return 成功返回0，失败返回非0值
 */
uint8_t TIM_GetClockInfo(TIM_TypeDef* TIMx, uint8_t* is_apb1, uint8_t* is_16bits)
{
    // 检查参数有效性
    if (TIMx == NULL || is_apb1 == NULL || is_16bits == NULL)
    {
        return 1; // 参数错误
    }
    
    // 默认值
    *is_apb1 = 0;
    *is_16bits = 1; // 大多数定时器是16位的
    
    // 判断定时器类型和总线
    if (TIMx == TIM1)
    {
        *is_apb1 = 0; // APB2
        *is_16bits = 1; // 16位
    }
    else if (TIMx == TIM2)
    {
        *is_apb1 = 1; // APB1
        *is_16bits = 0; // 32位
    }
    else if (TIMx == TIM3 || TIMx == TIM4)
    {
        *is_apb1 = 1; // APB1
        *is_16bits = 1; // 16位
    }
    else if (TIMx == TIM5)
    {
        *is_apb1 = 1; // APB1
        *is_16bits = 0; // 32位
    }
    else if (TIMx == TIM6 || TIMx == TIM7)
    {
        *is_apb1 = 1; // APB1
        *is_16bits = 1; // 16位
    }
#ifdef TIM8
    else if (TIMx == TIM8)
    {
        *is_apb1 = 0; // APB2
        *is_16bits = 1; // 16位
    }
#endif
#ifdef TIM9
    else if (TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11)
    {
        *is_apb1 = 0; // APB2
        *is_16bits = 1; // 16位
    }
#endif
#ifdef TIM12
    else if (TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14)
    {
        *is_apb1 = 1; // APB1
        *is_16bits = 1; // 16位
    }
#endif
#ifdef TIM15
    else if (TIMx == TIM15 || TIMx == TIM16 || TIMx == TIM17)
    {
        *is_apb1 = 0; // APB2
        *is_16bits = 1; // 16位
    }
#endif
#ifdef TIM18
    else if (TIMx == TIM18 || TIMx == TIM19)
    {
        *is_apb1 = 0; // APB2
        *is_16bits = 1; // 16位
    }
#endif
#ifdef TIM20
    else if (TIMx == TIM20)
    {
        *is_apb1 = 0; // APB2
        *is_16bits = 1; // 16位
    }
#endif
#ifdef TIM21
    else if (TIMx == TIM21 || TIMx == TIM22)
    {
        *is_apb1 = 0; // APB2
        *is_16bits = 1; // 16位
    }
#endif
#ifdef TIM23
    else if (TIMx == TIM23 || TIMx == TIM24)
    {
        *is_apb1 = 1; // APB1
        *is_16bits = 0; // 32位
    }
#endif
    else
    {
        return 2; // 未知定时器
    }
    
    return 0; // 成功
}

/**
 * @brief 获取定时器的时钟频率
 * @param htim 定时器句柄指针
 * @return 定时器时钟频率（Hz）
 */
uint32_t TimDrv_GetPeriphCLKFreq(const TIM_HandleTypeDef *htim)
{
    uint8_t is_apb1, is_16bits;
    uint32_t pclk = 0;
    uint32_t freq = 0;

    // 获取定时器的时钟总线和位宽信息
    if (TIM_GetClockInfo(htim->Instance, &is_apb1, &is_16bits) != 0)
    {
        return 0; // 错误处理
    }

    // 获取APB时钟频率
    if (is_apb1)
    {
        pclk = HAL_RCC_GetPCLK1Freq();
        #if defined(RCC_D2CFGR_D2PPRE1)
            if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != 0)
            {
                pclk *= 2; // APB1时钟频率翻倍
            }
        #else
            if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
            {
                pclk *= 2; // APB1时钟频率翻倍
            }
        #endif
    }
    else
    {
        pclk = HAL_RCC_GetPCLK2Freq();
        #if defined(RCC_D2CFGR_D2PPRE2)
            if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) != 0)
            {
                pclk *= 2; // APB2时钟频率翻倍
            }
        #else
            if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0)
            {
                pclk *= 2; // APB2时钟频率翻倍
            }
        #endif
    }
    return pclk; // 返回APB时钟频率
}

//开启定时器
void TimDrv_Start(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Start_IT(htim);
    HAL_TIM_Base_Start(htim);
}
//关闭定时器
void TimDrv_Shutdown(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop_IT(htim);
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(htim, 0);
}

//为使得定时器以freq频率溢出，计算预分频器和自动重装载寄存器的值
uint8_t TimDrv_CalcPscAndAtr(TIM_HandleTypeDef *htim, double freq, uint32_t *psc, uint32_t *atr)
{
    uint8_t is_apb1, is_16bits;
    uint64_t timer_clk;
    uint64_t intPeriodCnt;
    uint32_t prescaler = 1; // 计算值 (PSC = prescaler - 1)
    uint32_t period = 1;    // 计算值 (ARR = period - 1)
    uint64_t best_product = 0;
    uint64_t max_arr_val; // 定时器ARR寄存器的最大值

    // 检查输入频率是否有效
    if (freq <= 0)
    {
        return 3; // 无效频率
    }

    timer_clk = TimDrv_GetPeriphCLKFreq(htim); // 获取定时器时钟
    if (timer_clk == 0)
    {
        return 4; // 无法获取定时器时钟
    }

    intPeriodCnt = (uint64_t)((double)timer_clk / freq);
    if (intPeriodCnt == 0) // 如果目标周期计数为0 (频率过高)
    {
        return 5; // 目标频率过高
    }

    // 获取定时器位宽信息
    if (TIM_GetClockInfo(htim->Instance, &is_apb1, &is_16bits) != 0)
    {
        return 1; // 获取定时器信息失败
    }

    if (is_16bits)
    {
        max_arr_val = 0xFFFF; // 16位ARR最大值
    }
    else
    {
        max_arr_val = 0xFFFFFFFF; // 32位ARR最大值
    }

    // 预分频器最大值为 65536 (PSC = 0 to 65535)
    // 循环查找最佳的 prescaler (i) 和 period (j)
    // prescaler = i, period = j
    // PSC = i - 1, ARR = j - 1
    for (uint32_t i = 1; i <= 65536; ++i) {
        // 检查 intPeriodCnt / i 是否会导致溢出或结果为0
        if (intPeriodCnt < i) continue; // 避免除以0或得到小于1的周期

        uint64_t j_64 = intPeriodCnt / i; // 计算所需的周期 (ARR+1)

        // 检查计算出的周期 j 是否在有效范围内 [1, max_arr_val + 1]
        // j_64 必须大于 0
        // j_64 - 1 (即 ARR) 必须小于等于 max_arr_val
        if (j_64 == 0 || (j_64 - 1) > max_arr_val) continue;

        uint32_t j = (uint32_t)j_64; // 此时 j_64 肯定在 uint32_t 范围内 (因为 i>=1, j_64 <= intPeriodCnt)
                                     // 对于32位定时器，如果 j_64 > 0xFFFFFFFF，上面的检查已经 continue 了

        uint64_t product = (uint64_t)i * j; // 使用64位计算乘积

        // 寻找最接近 intPeriodCnt 的乘积 (且不超过 intPeriodCnt)
        // 这会优先选择较大的 period (j)，从而可能获得更好的分辨率
        if (product > best_product) {
            best_product = product;
            prescaler = i;
            period = j;
        }
		// 如果找到了精确匹配，可以提前退出 (可选优化)
		if(product == intPeriodCnt)
			break;
    }

    // 检查是否找到了有效的 prescaler 和 period
    if (best_product == 0) {
        // 如果没有找到合适的组合 (例如 freq 太高或太低)
        return 2; // 找不到合适的 PSC 和 ARR 组合
    } else {
        *psc = prescaler - 1; // 寄存器值 = 计算值 - 1
        *atr = period - 1;   // 寄存器值 = 计算值 - 1
    }

    return 0; // 成功
}
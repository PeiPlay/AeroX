#include "ws2812.h" // 更新包含的头文件
#include <string.h>
#include <stdlib.h>
#include "tim_drv.h" // 包含 tim_drv 头文件

// 重命名函数 Ws2812_Init，使用 Ws2812_t
void Ws2812_Init(Ws2812_t* ws2812) {
	if(ws2812->htim == NULL) return;

    // 获取定时器基础时钟频率
    uint32_t tim_freq = TimDrv_GetPeriphCLKFreq(ws2812->htim);
    if (tim_freq == 0) {
        // 错误处理：无法获取定时器时钟频率
        return;
    }

    //为使频率为800KHz，设置定时器的自动重装载寄存器为tim_freq/800000
    // 计算定时器的自动重装载寄存器值 (ARR)
    uint32_t arr = (tim_freq / 800000) - 1; // 减去1是因为ARR是从0开始计数的

    // 设置定时器的自动重装载寄存器值
    __HAL_TIM_SET_AUTORELOAD(ws2812->htim, arr); // 设置自动重装载寄存器值
    
    // 设置定时器的预分频器值 (PSC) 为0
    __HAL_TIM_SET_PRESCALER(ws2812->htim, 0); // 设置预分频器值

    // 设置定时器的计数器初始值为0
    __HAL_TIM_SET_COUNTER(ws2812->htim, 0); // 设置计数器初始值为0

    //计算cnt_one和cnt_zero的值(arr的%70和%30)
    ws2812->cnt_one = (arr * 70) / 100; // 70% 的占空比
    ws2812->cnt_zero = (arr * 30) / 100; // 30% 的占空比

	// 使用 memset 清零缓冲区
	memset(ws2812->color_buf, 0, sizeof(ws2812->color_buf));
}

// 重命名函数 Ws2812_SetColor，使用 Ws2812_t
// 调整 color_buf 的访问方式
void Ws2812_SetColor(Ws2812_t* ws2812, uint16_t r, uint16_t g, uint16_t b) {
    if(ws2812->cnt_one == 0 || ws2812->cnt_zero == 0)
    {
        // 如果 cnt_one 和 cnt_zero 没有初始化，调用初始化函数
        Ws2812_Init(ws2812);
    }

    if(ws2812->htim == NULL) return;
	if(r > 255) r = 255;
	if(g > 255) g = 255;
	if(b > 255) b = 255;

    // 注意：这里的逻辑假定缓冲区大小和填充方式适用于 WS2812 协议
    // 70 和 30 是 PWM 占空比，对应 WS2812 的 1 和 0 码
    // 索引 237, 245, 253 可能与特定的 LED 数量或缓冲区布局有关
    // 需要确保 WS2812_BUFFER_SIZE 足够大

#ifndef stm32f0xx
if(ws2812->htim->Instance == TIM2 || ws2812->htim->Instance == TIM5)	//TIM2与TIM5的自动重装寄存器为32位
	{
		// 直接访问 color_buf 数组，类型已经是 uint32_t
		uint32_t* color_buf = ws2812->color_buf;
		for(int8_t i = 7; i>=0; i--) color_buf[237-i] = ((g >>i)&0x01) ? ws2812->cnt_one : ws2812->cnt_zero;
		for(int8_t i = 7; i>=0; i--) color_buf[245-i] = ((r >>i)&0x01) ? ws2812->cnt_one : ws2812->cnt_zero;
		for(int8_t i = 7; i>=0; i--) color_buf[253-i] = ((b >>i)&0x01) ? ws2812->cnt_one : ws2812->cnt_zero;
	}
	else	//其余定时器的自动重装寄存器为16位
#endif
	{
        // 如果 TIM ARR 是 16 位，DMA 数据宽度通常也应配置为 16 位 (HalfWord)
        // 这里仍然写入 uint32_t 数组，但 HAL 库会处理传输宽度
        // 或者，可以将 color_buf 定义为 uint16_t，并相应调整 DMA 配置
		uint32_t* color_buf = ws2812->color_buf; // 保持 uint32_t 指针类型以匹配 DMA 函数
		for(int8_t i = 7; i>=0; i--) color_buf[237-i] = ((g >>i)&0x01) ? ws2812->cnt_one : ws2812->cnt_zero;
		for(int8_t i = 7; i>=0; i--) color_buf[245-i] = ((r >>i)&0x01) ? ws2812->cnt_one : ws2812->cnt_zero;
		for(int8_t i = 7; i>=0; i--) color_buf[253-i] = ((b >>i)&0x01) ? ws2812->cnt_one : ws2812->cnt_zero;
	}
    // DMA 函数直接使用 color_buf 地址
	HAL_TIM_PWM_Start_DMA(ws2812->htim, ws2812->channel, ws2812->color_buf, WS2812_BUFFER_SIZE);
}

// 重命名函数 Ws2812_SetColorRGB，使用 Ws2812_t 和 Ws2812_RGB_t
void Ws2812_SetColorRGB(Ws2812_t* ws2812, Ws2812_RGB_t rgb) {
    // 调用重命名后的函数 Ws2812_SetColor
	Ws2812_SetColor(ws2812, rgb.r, rgb.g, rgb.b);
}

#ifdef __cplusplus
// C++ Class Implementation

// Constructor implementation
Ws2812::Ws2812(TIM_HandleTypeDef *htim, uint32_t channel) {
    ws2812_handle.htim = htim;
    ws2812_handle.channel = channel;
}

// Member function implementation to initialize WS2812
void Ws2812::init() {
    Ws2812_Init(&ws2812_handle); // Call the C function
}

// Member function implementation to set color using R, G, B values
void Ws2812::setColor(uint16_t r, uint16_t g, uint16_t b) {
    Ws2812_SetColor(&ws2812_handle, r, g, b); // Call the C function
}

// Member function implementation to set color using Ws2812_RGB_t struct
void Ws2812::setColorRGB(Ws2812_RGB_t rgb) {
    Ws2812_SetColorRGB(&ws2812_handle, rgb); // Call the C function
}

#endif // __cplusplus
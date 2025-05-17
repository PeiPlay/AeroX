#pragma once
#include "main.h"
#include "tim.h"

// 使用 Ws2812_RGB_t 代替 PwmLed_RGB_t
typedef struct{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} Ws2812_RGB_t;

// 使用 Ws2812_t 代替 PwmLed_t
// 将 color_buf 定义为实际存储数据的数组，而不是指针数组
// 缓冲区大小 255 可能需要根据实际 LED 数量和协议要求调整
#define WS2812_BUFFER_SIZE 255 // 定义缓冲区大小宏
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t color_buf[WS2812_BUFFER_SIZE]; // 直接存储 PWM 数据的缓冲区
    uint32_t cnt_one;
    uint32_t cnt_zero;
} Ws2812_t;

// 重命名宏定义
#define WS2812_RGB_BLACK   (Ws2812_RGB_t){0,   0,   0}
#define WS2812_RGB_WHITE   (Ws2812_RGB_t){30,   30,   30}
#define WS2812_RGB_RED     (Ws2812_RGB_t){90,  0,   0}
#define WS2812_RGB_GREEN   (Ws2812_RGB_t){0,   90,  0}
#define WS2812_RGB_BLUE    (Ws2812_RGB_t){0,   0,   90}
#define WS2812_RGB_YELLOW  (Ws2812_RGB_t){45,  45,  0}
#define WS2812_RGB_PURPLE  (Ws2812_RGB_t){45,  0,   45}
#define WS2812_RGB_CYAN    (Ws2812_RGB_t){0,   45,  45}

// 添加 extern "C" 以支持 C++
#ifdef __cplusplus
extern "C" {
#endif

// 重命名函数声明
void Ws2812_Init(Ws2812_t* ws2812);
void Ws2812_SetColor(Ws2812_t* ws2812, uint16_t r, uint16_t g, uint16_t b);
void Ws2812_SetColorRGB(Ws2812_t* ws2812, Ws2812_RGB_t rgb);

#ifdef __cplusplus
} // extern "C"

// C++ Class Wrapper
class Ws2812 {
private:
    Ws2812_t ws2812_handle; // Encapsulated C struct instance

public:
    // Constructor declaration
    Ws2812(TIM_HandleTypeDef *htim, uint32_t channel);

    void init(); // Member function to initialize WS2812

    // Member function declaration to set color using R, G, B values
    void setColor(uint16_t r, uint16_t g, uint16_t b);

    // Member function declaration to set color using Ws2812_RGB_t struct
    void setColorRGB(Ws2812_RGB_t rgb);

};

#endif // __cplusplus



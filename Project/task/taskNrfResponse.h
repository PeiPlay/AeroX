#ifndef TASK_NRF_RESPONSE_H
#define TASK_NRF_RESPONSE_H

#include "main.h"
#include "cmsis_os.h"

// 结构体定义保持在 extern "C" 之外
typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        uint8_t vesc:8;         //VESC连接异常
        uint8_t as5047:8;       //磁编连接异常
        uint8_t azimuth:4;      //航向电机复位角度异常
        uint8_t dt35:3;         //DT35连接异常
        uint8_t gyro:1;         //陀螺仪连接异常
        uint8_t openmv:2;       //OpenMV连接异常
        uint8_t fetcher:2;      //取球VESC连接异常
        uint8_t htdw:2;         //瓣膜高擎电机连接异常
        uint8_t vib_motor:1;    //震动马达
        uint8_t reserve:1;      //保留位
    } chassis;  //4bytes
    struct __attribute__((packed))
    {
        uint8_t m2006:4;        //M2006连接异常
        uint8_t encoder:4;      //编码器连接异常
    } seedling; //1bytes
    struct __attribute__((packed))
    {
        uint8_t vesc:3;         //VESC连接异常
        uint8_t as5047:3;       //磁编连接异常
        uint8_t bsqjn:1;        //压力传感器连接异常
        uint8_t m2006_roller:1; //M2006连接异常
        uint8_t encoder:1;      //编码器连接异常        
        uint8_t m3508_blade:2;  //M3508连接异常
        uint8_t m2006_clamp:4;  //M2006连接异常
        uint8_t reserve:1;      //保留位
    } shoot;    //2bytes
} NrfCommu_BitsWarning_t;   //7bytes

typedef struct __attribute__((packed))
{
    float gyro;         //陀螺仪数据
    uint8_t openmv_left;  //OpenMV左侧数据
    uint8_t openmv_right; //OpenMV右侧数据
} NrfCommu_WatchValue_t;    //6bytes

typedef struct __attribute__((packed))
{
    NrfCommu_BitsWarning_t bits_warning;    //7bytes
    NrfCommu_WatchValue_t watch_value;      //6bytes
    char string[32 - sizeof(NrfCommu_BitsWarning_t) - sizeof(NrfCommu_WatchValue_t)]; //19
} NrfCommu_ReceivePackage_t;

#ifdef __cplusplus
extern "C" {
#endif

// 全局变量声明
extern NrfCommu_ReceivePackage_t nrf_response_package;

// 函数声明
void taskNrfResponse(void *argument);

#ifdef __cplusplus
}
#endif

// ========== 便捷宏定义 ==========
// 底盘警告位设置宏 - 支持位操作
#define NRF_SET_CHASSIS_VESC_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.chassis.vesc |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.vesc &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_CHASSIS_AS5047_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.chassis.as5047 |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.as5047 &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_CHASSIS_AZIMUTH_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.chassis.azimuth |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.azimuth &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_CHASSIS_DT35_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.chassis.dt35 |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.dt35 &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_CHASSIS_GYRO_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.chassis.gyro |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.gyro &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_CHASSIS_OPENMV_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.chassis.openmv |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.openmv &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_CHASSIS_FETCHER_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.chassis.fetcher |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.fetcher &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_CHASSIS_HTDW_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.chassis.htdw |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.htdw &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_CHASSIS_VIB_MOTOR_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.chassis.vib_motor |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.vib_motor &= ~(1 << (bit)); \
} while(0)

// 秧苗警告位设置宏 - 支持位操作
#define NRF_SET_SEEDLING_M2006_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.seedling.m2006 |= (1 << (bit)); \
    else nrf_response_package.bits_warning.seedling.m2006 &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_SEEDLING_ENCODER_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.seedling.encoder |= (1 << (bit)); \
    else nrf_response_package.bits_warning.seedling.encoder &= ~(1 << (bit)); \
} while(0)

// 射击警告位设置宏 - 支持位操作
#define NRF_SET_SHOOT_VESC_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.shoot.vesc |= (1 << (bit)); \
    else nrf_response_package.bits_warning.shoot.vesc &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_SHOOT_AS5047_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.shoot.as5047 |= (1 << (bit)); \
    else nrf_response_package.bits_warning.shoot.as5047 &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_SHOOT_BSQJN_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.shoot.bsqjn |= (1 << (bit)); \
    else nrf_response_package.bits_warning.shoot.bsqjn &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_SHOOT_M2006_ROLLER_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.shoot.m2006_roller |= (1 << (bit)); \
    else nrf_response_package.bits_warning.shoot.m2006_roller &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_SHOOT_ENCODER_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.shoot.encoder |= (1 << (bit)); \
    else nrf_response_package.bits_warning.shoot.encoder &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_SHOOT_M3508_BLADE_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.shoot.m3508_blade |= (1 << (bit)); \
    else nrf_response_package.bits_warning.shoot.m3508_blade &= ~(1 << (bit)); \
} while(0)

#define NRF_SET_SHOOT_M2006_CLAMP_BIT(bit, val) do { \
    if (val) nrf_response_package.bits_warning.shoot.m2006_clamp |= (1 << (bit)); \
    else nrf_response_package.bits_warning.shoot.m2006_clamp &= ~(1 << (bit)); \
} while(0)

// 线程安全版本的位操作宏
#define NRF_SET_CHASSIS_VESC_BIT_SAFE(bit, val) do { \
    taskENTER_CRITICAL(); \
    if (val) nrf_response_package.bits_warning.chassis.vesc |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.vesc &= ~(1 << (bit)); \
    taskEXIT_CRITICAL(); \
} while(0)

#define NRF_SET_CHASSIS_AS5047_BIT_SAFE(bit, val) do { \
    taskENTER_CRITICAL(); \
    if (val) nrf_response_package.bits_warning.chassis.as5047 |= (1 << (bit)); \
    else nrf_response_package.bits_warning.chassis.as5047 &= ~(1 << (bit)); \
    taskEXIT_CRITICAL(); \
} while(0)

// 监控数值设置宏（保持原有功能）
#define NRF_SET_GYRO(val)               (nrf_response_package.watch_value.gyro = (val))
#define NRF_SET_OPENMV_LEFT(val)        (nrf_response_package.watch_value.openmv_left = (val))
#define NRF_SET_OPENMV_RIGHT(val)       (nrf_response_package.watch_value.openmv_right = (val))

// 字符串设置宏（线程安全版本）
#define NRF_SET_STRING(str) do { \
    taskENTER_CRITICAL(); \
    strncpy(nrf_response_package.string, (str), sizeof(nrf_response_package.string) - 1); \
    nrf_response_package.string[sizeof(nrf_response_package.string) - 1] = '\0'; \
    taskEXIT_CRITICAL(); \
} while(0)

// 线程安全的数值设置宏
#define NRF_SET_VALUE_SAFE(field, val) do { \
    taskENTER_CRITICAL(); \
    (field) = (val); \
    taskEXIT_CRITICAL(); \
} while(0)

// 批量清零宏
#define NRF_CLEAR_ALL_WARNINGS() do { \
    taskENTER_CRITICAL(); \
    memset(&nrf_response_package.bits_warning, 0, sizeof(nrf_response_package.bits_warning)); \
    taskEXIT_CRITICAL(); \
} while(0)

#define NRF_CLEAR_WATCH_VALUES() do { \
    taskENTER_CRITICAL(); \
    memset(&nrf_response_package.watch_value, 0, sizeof(nrf_response_package.watch_value)); \
    taskEXIT_CRITICAL(); \
} while(0)

#endif // TASK_NRF_RESPONSE_H
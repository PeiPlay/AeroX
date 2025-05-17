#include "time_utils.h"
#include "cmsis_os.h" 
#include "main.h"


extern uint32_t SystemCoreClock;


#ifdef __cplusplus
extern "C" {
#endif

#define __TIMEUTILS_GET_SYSTICK_VAL() (SysTick->VAL)

uint64_t TimeUtils_GetGlobalTick(void)
{
    uint32_t ms_count1, ms_count2, systick_val_raw;
    uint64_t systick_load;

    systick_load = (uint64_t)SysTick->LOAD;

    do
    {
        ms_count1 = xTaskGetTickCount();
        systick_val_raw = __TIMEUTILS_GET_SYSTICK_VAL(); // 读取 SysTick 当前计数值 (递减)
        ms_count2 = xTaskGetTickCount();
    } while (ms_count1 != ms_count2);

    return (uint64_t)ms_count2 * (uint64_t)SystemCoreClock / 1000ULL + (systick_load - (uint64_t)systick_val_raw);
}

#undef __TIMEUTILS_GET_SYSTICK_VAL

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
namespace utils {
namespace time {

uint64_t getGlobalTick() {
    return TimeUtils_GetGlobalTick();
}

} // namespace time
} // namespace utils
#endif // __cplusplus


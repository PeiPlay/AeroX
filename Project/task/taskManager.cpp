#include "taskManager.h"
#include "config.h"
extern "C" {  // 添加 extern "C" 块
__weak void taskManager_Init(void* argument)
{
    upt201.init();
    osDelay(5);
    upt201.start();
    taskGroundStation_Init(argument);
}
}  // 结束 extern "C" 块


#include "scheduler.h"
#include "tim_drv.h"
#include <functional>
#include <new> 
#include <atomic>
#include <cstdint> // For uintptr_t

// --- 构造函数 ---
Scheduler::Scheduler(TIM_HandleTypeDef* htim, uint32_t intPeriod)
    : htim(htim), 
      mode(SchedulerMode::Obstructed), 
      intPeriod(intPeriod), 
      initialized(false), 
      listHead(nullptr), // 初始化链表头
      currentTaskCount(0), 
      nextTaskId(1), 
      requestQueue(nullptr), // 初始化队列句柄
      cleanupQueue(nullptr)
{
    // 构造函数体
}

// --- 析构函数 ---
Scheduler::~Scheduler() {
    shutdown(); 

    // 尝试最后一次清理 (可能在 OS 关闭后调用，需谨慎)
    // processCleanupQueue(); // 可能不安全

    // 直接清理剩余链表节点 (不通过队列)
    TaskNode* current = listHead;
    while (current != nullptr) {
        TaskNode* next = current->next;
        delete current; // 直接删除
        current = next;
    }
    listHead = nullptr;
    currentTaskCount = 0;

    // 删除消息队列
    if (requestQueue != nullptr) {
        osMessageQueueDelete(requestQueue);
        requestQueue = nullptr;
    }
    if (cleanupQueue != nullptr) {
        // 清理队列中可能还有指针，但对应的节点已在上面删除
        // 只需删除队列本身
        osMessageQueueDelete(cleanupQueue);
        cleanupQueue = nullptr;
    }
    
    // tasks 指针不再使用，移除相关 delete
}

// --- 初始化函数 ---
void Scheduler::init() {
    if (!htim || intPeriod == 0) {
        Error_Handler(); 
        return;
    }
    
    // 创建请求队列 (Queue A)
    if (requestQueue == nullptr) {
        osMessageQueueAttr_t reqAttr = { .name = "SchedReqQ" };
        requestQueue = osMessageQueueNew(16, sizeof(Scheduler::RequestMsg), &reqAttr); 
        if (requestQueue == nullptr) {
            Error_Handler(); 
            return;
        }
    }

    // 创建清理队列 (Queue B)
    if (cleanupQueue == nullptr) {
        osMessageQueueAttr_t cleanAttr = { .name = "SchedCleanQ" };
        cleanupQueue = osMessageQueueNew(16, sizeof(Scheduler::CleanupMsg), &cleanAttr); // 队列大小可调整
        if (cleanupQueue == nullptr) {
            Error_Handler(); 
            // 清理已创建的请求队列
            if (requestQueue != nullptr) {
                 osMessageQueueDelete(requestQueue);
                 requestQueue = nullptr;
            }
            return;
        }
    }
    
    // --- 定时器配置 (与之前相同) ---
    double freq = 1000000.0 / static_cast<double>(intPeriod);
    if (freq <= 0) { Error_Handler(); return; }
    uint32_t psc, atr;
    uint8_t calc_status = TimDrv_CalcPscAndAtr(htim, freq, &psc, &atr);
    if (calc_status != 0) { Error_Handler(); return; }
    HAL_TIM_Base_Stop_IT(htim);
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(htim, 0);
    htim->Init.Prescaler = psc;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = atr;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(htim) != HAL_OK) { Error_Handler(); return; }
    // --- 定时器配置结束 ---

    initialized = true; 
    currentTaskCount = 0; // 重置计数
    listHead = nullptr;   // 确保链表头为空
    
    start(); // 启动定时器
}

// --- 处理清理队列 ---
void Scheduler::processCleanupQueue() {
    if (!cleanupQueue) return;

    CleanupMsg cleanMsg;
    // 循环处理直到队列为空，使用 timeout 0 (非阻塞)
    while (osMessageQueueGet(cleanupQueue, &cleanMsg, NULL, 0) == osOK) {
        TaskNode* nodeToDelete = reinterpret_cast<TaskNode*>(cleanMsg.nodePtr);
        if (nodeToDelete) {
            delete nodeToDelete; // 释放节点内存
        }
    }
}

// --- 生成任务 ID ---
Scheduler::TaskId Scheduler::generateTaskId() {
    if (!initialized) return 0; 
    if (nextTaskId == 0) {
        nextTaskId = 1; 
    }
    TaskId id = nextTaskId++;
    return id;
}

// --- 移除任务请求 ---
bool Scheduler::removeTask(Scheduler::TaskId taskId) { 
    processCleanupQueue(); // 处理待清理节点

    if (taskId == 0 || !initialized || !requestQueue) return false; 
    
    // 创建移除请求消息
    RequestMsg reqMsg;
    reqMsg.operation = RequestOpType::REMOVE_TASK;
    reqMsg.data = taskId; // data 存储 TaskId
    
    // 发送消息到请求队列
    if (osMessageQueuePut(requestQueue, &reqMsg, 0, 10) != osOK) {
        return false; // 消息发送失败
    }
    
    return true; // 请求已成功发送
}

// --- 清空所有任务请求 ---
bool Scheduler::clearAllTasks() {
    // 注意：这里不直接清理，仅发送消息
    if (!initialized || !requestQueue) return false;

    RequestMsg reqMsg;
    reqMsg.operation = RequestOpType::CLEAR_ALL_TASKS;
    reqMsg.data = 0; // data 字段未使用

    if (osMessageQueuePut(requestQueue, &reqMsg, 0, 10) != osOK) {
        return false; // 消息发送失败
    }

    return true; // 请求已成功发送
}


// --- 处理请求队列 (在中断回调中调用) ---
void Scheduler::processRequestQueue() {
    if (!initialized || !requestQueue || !cleanupQueue) return; 
    
    RequestMsg reqMsg;
    osStatus_t status;
    
    // 处理请求队列中的所有消息
    while ((status = osMessageQueueGet(requestQueue, &reqMsg, NULL, 0)) == osOK) {
        switch (reqMsg.operation) {
            case RequestOpType::ADD_TASK: {
                TaskNode* newNode = reinterpret_cast<TaskNode*>(reqMsg.data);
                if (newNode) {
                    // 插入到链表头部
                    newNode->next = listHead;
                    listHead = newNode;
                    currentTaskCount++; // 增加计数
                }
                break;
            }
            case RequestOpType::REMOVE_TASK: {
                TaskId idToRemove = static_cast<TaskId>(reqMsg.data);
                if (idToRemove != 0) {
                    TaskNode* current = listHead;
                    TaskNode* prev = nullptr;
                    bool removed = false;

                    while (current != nullptr) {
                        if (current->info.id == idToRemove) {
                            // 找到了要移除的节点
                            if (prev == nullptr) { // 移除的是头节点
                                listHead = current->next;
                            } else { // 移除的是中间或尾部节点
                                prev->next = current->next;
                            }
                            
                            // 将移除的节点指针放入清理队列
                            CleanupMsg cleanMsg;
                            cleanMsg.nodePtr = current;
                            // 假设清理队列不会满，忽略发送失败的情况 (简化处理)
                            osMessageQueuePut(cleanupQueue, &cleanMsg, 0, 0); 

                            currentTaskCount--; // 减少计数
                            removed = true;
                            break; // 找到并处理后退出循环
                        }
                        prev = current;
                        current = current->next;
                    }
                }
                break;
            }
            case RequestOpType::CLEAR_ALL_TASKS: {
                TaskNode* current = listHead;
                while (current != nullptr) {
                    TaskNode* nodeToClean = current;
                    current = current->next; // 先移动到下一个节点

                    // 将节点指针放入清理队列
                    CleanupMsg cleanMsg;
                    cleanMsg.nodePtr = nodeToClean;
                    osMessageQueuePut(cleanupQueue, &cleanMsg, 0, 0); // 忽略失败
                }
                listHead = nullptr; // 清空链表头
                currentTaskCount = 0; // 重置计数
                break;
            }
        } // end switch
    } // end while
}

// --- 定时器回调 ---
void Scheduler::timerCallback(TIM_HandleTypeDef* htim) {
    if (!initialized || !htim || htim->Instance != this->htim->Instance) {
        return;
    }
    
    // 根据模式执行任务 (遍历当前链表)
    if (mode == SchedulerMode::Obstructed) {
        shutdown();     // 关闭定时器以避免重复触发
        processRequestQueue();  // 处理请求队列
        
        TaskNode* current = listHead;
        while (current != nullptr) {
            current->info.currentUs += intPeriod; 
            if (current->info.currentUs >= current->info.period) { 
                current->info.currentUs = 0; 
                if (current->info.function) { 
                    current->info.function(); 
                }
            }
            current = current->next; // 移动到下一个节点
        }
        
        start(); 
    }
    else { // SchedulerMode::Independent
        start(); 
        processRequestQueue(); // 处理请求队列
        
        TaskNode* current = listHead;
        while (current != nullptr) {
             current->info.currentUs += intPeriod; 
            if (current->info.currentUs >= current->info.period) { 
                current->info.currentUs = 0; 
                if (current->info.function) { 
                    current->info.function(); 
                }
            }
            current = current->next; // 移动到下一个节点
        }
    }
}

// --- 其他方法 (setMode, getMode, setPeriod, getPeriod, shutdown, start) ---
// ... existing implementations for setMode, getMode, setPeriod, getPeriod, shutdown, start ...
// (No changes needed in these methods for the linked list redesign)
void Scheduler::setMode(Scheduler::SchedulerMode mode) { this->mode = mode; }
Scheduler::SchedulerMode Scheduler::getMode() const { return mode; }
void Scheduler::setPeriod(uint32_t period_us) {
    if (period_us == 0) { Error_Handler(); return; }
    this->intPeriod = period_us;
    init(); 
}
uint32_t Scheduler::getPeriod() const { return intPeriod; }
void Scheduler::shutdown() { if (htim) { TimDrv_Shutdown(htim); } }
void Scheduler::start() { if (htim) { TimDrv_Shutdown(htim); TimDrv_Start(htim); } }

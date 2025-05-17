#pragma once

/**
 * @file scheduler.h
 * @brief 基于定时器中断回调周期执行优先级高于os且需要精确频率触发的任务
 * @details 中断中不能执行耗时，或内存分配等操作。
 *
 * 功能特性:
 * - 使用指定的硬件定时器 (TIM) 作为调度基准时钟。
 * - 支持两种运行模式：独立模式 (Independent) 和阻塞模式 (Obstructed)。
 * - 使用单向链表管理活动任务。
 * - 使用两个消息队列进行异步操作：
 *     - 请求队列 (Queue A): 用于接收 addTask, removeTask, clearAllTasks 的请求。
 *     - 清理队列 (Queue B): 用于暂存待释放内存的任务节点指针。
 * - 任务节点 (TaskNode) 在调用 addTask 时动态分配，包含 TaskInfo 和链表指针。
 * - 任务节点的释放 (delete) 在 addTask 或 removeTask 方法开始时，通过处理清理队列完成。
 * - 任务的链表操作 (添加/移除) 在定时器中断回调中处理请求队列时完成。
 * - 支持存储任意可调用对象作为任务。
 * - 提供唯一的 TaskId 用于任务管理。
 *
 * 使用说明:
 * 1. 创建 Scheduler 实例，传入 TIM 句柄和基础中断周期 (微秒)。
 * 2. 调用 init() 方法进行初始化 (必须在 OS 内核启动后调用)。
 * 3. 使用 addTask() 添加任务。
 * 4. 使用 removeTask() 异步移除任务。
 * 5. 在对应的 TIM 中断回调函数中调用 timerCallback() 方法。
 * 6. 确保链接了 utils::allocator 的实现，并且 FreeRTOS 内存管理已配置。
 */

#include "main.h"
#include "tim.h"
#include "tim_drv.h"
#include "cmsis_os.h"
#include <functional>
#include <memory>
#include "allocator.h" // 用于 TaskNode 动态分配
#include <new>       // For std::nothrow
#include <cstdint>   // For uintptr_t

class Scheduler {
public:
    // --- 公共类型定义 ---
    
    /** @brief 调度器运行模式 */
    enum class SchedulerMode {
        Independent = 0, ///< 时钟独立模式
        Obstructed,      ///< 时钟阻塞模式
    };

    /** @brief 任务函数类型 */
    using TaskFunction = std::function<void()>; 

    /** @brief 任务唯一标识符类型 */
    using TaskId = uint32_t;

    /** @brief 任务信息结构体 (包含在 TaskNode 中) */
    struct TaskInfo {
        TaskId id;              ///< 任务的唯一 ID
        TaskFunction function;  ///< 任务函数对象
        uint64_t period;        ///< 任务执行周期 (微秒)
        uint64_t currentUs;     ///< 当前已累积的微秒数
    };

private:
    // --- 私有类型定义 ---

    /** @brief 任务链表节点结构体 */
    struct TaskNode {
        TaskInfo info;      ///< 任务信息
        TaskNode* next;     ///< 指向下一个节点的指针
    };

    /** @brief 请求队列的操作类型 */
    enum class RequestOpType : uint8_t { // Use smaller type
        ADD_TASK,
        REMOVE_TASK,
        CLEAR_ALL_TASKS
    };

    /** @brief 请求队列 (Queue A) 的消息结构体 */
    struct RequestMsg {
        RequestOpType operation; ///< 请求的操作类型
        uint32_t data;          ///< 操作关联的数据 (ADD: TaskNode*, REMOVE: TaskId, CLEAR: 0)
    };

    /** @brief 清理队列 (Queue B) 的消息结构体 (仅包含指针) */
    struct CleanupMsg {
        TaskNode* nodePtr;       ///< 待清理的 TaskNode 指针
    };

public:
    // --- 公共接口 ---

    /** @brief 构造函数 */
    Scheduler(TIM_HandleTypeDef* htim, uint32_t intPeriod);
    
    /** @brief 析构函数 */
    ~Scheduler();

    // 禁止拷贝和赋值
    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;
    Scheduler(Scheduler&&) = delete;
    Scheduler& operator=(Scheduler&&) = delete;

    /** @brief 初始化调度器 */
    void init();

    /** @brief 设置调度器运行模式 */
    void setMode(SchedulerMode mode);
    
    /** @brief 获取当前调度器运行模式 */
    SchedulerMode getMode() const;
    
    /** @brief 设置调度器的基础中断周期 */
    void setPeriod(uint32_t period_us);
    
    /** @brief 获取当前调度器的基础中断周期 */
    uint32_t getPeriod() const;
    
    /**
     * @brief 添加一个新任务。
     *        先处理清理队列，然后分配 TaskNode，最后发送添加请求到请求队列。
     * @tparam Callable 可调用对象的类型。
     * @param task 要调度的可调用对象。
     * @param period 任务的执行周期 (微秒)。0 表示使用调度器基础周期。
     * @return 成功则返回任务 TaskId，失败返回 0。
     */
    template<typename Callable>
    TaskId addTask(Callable&& task, uint64_t period = 0) {
        processCleanupQueue(); // 处理待清理节点
        
        TaskId id = generateTaskId(); 
        if (id == 0) return 0;

        // 分配 TaskNode
        TaskNode* newNode = new(std::nothrow) TaskNode();
        if (!newNode) {
            return 0; // 内存分配失败
        }

        // 填充 TaskNode
        newNode->info.id = id;
        newNode->info.function = std::forward<Callable>(task);
        newNode->info.period = (period == 0) ? intPeriod : period;
        newNode->info.currentUs = 0; 
        newNode->next = nullptr; // 初始化 next 指针
        
        // 创建添加请求消息
        RequestMsg reqMsg;
        reqMsg.operation = RequestOpType::ADD_TASK;
        reqMsg.data = reinterpret_cast<uintptr_t>(newNode); // 传递节点指针
        
        // 发送消息到请求队列
        if (osMessageQueuePut(requestQueue, &reqMsg, 0, 10) != osOK) {
            delete newNode; // 发送失败，清理已分配的节点
            return 0; 
        }
        return id; // 返回任务 ID
    }
    
    /**
     * @brief 添加一个新任务 (带参数版本)。
     *        先处理清理队列，然后分配 TaskNode，使用 std::bind 绑定参数，最后发送添加请求。
     * @tparam Callable 可调用对象的类型。
     * @tparam Args 参数类型。
     * @param task 要调度的可调用对象。
     * @param period 任务的执行周期 (微秒)。0 表示使用调度器基础周期。
     * @param args 传递给可调用对象的参数。
     * @return 成功则返回任务 TaskId，失败返回 0。
     */
    template<typename Callable, typename... Args>
    TaskId addTask(Callable&& task, uint64_t period, Args&&... args) {
        processCleanupQueue(); // 处理待清理节点
        
        TaskId id = generateTaskId(); 
        if (id == 0) return 0;

        // 分配 TaskNode
        TaskNode* newNode = new(std::nothrow) TaskNode();
        if (!newNode) {
            return 0; // 内存分配失败
        }

        // 填充 TaskNode
        newNode->info.id = id;
        newNode->info.function = std::bind(std::forward<Callable>(task), std::forward<Args>(args)...);
        newNode->info.period = (period == 0) ? intPeriod : period;
        newNode->info.currentUs = 0;
        newNode->next = nullptr;
        
        // 创建添加请求消息
        RequestMsg reqMsg;
        reqMsg.operation = RequestOpType::ADD_TASK;
        reqMsg.data = reinterpret_cast<uintptr_t>(newNode);
        
        // 发送消息到请求队列
        if (osMessageQueuePut(requestQueue, &reqMsg, 0, 10) != osOK) {
            delete newNode; // 发送失败，清理节点
            return 0; 
        }
        
        return id; // 返回任务 ID
    }
    
    /**
     * @brief 异步请求移除一个任务。
     *        先处理清理队列，然后发送移除请求到请求队列。
     * @param taskId 要移除的任务的 ID。
     * @return true 如果移除请求成功发送，false 如果发送失败。
     */
    bool removeTask(TaskId taskId); 
    
    /**
     * @brief 异步请求清空所有任务。
     *        发送清空请求到请求队列。实际清理在中断中完成。
     * @return true 如果清空请求成功发送，false 如果发送失败。
     */
    bool clearAllTasks();
    
    /**
     * @brief 定时器中断回调函数。
     *        处理请求队列，然后遍历链表执行到期任务。
     * @param htim 触发中断的 TIM_HandleTypeDef 结构体指针。
     */
    void timerCallback(TIM_HandleTypeDef* htim);
    
private:
    // --- 私有成员变量 ---
    TIM_HandleTypeDef* htim;             ///< 定时器句柄
    SchedulerMode mode;                  ///< 运行模式
    uint32_t intPeriod;                  ///< 中断周期 (us)
    bool initialized;                    ///< 初始化标志
    
    TaskNode* listHead;                  ///< 任务链表头指针
    uint32_t currentTaskCount;///< 当前活动任务数量
    TaskId nextTaskId;                   ///< 用于生成下一个任务 ID 的计数器
    
    osMessageQueueId_t requestQueue;     ///< 请求队列 (Queue A) 句柄
    osMessageQueueId_t cleanupQueue;     ///< 清理队列 (Queue B) 句柄
    

    
    // --- 私有成员函数 ---

    /** @brief 处理清理队列中的所有待删除节点。 */
    void processCleanupQueue();

    /** @brief 生成唯一的任务 ID (需要考虑链表中的 ID)。 */
    TaskId generateTaskId();
    
    /** @brief 处理请求队列中的消息 (在 timerCallback 中调用)。 */
    void processRequestQueue();
    
    /** @brief 关闭定时器。 */
    void shutdown();
    
    /** @brief 启动定时器。 */
    void start();
};


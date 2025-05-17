#ifndef __MODULE_WATCHDOG_H__
#define __MODULE_WATCHDOG_H__

#include "time_timestamp.h"
#include <functional>
#include "cmsis_os.h" // 添加CMSIS-RTOS头文件


// 前向声明
class WatchdogManager;

class Watchdog {
public:
    /**
     * @brief 构造函数，仅设置参数，不进行初始化或注册
     * @param timeout_ms 超时时间（毫秒）
     * @param callback 超时时触发的回调函数
     */
    Watchdog(uint32_t timeout_ms, std::function<void()> callback);
    
    /**
     * @brief 析构函数，如果看门狗已启动则会自动停止
     */
    ~Watchdog();

    // 禁止拷贝
    Watchdog(const Watchdog&) = delete;
    Watchdog& operator=(const Watchdog&) = delete;
    
    // 允许移动
    Watchdog(Watchdog&& other) noexcept;
    Watchdog& operator=(Watchdog&& other) noexcept;
    
    /**
     * @brief 启动看门狗并将其注册到管理器
     * @param manager 看门狗管理器引用
     * @return 是否成功启动
     */
    bool start(WatchdogManager& manager);
    
    /**
     * @brief 停止看门狗并从管理器中注销
     */
    void stop();
    
    /**
     * @brief "喂狗"操作，重置看门狗计时器
     */
    void feed();
    
    /**
     * @brief 检查看门狗是否处于活动状态
     * @return 当前是否活动
     */
    bool isActive() const;

private:
    friend class WatchdogManager;
    
    uint32_t timeout_ms_;               // 超时时间（毫秒）
    timestamp_t timeout_ticks_;         // 超时时间（系统时钟节拍）
    timestamp_t last_feed_time_;        // 上次"喂狗"时间
    std::function<void()> callback_;    // 超时回调函数
    bool active_;                       // 是否活动
    Watchdog* next_;                    // 链表下一节点
    WatchdogManager* manager_;          // 关联的管理器
};

class WatchdogManager {
public:
    /**
     * @brief 创建看门狗管理器
     */
    WatchdogManager();
    
    /**
     * @brief 销毁看门狗管理器，自动停止所有看门狗
     */
    ~WatchdogManager();
    
    // 禁止拷贝和移动
    WatchdogManager(const WatchdogManager&) = delete;
    WatchdogManager& operator=(const WatchdogManager&) = delete;
    WatchdogManager(WatchdogManager&&) = delete;
    WatchdogManager& operator=(WatchdogManager&&) = delete;

    /**
     * @brief 初始化看门狗管理系统
     * @return 是否成功初始化
     */
    bool init();
    
    /**
     * @brief 反初始化看门狗管理系统
     */
    void deinit();
    
    /**
     * @brief 注册看门狗到管理器
     * @param watchdog 要注册的看门狗
     * @return 是否注册成功
     */
    bool registerWatchdog(Watchdog* watchdog);
    
    /**
     * @brief 从管理器注销看门狗
     * @param watchdog 要注销的看门狗
     */
    void unregisterWatchdog(Watchdog* watchdog);

    /**
     * @brief 检查管理器是否已初始化
     * @return 是否已初始化
     */
    bool isInitialized() const;

private:
    Watchdog* head_;                  // 看门狗链表头指针
    osMutexId_t mutex_;               // 保护链表的互斥锁 (CMSIS-RTOS V2)
    osThreadId_t task_handle_;        // 管理线程句柄 (CMSIS-RTOS V2)
    bool initialized_;   // 是否已初始化
    
    // 管理线程函数
    static void managerTask(void* param);
};

#endif // __MODULE_WATCHDOG_H__

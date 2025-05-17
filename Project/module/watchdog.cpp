#include "watchdog.h"
#include "time_utils.h"
#include "cmsis_os.h"

// Watchdog类实现
Watchdog::Watchdog(uint32_t timeout_ms, std::function<void()> callback)
    : timeout_ms_(timeout_ms),
      timeout_ticks_(0),  // 这个值将在start时计算
      last_feed_time_(0), // 这个值将在start时初始化
      callback_(callback),
      active_(false),
      next_(nullptr),
      manager_(nullptr) {
}

Watchdog::~Watchdog() {
    stop();
}

Watchdog::Watchdog(Watchdog&& other) noexcept
    : timeout_ms_(other.timeout_ms_),
      timeout_ticks_(other.timeout_ticks_),
      last_feed_time_(other.last_feed_time_),
      callback_(std::move(other.callback_)),
      active_(other.active_),
      next_(other.next_),
      manager_(other.manager_) {
    
    other.active_ = false;
    other.next_ = nullptr;
    other.manager_ = nullptr;
    
    // 如果处于活动状态，需要更新管理器中的指针
    if (active_ && manager_) {
        // 为简单起见，先停止再重新启动
        stop();
        start(*manager_);
    }
}

Watchdog& Watchdog::operator=(Watchdog&& other) noexcept {
    if (this != &other) {
        stop(); // 先停止当前看门狗
        
        timeout_ms_ = other.timeout_ms_;
        timeout_ticks_ = other.timeout_ticks_;
        last_feed_time_ = other.last_feed_time_;
        callback_ = std::move(other.callback_);
        active_ = other.active_;
        next_ = other.next_;
        manager_ = other.manager_;
        
        other.active_ = false;
        other.next_ = nullptr;
        other.manager_ = nullptr;
        
        // 如果移动源处于活动状态，则启动当前看门狗
        if (active_ && manager_) {
            stop();
            start(*manager_);
        }
    }
    return *this;
}

bool Watchdog::start(WatchdogManager& manager) {
    if (active_) {
        return true; // 已经启动，无需重复操作
    }
    
    if (!manager.isInitialized()) {
        return false; // 管理器未初始化
    }
    
    // 计算超时时间（系统时钟节拍）
    timeout_ticks_ = utils::time::TimeStamp::fromMilliseconds(timeout_ms_);
    
    // 记录当前时间作为最后一次喂狗时间
    last_feed_time_ = utils::time::TimeStamp::now();
    
    // 设置为活动状态
    active_ = true;
    manager_ = &manager;
    
    // 注册到管理器
    return manager_->registerWatchdog(this);
}

void Watchdog::stop() {
    if (!active_) {
        return; // 已经停止，无需操作
    }
    
    // 如果已注册到管理器，则从管理器中注销
    if (manager_) {
        manager_->unregisterWatchdog(this);
        manager_ = nullptr;
    }
    
    active_ = false;
}

void Watchdog::feed() {
    if (active_) {
        last_feed_time_ = utils::time::TimeStamp::now();
    }
}

bool Watchdog::isActive() const {
    return active_;
}

// WatchdogManager类实现
WatchdogManager::WatchdogManager()
    : head_(nullptr),
      mutex_(nullptr),
      task_handle_(nullptr),
      initialized_(false) {
}

WatchdogManager::~WatchdogManager() {
    deinit();
}

bool WatchdogManager::init() {
    if (initialized_) {
        return true; // 已经初始化
    }
    
    // 创建互斥锁 (CMSIS-RTOS V2)
    osMutexAttr_t mutex_attr = {
        .name = "WatchdogMutex",
        .attr_bits = osMutexRecursive,
        .cb_mem = nullptr,
        .cb_size = 0
    };
    mutex_ = osMutexNew(&mutex_attr);
    if (!mutex_) {
        return false; // 创建互斥锁失败
    }
    
    // 创建管理任务 (CMSIS-RTOS V2)
    osThreadAttr_t task_attr = {
        .name = "WatchdogManager",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = osPriorityAboveNormal,
        .tz_module = 0,
        .reserved = 0
    };
    
    task_handle_ = osThreadNew(managerTask, this, &task_attr);
    if (task_handle_ == nullptr) {
        osMutexDelete(mutex_);
        mutex_ = nullptr;
        return false;
    }
    
    initialized_ = true;
    return true;
}

void WatchdogManager::deinit() {
    if (!initialized_) {
        return; // 未初始化，无需操作
    }
    
    // 停止管理任务 (CMSIS-RTOS V2)
    if (task_handle_) {
        osThreadTerminate(task_handle_);
        task_handle_ = nullptr;
    }
    
    // 获取互斥锁 (CMSIS-RTOS V2)
    if (mutex_) {
        osMutexAcquire(mutex_, osWaitForever);
    }
    
    // 清空所有看门狗（将它们设为非活动状态）
    Watchdog* current = head_;
    while (current) {
        current->active_ = false;
        current->manager_ = nullptr;
        current = current->next_;
    }
    head_ = nullptr;
    
    // 释放并删除互斥锁 (CMSIS-RTOS V2)
    if (mutex_) {
        osMutexRelease(mutex_);
        osMutexDelete(mutex_);
        mutex_ = nullptr;
    }
    
    initialized_ = false;
}

bool WatchdogManager::registerWatchdog(Watchdog* watchdog) {
    if (!initialized_ || !watchdog) {
        return false;
    }
    
    // 获取互斥锁 (CMSIS-RTOS V2)
    if (osMutexAcquire(mutex_, osWaitForever) != osOK) {
        return false;
    }
    
    // 将看门狗添加到链表头部
    watchdog->next_ = head_;
    head_ = watchdog;
    
    // 释放互斥锁 (CMSIS-RTOS V2)
    osMutexRelease(mutex_);
    return true;
}

void WatchdogManager::unregisterWatchdog(Watchdog* watchdog) {
    if (!initialized_ || !watchdog) {
        return;
    }
    
    // 获取互斥锁 (CMSIS-RTOS V2)
    if (osMutexAcquire(mutex_, osWaitForever) != osOK) {
        return;
    }
    
    // 从链表中移除看门狗
    if (head_ == watchdog) {
        // 如果是头节点
        head_ = watchdog->next_;
    } else {
        // 在链表中查找
        Watchdog* prev = head_;
        while (prev && prev->next_ != watchdog) {
            prev = prev->next_;
        }
        
        if (prev && prev->next_ == watchdog) {
            prev->next_ = watchdog->next_;
        }
    }
    
    // 清除看门狗中的链表指针和管理器指针
    watchdog->next_ = nullptr;
    watchdog->manager_ = nullptr;
    watchdog->active_ = false;
    
    // 释放互斥锁 (CMSIS-RTOS V2)
    osMutexRelease(mutex_);
}

bool WatchdogManager::isInitialized() const {
    return initialized_;
}

void WatchdogManager::managerTask(void* param) {
    WatchdogManager* manager = static_cast<WatchdogManager*>(param);
    uint32_t last_wake_time = osKernelGetTickCount();
    const uint32_t check_interval = 2; // 每2ms检查一次
    
    while (true) {
        // 延时到下一个检查点 (CMSIS-RTOS V2)
        osDelayUntil(last_wake_time + check_interval);
        last_wake_time = osKernelGetTickCount();
        
        // 获取互斥锁 (CMSIS-RTOS V2)
        if (osMutexAcquire(manager->mutex_, 10) != osOK) {
            continue; // 如果无法获取互斥锁，跳过本次检查
        }
        
        // 遍历所有看门狗
        Watchdog* current = manager->head_;
        timestamp_t now = utils::time::TimeStamp::now();
        
        while (current) {
            if (current->active_) {
                // 计算经过的时间
                timestamp_t elapsed = utils::time::TimeStamp::diff(now, current->last_feed_time_);
                
                // 检查是否超时
                if (elapsed >= current->timeout_ticks_) {
                    // 记录需要触发回调的看门狗
                    Watchdog* to_handle = current;
                    auto callback = to_handle->callback_;
                    
                    // 从链表中移除
                    if (manager->head_ == to_handle) {
                        manager->head_ = to_handle->next_;
                        current = manager->head_;
                    } else {
                        Watchdog* prev = manager->head_;
                        while (prev && prev->next_ != to_handle) {
                            prev = prev->next_;
                        }
                        if (prev) {
                            prev->next_ = to_handle->next_;
                            current = to_handle->next_;
                        } else {
                            current = current->next_;
                        }
                    }
                    
                    // 重置看门狗状态
                    to_handle->active_ = false;
                    to_handle->next_ = nullptr;
                    to_handle->manager_ = nullptr;
                    
                    // 释放互斥锁，调用回调 (CMSIS-RTOS V2)
                    osMutexRelease(manager->mutex_);
                    if (callback) {
                        callback(); // 调用超时回调
                    }
                    
                    // 重新获取互斥锁继续处理 (CMSIS-RTOS V2)
                    if (osMutexAcquire(manager->mutex_, 10) != osOK) {
                        break; // 如果无法重新获取互斥锁，结束本次检查
                    }
                } else {
                    // 继续检查下一个看门狗
                    current = current->next_;
                }
            } else {
                current = current->next_;
            }
        }
        
        // 释放互斥锁 (CMSIS-RTOS V2)
        osMutexRelease(manager->mutex_);
    }
}

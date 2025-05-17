// allocator.h
#ifndef __UTILS_MEMORY_ALLOCATOR_H__
#define __UTILS_MEMORY_ALLOCATOR_H__

#include "cmsis_os.h"
// 内存分配宏定义
#define __utils_malloc(size) pvPortMalloc(size)
#define __utils_free(ptr) vPortFree(ptr)

#ifdef __cplusplus

#include <cstddef>
#include <new>          // std::bad_alloc
#include <utility>      // std::forward

namespace utils {

template<typename T>
class allocator
{
public:
    using value_type      = T;
    using pointer         = T*;
    using const_pointer   = const T*;
    using void_pointer    = void*;
    using const_void_ptr  = const void*;
    using difference_type = std::ptrdiff_t;
    using size_type       = std::size_t;

    // ① 在C++11中，使用propagate_on_xxx替代is_always_equal
    using propagate_on_container_move_assignment = std::true_type;
    using propagate_on_container_copy_assignment = std::false_type;
    using propagate_on_container_swap = std::false_type;

    // ② 必须要有的默认/拷贝构造
    allocator() noexcept = default;
    template<class U>
    allocator(const allocator<U>&) noexcept {}

    // ③ allocate / deallocate
    pointer allocate(size_type n)
    {
        if (n == 0) return nullptr;
        if (void* p = __utils_malloc(n * sizeof(T)))
            return static_cast<pointer>(p);

        return nullptr; // 这里可以抛出异常 std::bad_alloc
    }

    void deallocate(pointer p, size_type /*unused*/) noexcept
    {
        __utils_free(static_cast<void*>(p));
    }

    // ④ 旧式 STL 兼容：rebind
    template<class U>
    struct rebind { using other = allocator<U>; };
};

// ⑤ 相等性：任何两个 allocator 都等价
template<class T, class U>
constexpr bool operator==(const allocator<T>&, const allocator<U>&) noexcept { return true; }
template<class T, class U>
constexpr bool operator!=(const allocator<T>&, const allocator<U>&) noexcept { return false; }

} // namespace utils

#endif // __cplusplus

#endif // __UTILS_MEMORY_ALLOCATOR_H__
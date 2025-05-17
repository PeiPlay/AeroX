#include "allocator.h"
#include "cmsis_os.h" // FreeRTOS 头文件
#include <cstddef>     // std::size_t
#include <new>         // std::bad_alloc, std::nothrow_t, std::align_val_t

/*------------------------ 基本版本 ------------------------*/
void* operator new(std::size_t sz)
{
    if (void* p = __utils_malloc(sz)) return p;
    throw std::bad_alloc{};
}
void  operator delete(void* p) noexcept
{
    __utils_free(p);
}

void* operator new[](std::size_t sz)                { return ::operator new(sz); }
void  operator delete[](void* p)        noexcept    { ::operator delete(p); }

/*---------------------- nothrow 版本 ----------------------*/
void* operator new (std::size_t sz, const std::nothrow_t&) noexcept
{ return __utils_malloc(sz); }

void* operator new[](std::size_t sz, const std::nothrow_t&) noexcept
{ return __utils_malloc(sz); }

void  operator delete (void* p, const std::nothrow_t&) noexcept
{ __utils_free(p); }

void  operator delete[](void* p, const std::nothrow_t&) noexcept
{ __utils_free(p); }

/*------------------- sized-deallocation 兼容 -------------------*/
#if __cpp_sized_deallocation
void operator delete(void* p, std::size_t)  noexcept { __utils_free(p); }
void operator delete[](void* p, std::size_t) noexcept { __utils_free(p); }
#endif

/*--------------------- aligned_new 兼容 ---------------------*/
#if __cpp_aligned_new   // GCC / Clang 均已支持
void* operator new (std::size_t sz, std::align_val_t)           { return ::operator new(sz); }
void* operator new[](std::size_t sz, std::align_val_t)          { return ::operator new(sz); }
void  operator delete (void* p,  std::align_val_t)  noexcept    { ::operator delete(p); }
void  operator delete[](void* p, std::align_val_t)  noexcept    { ::operator delete(p); }
#endif
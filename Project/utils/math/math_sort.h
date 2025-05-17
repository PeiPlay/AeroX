#ifndef __MATH_SORT_H__
#define __MATH_SORT_H__

#include <stdint.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

// 基本排序算法
void math_sort_bubble(float *arr, uint32_t size);
void math_sort_insertion(float *arr, uint32_t size);
void math_sort_selection(float *arr, uint32_t size);
void math_sort_quick(float *arr, uint32_t size);
void math_sort_heap(float *arr, uint32_t size);
void math_sort_merge(float *arr, uint32_t size);

// 针对整数的排序算法
void math_sort_counting(int32_t *arr, uint32_t size, int32_t min_val, int32_t max_val);
void math_sort_radix(int32_t *arr, uint32_t size);

// 辅助函数
void math_sort_reverse(float *arr, uint32_t size);
float math_sort_median(float *arr, uint32_t size);
float math_sort_kth_element(float *arr, uint32_t size, uint32_t k);

// 二分查找
int32_t math_sort_binary_search(const float *arr, uint32_t size, float value);
int32_t math_sort_binary_search_left(const float *arr, uint32_t size, float value);
int32_t math_sort_binary_search_right(const float *arr, uint32_t size, float value);

// 内部API，用于获取内部使用的临时缓冲区
void* math_sort_get_buffer(uint32_t size_bytes);
void math_sort_release_buffer(void* buffer);

#ifdef __cplusplus
}

namespace utils {
namespace math {

// C++ 类封装
class Sorter {
private:
    // 堆排序的辅助函数
    template<typename T>
    static void heapify(T* arr, uint32_t size, uint32_t i);

public:
    // 排序模板方法
    template<typename T>
    static void quickSort(T* arr, uint32_t size);
    
    template<typename T>
    static void insertionSort(T* arr, uint32_t size);
    
    template<typename T>
    static void heapSort(T* arr, uint32_t size);
    
    template<typename T>
    static void mergeSort(T* arr, uint32_t size);
    
    // 查找方法
    template<typename T>
    static int32_t binarySearch(const T* arr, uint32_t size, const T& value);
    
    template<typename T>
    static T median(T* arr, uint32_t size);
    
    template<typename T>
    static T kthElement(T* arr, uint32_t size, uint32_t k);
};

} // namespace math
} // namespace utils

#endif /* __cplusplus */

#endif /* __MATH_SORT_H__ */

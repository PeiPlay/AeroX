#include "math_sort.h"
#include <string.h>
#include "math_utils.h"
#include <cmsis_os.h>

// 内存管理优化 - 使用math_utils统一内存分配接口
void* math_sort_get_buffer(uint32_t size_bytes) {
    return __math_utils_malloc(size_bytes);
}

void math_sort_release_buffer(void* buffer) {
    if (buffer != NULL) {
        __math_utils_free(buffer);
    }
}

// 交换两个元素
static inline void swap_float(float* a, float* b) {
    float temp = *a;
    *a = *b;
    *b = temp;
}

static inline void swap_int32(int32_t* a, int32_t* b) {
    int32_t temp = *a;
    *a = *b;
    *b = temp;
}

// 冒泡排序
void math_sort_bubble(float *arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    uint32_t i, j;
    uint8_t swapped;
    
    for (i = 0; i < size - 1; i++) {
        swapped = 0;
        for (j = 0; j < size - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                swap_float(&arr[j], &arr[j + 1]);
                swapped = 1;
            }
        }
        // 如果内层循环没有进行交换，说明数组已经排序完成
        if (swapped == 0) break;
    }
}

// 插入排序
void math_sort_insertion(float *arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    uint32_t i, j;
    float key;
    
    for (i = 1; i < size; i++) {
        key = arr[i];
        j = i - 1;
        
        while (j < UINT32_MAX && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// 选择排序
void math_sort_selection(float *arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    uint32_t i, j, min_idx;
    
    for (i = 0; i < size - 1; i++) {
        min_idx = i;
        for (j = i + 1; j < size; j++) {
            if (arr[j] < arr[min_idx]) {
                min_idx = j;
            }
        }
        if (min_idx != i) {
            swap_float(&arr[min_idx], &arr[i]);
        }
    }
}

// 快速排序辅助函数 - 分区
static uint32_t partition(float *arr, int32_t low, int32_t high) {
    float pivot = arr[high];
    int32_t i = low - 1;
    int32_t j;
    
    for (j = low; j < high; j++) {
        if (arr[j] <= pivot) {
            i++;
            swap_float(&arr[i], &arr[j]);
        }
    }
    swap_float(&arr[i + 1], &arr[high]);
    return (uint32_t)(i + 1);
}

// 快速排序辅助函数 - 递归
static void quick_sort_recursive(float *arr, int32_t low, int32_t high) {
    if (low < high) {
        uint32_t pi = partition(arr, low, high);
        
        // 小数组使用插入排序提高性能
        if (pi - low < 10) {
            for (int32_t i = low + 1; i <= (int32_t)pi; i++) {
                float key = arr[i];
                int32_t j = i - 1;
                while (j >= low && arr[j] > key) {
                    arr[j + 1] = arr[j];
                    j--;
                }
                arr[j + 1] = key;
            }
        } else {
            quick_sort_recursive(arr, low, pi - 1);
        }
        
        if (high - pi < 10) {
            for (int32_t i = pi + 1; i <= high; i++) {
                float key = arr[i];
                int32_t j = i - 1;
                while (j >= (int32_t)pi && arr[j] > key) {
                    arr[j + 1] = arr[j];
                    j--;
                }
                arr[j + 1] = key;
            }
        } else {
            quick_sort_recursive(arr, pi + 1, high);
        }
    }
}

// 快速排序
void math_sort_quick(float *arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    // 对小数组使用插入排序
    if (size <= 10) {
        math_sort_insertion(arr, size);
        return;
    }
    
    quick_sort_recursive(arr, 0, size - 1);
}

// 堆排序辅助函数 - 堆化
static void heapify(float *arr, uint32_t size, uint32_t i) {
    uint32_t largest = i;
    uint32_t left = 2 * i + 1;
    uint32_t right = 2 * i + 2;
    
    if (left < size && arr[left] > arr[largest])
        largest = left;
    
    if (right < size && arr[right] > arr[largest])
        largest = right;
    
    if (largest != i) {
        swap_float(&arr[i], &arr[largest]);
        heapify(arr, size, largest);
    }
}

// 堆排序
void math_sort_heap(float *arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    uint32_t i;
    
    // 构建最大堆
    for (i = size / 2 - 1; i < UINT32_MAX; i--) {
        heapify(arr, size, i);
    }
    
    // 从堆顶提取元素
    for (i = size - 1; i > 0; i--) {
        swap_float(&arr[0], &arr[i]);
        heapify(arr, i, 0);
    }
}

// 归并排序辅助函数 - 合并
static void merge(float *arr, uint32_t left, uint32_t mid, uint32_t right, float *temp) {
    uint32_t i = left;
    uint32_t j = mid + 1;
    uint32_t k = 0;
    
    while (i <= mid && j <= right) {
        if (arr[i] <= arr[j]) {
            temp[k++] = arr[i++];
        } else {
            temp[k++] = arr[j++];
        }
    }
    
    while (i <= mid) {
        temp[k++] = arr[i++];
    }
    
    while (j <= right) {
        temp[k++] = arr[j++];
    }
    
    memcpy(arr + left, temp, k * sizeof(float));
}

// 归并排序辅助函数 - 递归
static void merge_sort_recursive(float *arr, uint32_t left, uint32_t right, float *temp) {
    if (left >= right) return;
    
    uint32_t mid = left + (right - left) / 2;
    
    merge_sort_recursive(arr, left, mid, temp);
    merge_sort_recursive(arr, mid + 1, right, temp);
    merge(arr, left, mid, right, temp);
}

// 归并排序
void math_sort_merge(float *arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    // 小数组使用插入排序
    if (size <= 10) {
        math_sort_insertion(arr, size);
        return;
    }
    
    float *temp = (float*)math_sort_get_buffer(size * sizeof(float));
    if (!temp) return; // 内存分配失败
    
    merge_sort_recursive(arr, 0, size - 1, temp);
    
    math_sort_release_buffer(temp);
}

// 计数排序
void math_sort_counting(int32_t *arr, uint32_t size, int32_t min_val, int32_t max_val) {
    if (!arr || size <= 1 || min_val > max_val) return;
    
    uint32_t range = max_val - min_val + 1;
    
    // 分配计数数组
    uint32_t *count = (uint32_t*)math_sort_get_buffer(range * sizeof(uint32_t));
    if (!count) return; // 内存分配失败
    
    memset(count, 0, range * sizeof(uint32_t));
    
    // 计数
    for (uint32_t i = 0; i < size; i++) {
        count[arr[i] - min_val]++;
    }
    
    // 重建数组
    uint32_t index = 0;
    for (uint32_t i = 0; i < range; i++) {
        while (count[i] > 0) {
            arr[index++] = i + min_val;
            count[i]--;
        }
    }
    
    math_sort_release_buffer(count);
}

// 基数排序辅助函数 - 按特定位排序
static void counting_sort_by_digit(int32_t *arr, uint32_t size, int32_t exp) {
    int32_t *output = (int32_t*)math_sort_get_buffer(size * sizeof(int32_t));
    if (!output) return; // 内存分配失败
    
    uint32_t count[10] = {0};
    
    // 计算当前位的频率
    for (uint32_t i = 0; i < size; i++) {
        count[(arr[i] / exp) % 10]++;
    }
    
    // 累加频率
    for (uint32_t i = 1; i < 10; i++) {
        count[i] += count[i - 1];
    }
    
    // 构建输出数组
    for (int32_t i = size - 1; i >= 0; i--) {
        output[count[(arr[i] / exp) % 10] - 1] = arr[i];
        count[(arr[i] / exp) % 10]--;
    }
    
    // 拷贝回原数组
    memcpy(arr, output, size * sizeof(int32_t));
    
    math_sort_release_buffer(output);
}

// 基数排序
void math_sort_radix(int32_t *arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    // 找到最大值以确定位数
    int32_t max_val = arr[0];
    for (uint32_t i = 1; i < size; i++) {
        if (arr[i] > max_val) {
            max_val = arr[i];
        }
    }
    
    // 对负数的处理
    int32_t min_val = arr[0];
    for (uint32_t i = 1; i < size; i++) {
        if (arr[i] < min_val) {
            min_val = arr[i];
        }
    }
    
    // 如果有负数，将所有数值调整为非负
    if (min_val < 0) {
        for (uint32_t i = 0; i < size; i++) {
            arr[i] -= min_val; // 偏移使所有值非负
        }
        max_val -= min_val; // 相应调整最大值
    }
    
    // 按位排序
    for (int32_t exp = 1; max_val / exp > 0; exp *= 10) {
        counting_sort_by_digit(arr, size, exp);
    }
    
    // 如果有负数，恢复原始值
    if (min_val < 0) {
        for (uint32_t i = 0; i < size; i++) {
            arr[i] += min_val;
        }
    }
}

// 数组反转
void math_sort_reverse(float *arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    for (uint32_t i = 0; i < size / 2; i++) {
        swap_float(&arr[i], &arr[size - 1 - i]);
    }
}

// 计算中位数 - 不改变原数组
float math_sort_median(float *arr, uint32_t size) {
    if (!arr || size == 0) return 0.0f;
    if (size == 1) return arr[0];
    
    // 创建临时数组拷贝
    float *temp = (float*)math_sort_get_buffer(size * sizeof(float));
    if (!temp) return arr[0]; // 内存分配失败，返回第一个元素
    
    // 使用arm_copy_f32优化性能
    arm_copy_f32(arr, temp, size);
    
    // 使用快速选择算法找中位数
    uint32_t mid_idx = size / 2;
    float result;
    
    if (size % 2 == 0) {
        // 偶数个元素，找第 size/2 和 size/2-1 个元素的平均值
        float val1 = math_sort_kth_element(temp, size, mid_idx);
        float val2 = math_sort_kth_element(temp, size, mid_idx - 1);
        result = (val1 + val2) / 2.0f;
    } else {
        // 奇数个元素，直接找第 size/2 个元素
        result = math_sort_kth_element(temp, size, mid_idx);
    }
    
    math_sort_release_buffer(temp);
    return result;
}

// 快速选择算法 - 找第k小的元素
float math_sort_kth_element(float *arr, uint32_t size, uint32_t k) {
    if (!arr || size == 0 || k >= size) return 0.0f;
    
    // 用快速排序的分区思想
    int32_t left = 0;
    int32_t right = size - 1;
    
    while (left <= right) {
        uint32_t pivot_idx = partition(arr, left, right);
        
        if (pivot_idx == k) {
            return arr[pivot_idx];
        } else if (pivot_idx > k) {
            right = pivot_idx - 1;
        } else {
            left = pivot_idx + 1;
        }
    }
    
    return arr[0]; // 正常情况不会到达这里
}

// 二分查找
int32_t math_sort_binary_search(const float *arr, uint32_t size, float value) {
    if (!arr || size == 0) return -1;
    
    int32_t left = 0;
    int32_t right = size - 1;
    
    while (left <= right) {
        int32_t mid = left + (right - left) / 2;
        
        if (arr[mid] == value) {
            return mid;
        } else if (arr[mid] < value) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    
    return -1; // 未找到
}

// 二分查找 - 查找第一个大于等于value的位置
int32_t math_sort_binary_search_left(const float *arr, uint32_t size, float value) {
    if (!arr || size == 0) return -1;
    
    int32_t left = 0;
    int32_t right = size - 1;
    int32_t result = -1;
    
    while (left <= right) {
        int32_t mid = left + (right - left) / 2;
        
        if (arr[mid] >= value) {
            result = mid;
            right = mid - 1;
        } else {
            left = mid + 1;
        }
    }
    
    return result;
}

// 二分查找 - 查找最后一个小于等于value的位置
int32_t math_sort_binary_search_right(const float *arr, uint32_t size, float value) {
    if (!arr || size == 0) return -1;
    
    int32_t left = 0;
    int32_t right = size - 1;
    int32_t result = -1;
    
    while (left <= right) {
        int32_t mid = left + (right - left) / 2;
        
        if (arr[mid] <= value) {
            result = mid;
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    
    return result;
}

#ifdef __cplusplus
namespace MathUtils {

// 下面是C++模板方法实现
template<typename T>
void Sorter::quickSort(T* arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    // 对小数组使用插入排序
    if (size <= 10) {
        insertionSort(arr, size);
        return;
    }
    
    // 快排的内部实现
    class QuickSortImpl {
    public:
        static void sort(T* arr, int32_t low, int32_t high) {
            if (low < high) {
                int32_t pi = partition(arr, low, high);
                
                // 小数组使用插入排序提高性能
                if (pi - low < 10) {
                    insertionSort(arr, low, pi);
                } else {
                    sort(arr, low, pi - 1);
                }
                
                if (high - pi < 10) {
                    insertionSort(arr, pi + 1, high);
                } else {
                    sort(arr, pi + 1, high);
                }
            }
        }
        
    private:
        static int32_t partition(T* arr, int32_t low, int32_t high) {
            T pivot = arr[high];
            int32_t i = low - 1;
            
            for (int32_t j = low; j < high; j++) {
                if (arr[j] <= pivot) {
                    i++;
                    T temp = arr[i];
                    arr[i] = arr[j];
                    arr[j] = temp;
                }
            }
            T temp = arr[i + 1];
            arr[i + 1] = arr[high];
            arr[high] = temp;
            return i + 1;
        }
        
        static void insertionSort(T* arr, int32_t low, int32_t high) {
            for (int32_t i = low + 1; i <= high; i++) {
                T key = arr[i];
                int32_t j = i - 1;
                while (j >= low && arr[j] > key) {
                    arr[j + 1] = arr[j];
                    j--;
                }
                arr[j + 1] = key;
            }
        }
    };
    
    QuickSortImpl::sort(arr, 0, size - 1);
}

template<typename T>
void Sorter::insertionSort(T* arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    for (uint32_t i = 1; i < size; i++) {
        T key = arr[i];
        int32_t j = i - 1;
        
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

template<typename T>
void Sorter::heapSort(T* arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    // 构建最大堆
    for (int32_t i = size / 2 - 1; i >= 0; i--) {
        heapify(arr, size, i);
    }
    
    // 从堆顶提取元素
    for (int32_t i = size - 1; i > 0; i--) {
        T temp = arr[0];
        arr[0] = arr[i];
        arr[i] = temp;
        
        // 对剩余堆进行堆化
        heapify(arr, i, 0);
    }
}

template<typename T>
void Sorter::heapify(T* arr, uint32_t size, uint32_t i) {
    uint32_t largest = i;
    uint32_t left = 2 * i + 1;
    uint32_t right = 2 * i + 2;
    
    if (left < size && arr[left] > arr[largest])
        largest = left;
    
    if (right < size && arr[right] > arr[largest])
        largest = right;
    
    if (largest != i) {
        T temp = arr[i];
        arr[i] = arr[largest];
        arr[largest] = temp;
        
        // 递归堆化
        heapify(arr, size, largest);
    }
}

template<typename T>
void Sorter::mergeSort(T* arr, uint32_t size) {
    if (!arr || size <= 1) return;
    
    // 小数组使用插入排序
    if (size <= 10) {
        insertionSort(arr, size);
        return;
    }
    
    // 分配临时数组
    T* temp = (T*)__math_utils_malloc(size * sizeof(T));
    if (!temp) return; // 内存分配失败
    
    // 归并排序的内部实现
    class MergeSortImpl {
    public:
        static void sort(T* arr, uint32_t left, uint32_t right, T* temp) {
            if (left >= right) return;
            
            uint32_t mid = left + (right - left) / 2;
            
            sort(arr, left, mid, temp);
            sort(arr, mid + 1, right, temp);
            merge(arr, left, mid, right, temp);
        }
        
    private:
        static void merge(T* arr, uint32_t left, uint32_t mid, uint32_t right, T* temp) {
            uint32_t i = left;
            uint32_t j = mid + 1;
            uint32_t k = 0;
            
            while (i <= mid && j <= right) {
                if (arr[i] <= arr[j]) {
                    temp[k++] = arr[i++];
                } else {
                    temp[k++] = arr[j++];
                }
            }
            
            while (i <= mid) {
                temp[k++] = arr[i++];
            }
            
            while (j <= right) {
                temp[k++] = arr[j++];
            }
            
            for (i = 0; i < k; i++) {
                arr[left + i] = temp[i];
            }
        }
    };
    
    MergeSortImpl::sort(arr, 0, size - 1, temp);
    
    __math_utils_free(temp);
}

template<typename T>
int32_t Sorter::binarySearch(const T* arr, uint32_t size, const T& value) {
    if (!arr || size == 0) return -1;
    
    int32_t left = 0;
    int32_t right = size - 1;
    
    while (left <= right) {
        int32_t mid = left + (right - left) / 2;
        
        if (arr[mid] == value) {
            return mid;
        } else if (arr[mid] < value) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    
    return -1; // 未找到
}

template<typename T>
T Sorter::median(T* arr, uint32_t size) {
    if (!arr || size == 0) return T();
    if (size == 1) return arr[0];
    
    // 创建临时数组拷贝
    T* temp = (T*)__math_utils_malloc(size * sizeof(T));
    if (!temp) return arr[0]; // 内存分配失败，返回第一个元素
    
    memcpy(temp, arr, size * sizeof(T));
    
    // 寻找中间元素
    uint32_t mid_idx = size / 2;
    T result;
    
    if (size % 2 == 0) {
        // 偶数个元素
        T val1 = kthElement(temp, size, mid_idx);
        T val2 = kthElement(temp, size, mid_idx - 1);
        result = (val1 + val2) / 2; // 需要确保T类型支持这种运算
    } else {
        // 奇数个元素
        result = kthElement(temp, size, mid_idx);
    }
    
    __math_utils_free(temp);
    return result;
}

template<typename T>
T Sorter::kthElement(T* arr, uint32_t size, uint32_t k) {
    if (!arr || size == 0 || k >= size) return T();
    
    class QuickSelectImpl {
    public:
        static T select(T* arr, int32_t left, int32_t right, uint32_t k) {
            if (left == right) return arr[left];
            
            int32_t pivotIndex = partition(arr, left, right);
            
            if (pivotIndex == (int32_t)k) {
                return arr[pivotIndex];
            } else if (pivotIndex > (int32_t)k) {
                return select(arr, left, pivotIndex - 1, k);
            } else {
                return select(arr, pivotIndex + 1, right, k);
            }
        }
        
    private:
        static int32_t partition(T* arr, int32_t left, int32_t right) {
            T pivot = arr[right];
            int32_t i = left - 1;
            
            for (int32_t j = left; j < right; j++) {
                if (arr[j] <= pivot) {
                    i++;
                    T temp = arr[i];
                    arr[i] = arr[j];
                    arr[j] = temp;
                }
            }
            T temp = arr[i + 1];
            arr[i + 1] = arr[right];
            arr[right] = temp;
            return i + 1;
        }
    };
    
    // 创建副本以避免修改原始数组
    T* copy = (T*)__math_utils_malloc(size * sizeof(T));
    if (!copy) return arr[0]; // 内存分配失败
    
    memcpy(copy, arr, size * sizeof(T));
    T result = QuickSelectImpl::select(copy, 0, size - 1, k);
    
    __math_utils_free(copy);
    return result;
}

// 显式实例化常用类型的模板
template void Sorter::quickSort<int>(int*, uint32_t);
template void Sorter::quickSort<float>(float*, uint32_t);
template void Sorter::quickSort<double>(double*, uint32_t);

template void Sorter::insertionSort<int>(int*, uint32_t);
template void Sorter::insertionSort<float>(float*, uint32_t);
template void Sorter::insertionSort<double>(double*, uint32_t);

template void Sorter::heapSort<int>(int*, uint32_t);
template void Sorter::heapSort<float>(float*, uint32_t);
template void Sorter::heapSort<double>(double*, uint32_t);

template void Sorter::mergeSort<int>(int*, uint32_t);
template void Sorter::mergeSort<float>(float*, uint32_t);
template void Sorter::mergeSort<double>(double*, uint32_t);

template int32_t Sorter::binarySearch<int>(const int*, uint32_t, const int&);
template int32_t Sorter::binarySearch<float>(const float*, uint32_t, const float&);
template int32_t Sorter::binarySearch<double>(const double*, uint32_t, const double&);

template int Sorter::median<int>(int*, uint32_t);
template float Sorter::median<float>(float*, uint32_t);
template double Sorter::median<double>(double*, uint32_t);

template int Sorter::kthElement<int>(int*, uint32_t, uint32_t);
template float Sorter::kthElement<float>(float*, uint32_t, uint32_t);
template double Sorter::kthElement<double>(double*, uint32_t, uint32_t);

} // namespace MathUtils
#endif

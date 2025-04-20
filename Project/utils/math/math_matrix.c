#include "math_matrix.h"
#include <string.h>
#include <stdio.h>
#include "math_utils.h"

// 矩阵创建和内存管理

math_matrix_t* math_matrix_create(uint32_t rows, uint32_t cols) {
    if (rows == 0 || cols == 0) {
        return NULL;
    }
    
    math_matrix_t *matrix = (math_matrix_t*)__math_utils_malloc(sizeof(math_matrix_t));
    if (!matrix) {
        return NULL;
    }
    
    matrix->rows = rows;
    matrix->cols = cols;
    matrix->data = (float*)__math_utils_malloc(rows * cols * sizeof(float));
    
    if (!matrix->data) {
        __math_utils_free(matrix);
        return NULL;
    }
    
    matrix->is_allocated = 1;
    return matrix;
}

math_matrix_t* math_matrix_create_from_array(float *data, uint32_t rows, uint32_t cols) {
    if (!data || rows == 0 || cols == 0) {
        return NULL;
    }
    
    math_matrix_t *matrix = (math_matrix_t*)__math_utils_malloc(sizeof(math_matrix_t));
    if (!matrix) {
        return NULL;
    }
    
    matrix->rows = rows;
    matrix->cols = cols;
    matrix->data = data;
    matrix->is_allocated = 0; // 外部数据，不需要释放
    
    return matrix;
}

math_matrix_t* math_matrix_create_zeros(uint32_t rows, uint32_t cols) {
    math_matrix_t *matrix = math_matrix_create(rows, cols);
    if (matrix) {
        memset(matrix->data, 0, rows * cols * sizeof(float));
    }
    return matrix;
}

math_matrix_t* math_matrix_create_ones(uint32_t rows, uint32_t cols) {
    math_matrix_t *matrix = math_matrix_create(rows, cols);
    if (matrix) {
        for (uint32_t i = 0; i < rows * cols; i++) {
            matrix->data[i] = 1.0f;
        }
    }
    return matrix;
}

math_matrix_t* math_matrix_create_identity(uint32_t size) {
    math_matrix_t *matrix = math_matrix_create_zeros(size, size);
    if (matrix) {
        for (uint32_t i = 0; i < size; i++) {
            matrix->data[i * size + i] = 1.0f;
        }
    }
    return matrix;
}

void math_matrix_destroy(math_matrix_t *matrix) {
    if (!matrix) {
        return;
    }
    
    if (matrix->is_allocated && matrix->data) {
        __math_utils_free(matrix->data);
    }
    
    __math_utils_free(matrix);
}

// 矩阵索引和访问

float math_matrix_get(const math_matrix_t *matrix, uint32_t row, uint32_t col) {
    if (!matrix || row >= matrix->rows || col >= matrix->cols) {
        return 0.0f;
    }
    
    return matrix->data[row * matrix->cols + col];
}

void math_matrix_set(math_matrix_t *matrix, uint32_t row, uint32_t col, float value) {
    if (!matrix || row >= matrix->rows || col >= matrix->cols) {
        return;
    }
    
    matrix->data[row * matrix->cols + col] = value;
}

uint32_t math_matrix_index(const math_matrix_t *matrix, uint32_t row, uint32_t col) {
    if (!matrix || row >= matrix->rows || col >= matrix->cols) {
        return 0;
    }
    
    return row * matrix->cols + col;
}

void math_matrix_get_row(const math_matrix_t *matrix, uint32_t row, float *dst) {
    if (!matrix || !dst || row >= matrix->rows) {
        return;
    }
    
    memcpy(dst, &matrix->data[row * matrix->cols], matrix->cols * sizeof(float));
}

void math_matrix_get_col(const math_matrix_t *matrix, uint32_t col, float *dst) {
    if (!matrix || !dst || col >= matrix->cols) {
        return;
    }
    
    for (uint32_t i = 0; i < matrix->rows; i++) {
        dst[i] = matrix->data[i * matrix->cols + col];
    }
}

// 矩阵基本操作

void math_matrix_copy(const math_matrix_t *src, math_matrix_t *dst) {
    if (!src || !dst || src->rows != dst->rows || src->cols != dst->cols) {
        return;
    }
    
    memcpy(dst->data, src->data, src->rows * src->cols * sizeof(float));
}

void math_matrix_transpose(const math_matrix_t *src, math_matrix_t *dst) {
    if (!src || !dst || src->rows != dst->cols || src->cols != dst->rows) {
        return;
    }
    
    for (uint32_t i = 0; i < src->rows; i++) {
        for (uint32_t j = 0; j < src->cols; j++) {
            dst->data[j * dst->cols + i] = src->data[i * src->cols + j];
        }
    }
}

void math_matrix_add(const math_matrix_t *a, const math_matrix_t *b, math_matrix_t *result) {
    if (!a || !b || !result || 
        a->rows != b->rows || a->cols != b->cols || 
        a->rows != result->rows || a->cols != result->cols) {
        return;
    }
    
    arm_matrix_instance_f32 arm_a, arm_b, arm_result;
    arm_a.numRows = a->rows;
    arm_a.numCols = a->cols;
    arm_a.pData = a->data;
    
    arm_b.numRows = b->rows;
    arm_b.numCols = b->cols;
    arm_b.pData = b->data;
    
    arm_result.numRows = result->rows;
    arm_result.numCols = result->cols;
    arm_result.pData = result->data;
    
    arm_mat_add_f32(&arm_a, &arm_b, &arm_result);
}

void math_matrix_subtract(const math_matrix_t *a, const math_matrix_t *b, math_matrix_t *result) {
    if (!a || !b || !result || 
        a->rows != b->rows || a->cols != b->cols || 
        a->rows != result->rows || a->cols != result->cols) {
        return;
    }
    
    arm_matrix_instance_f32 arm_a, arm_b, arm_result;
    arm_a.numRows = a->rows;
    arm_a.numCols = a->cols;
    arm_a.pData = a->data;
    
    arm_b.numRows = b->rows;
    arm_b.numCols = b->cols;
    arm_b.pData = b->data;
    
    arm_result.numRows = result->rows;
    arm_result.numCols = result->cols;
    arm_result.pData = result->data;
    
    arm_mat_sub_f32(&arm_a, &arm_b, &arm_result);
}

void math_matrix_multiply(const math_matrix_t *a, const math_matrix_t *b, math_matrix_t *result) {
    if (!a || !b || !result || 
        a->cols != b->rows || 
        a->rows != result->rows || b->cols != result->cols) {
        return;
    }

    // 检查自引用情况，避免破坏输入数据
    float* temp = NULL;
    if (result == a || result == b) {
        temp = (float*)__math_utils_malloc(result->rows * result->cols * sizeof(float));
        if (!temp) return;
    }

    // 使用CMSIS-DSP库的arm_mat_mult_f32函数
    arm_matrix_instance_f32 arm_a, arm_b, arm_result;
    arm_a.numRows = a->rows;
    arm_a.numCols = a->cols;
    arm_a.pData = a->data;

    arm_b.numRows = b->rows;
    arm_b.numCols = b->cols;
    arm_b.pData = b->data;

    // 如果需要临时存储，使用临时缓冲区
    if (temp) {
        arm_result.pData = temp;
    } else {
        arm_result.pData = result->data;
    }
    arm_result.numRows = result->rows;
    arm_result.numCols = result->cols;

    // 执行矩阵乘法
    arm_status status = arm_mat_mult_f32(&arm_a, &arm_b, &arm_result);
    
    // 如果使用了临时缓冲区，将结果复制回目标矩阵
    if (temp) {
        memcpy(result->data, temp, result->rows * result->cols * sizeof(float));
        __math_utils_free(temp);
    }
}

void math_matrix_scale(const math_matrix_t *src, float scalar, math_matrix_t *result) {
    if (!src || !result || src->rows != result->rows || src->cols != result->cols) {
        return;
    }
    
    arm_matrix_instance_f32 arm_src, arm_result;
    arm_src.numRows = src->rows;
    arm_src.numCols = src->cols;
    arm_src.pData = src->data;
    
    arm_result.numRows = result->rows;
    arm_result.numCols = result->cols;
    arm_result.pData = result->data;
    
    arm_mat_scale_f32(&arm_src, scalar, &arm_result);
}

// 计算2x2矩阵的行列式
static float math_det2x2(float a, float b, float c, float d) {
    return a * d - b * c;
}

// 计算3x3矩阵的行列式
static float math_det3x3(const math_matrix_t *matrix) {
    float a = matrix->data[0];
    float b = matrix->data[1];
    float c = matrix->data[2];
    float d = matrix->data[3];
    float e = matrix->data[4];
    float f = matrix->data[5];
    float g = matrix->data[6];
    float h = matrix->data[7];
    float i = matrix->data[8];
    
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

// 矩阵高级操作

float math_matrix_determinant(const math_matrix_t *matrix) {
    if (!matrix || matrix->rows != matrix->cols) {
        return 0.0f;
    }
    
    if (matrix->rows == 1) {
        return matrix->data[0];
    }
    
    if (matrix->rows == 2) {
        return math_det2x2(
            matrix->data[0], matrix->data[1],
            matrix->data[2], matrix->data[3]
        );
    }
    
    if (matrix->rows == 3) {
        return math_det3x3(matrix);
    }
    
    // 对于更大的矩阵，使用初等行变换和上三角矩阵
    math_matrix_t *temp = math_matrix_create(matrix->rows, matrix->cols);
    if (!temp) {
        return 0.0f;
    }
    
    math_matrix_copy(matrix, temp);
    
    float det = 1.0f;
    for (uint32_t i = 0; i < matrix->rows; i++) {
        // 如果对角线元素为零，寻找非零元素交换行
        if (fabsf(temp->data[i * temp->cols + i]) < FLOAT_EPSILON) {
            uint32_t swap_row = i + 1;
            while (swap_row < matrix->rows && 
                   fabsf(temp->data[swap_row * temp->cols + i]) < FLOAT_EPSILON) {
                swap_row++;
            }
            
            if (swap_row >= matrix->rows) {
                math_matrix_destroy(temp);
                return 0.0f; // 奇异矩阵，行列式为零
            }
            
            // 交换行并改变行列式符号
            for (uint32_t j = i; j < matrix->cols; j++) {
                float t = temp->data[i * temp->cols + j];
                temp->data[i * temp->cols + j] = temp->data[swap_row * temp->cols + j];
                temp->data[swap_row * temp->cols + j] = t;
            }
            det = -det;
        }
        
        // 保存对角线元素
        float pivot = temp->data[i * temp->cols + i];
        det *= pivot;
        
        // 将当前行以下的行进行消元
        for (uint32_t j = i + 1; j < matrix->rows; j++) {
            float factor = temp->data[j * temp->cols + i] / pivot;
            temp->data[j * temp->cols + i] = 0.0f;
            
            for (uint32_t k = i + 1; k < matrix->cols; k++) {
                temp->data[j * temp->cols + k] -= factor * temp->data[i * temp->cols + k];
            }
        }
    }
    
    math_matrix_destroy(temp);
    return det;
}

uint8_t math_matrix_inverse(const math_matrix_t *src, math_matrix_t *dst) {
    if (!src || !dst || src->rows != src->cols || dst->rows != dst->cols || src->rows != dst->rows) {
        return 0;
    }

    // 创建一个临时矩阵来避免源矩阵被修改(CMSIS会修改输入)
    math_matrix_t* temp = math_matrix_create(src->rows, src->cols);
    if (!temp) return 0;
    
    // 复制源矩阵到临时矩阵
    math_matrix_copy(src, temp);
    
    // 使用CMSIS-DSP库的arm_mat_inverse_f32函数
    arm_matrix_instance_f32 arm_src, arm_dst;
    arm_src.numRows = temp->rows;
    arm_src.numCols = temp->cols;
    arm_src.pData = temp->data;
    
    arm_dst.numRows = dst->rows;
    arm_dst.numCols = dst->cols;
    arm_dst.pData = dst->data;
    
    arm_status status = arm_mat_inverse_f32(&arm_src, &arm_dst);
    
    // 清理临时矩阵
    math_matrix_destroy(temp);
    
    // 返回操作是否成功
    return (status == ARM_MATH_SUCCESS) ? 1 : 0;
}

uint8_t math_matrix_lu_decompose(const math_matrix_t *src, math_matrix_t *L, math_matrix_t *U, math_matrix_t *P) {
    if (!src || !L || !U || !P || 
        src->rows != src->cols || 
        L->rows != L->cols || U->rows != U->cols || P->rows != P->cols ||
        src->rows != L->rows || src->rows != U->rows || src->rows != P->rows) {
        return 0;
    }
    
    uint32_t n = src->rows;
    
    // 初始化 L, U, P
    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t j = 0; j < n; j++) {
            L->data[i * n + j] = (i == j) ? 1.0f : 0.0f;
            U->data[i * n + j] = 0.0f;
            P->data[i * n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 创建一个临时矩阵来存储 A
    math_matrix_t *A = math_matrix_create(n, n);
    if (!A) {
        return 0;
    }
    math_matrix_copy(src, A);
    
    for (uint32_t k = 0; k < n; k++) {
        // 寻找主元素
        float max_val = 0.0f;
        uint32_t max_row = k;
        
        for (uint32_t i = k; i < n; i++) {
            float abs_val = fabsf(A->data[i * n + k]);
            if (abs_val > max_val) {
                max_val = abs_val;
                max_row = i;
            }
        }
        
        if (max_val < FLOAT_EPSILON) {
            // 矩阵是奇异的
            math_matrix_destroy(A);
            return 0;
        }
        
        // 交换行
        if (max_row != k) {
            for (uint32_t j = 0; j < n; j++) {
                float temp = A->data[k * n + j];
                A->data[k * n + j] = A->data[max_row * n + j];
                A->data[max_row * n + j] = temp;
                
                temp = P->data[k * n + j];
                P->data[k * n + j] = P->data[max_row * n + j];
                P->data[max_row * n + j] = temp;
                
                if (j < k) {
                    temp = L->data[k * n + j];
                    L->data[k * n + j] = L->data[max_row * n + j];
                    L->data[max_row * n + j] = temp;
                }
            }
        }
        
        // 设置U的当前行
        for (uint32_t j = k; j < n; j++) {
            U->data[k * n + j] = A->data[k * n + j];
        }
        
        // 更新L的当前列和A的剩余部分
        for (uint32_t i = k + 1; i < n; i++) {
            float factor = A->data[i * n + k] / A->data[k * n + k];
            L->data[i * n + k] = factor;
            
            for (uint32_t j = k; j < n; j++) {
                A->data[i * n + j] -= factor * A->data[k * n + j];
            }
        }
    }
    
    math_matrix_destroy(A);
    return 1;
}

uint8_t math_matrix_solve(const math_matrix_t *A, const math_matrix_t *b, math_matrix_t *x) {
    if (!A || !b || !x || A->rows != A->cols || A->rows != b->rows ||
        b->cols != 1 || x->rows != A->cols || x->cols != 1) {
        return 0;
    }
    
    // 使用矩阵求逆方法解线性方程组：x = A^-1 * b
    math_matrix_t* A_inv = math_matrix_create(A->rows, A->cols);
    if (!A_inv) return 0;
    
    // 求A的逆矩阵
    uint8_t inv_result = math_matrix_inverse(A, A_inv);
    if (inv_result == 0) {
        math_matrix_destroy(A_inv);
        return 0; // 矩阵不可逆
    }
    
    // 计算x = A^-1 * b
    math_matrix_multiply(A_inv, b, x);
    
    // 释放内存
    math_matrix_destroy(A_inv);
    
    return 1;
}

float math_matrix_norm_frobenius(const math_matrix_t *matrix) {
    if (!matrix) {
        return 0.0f;
    }
    
    float sum = 0.0f;
    for (uint32_t i = 0; i < matrix->rows * matrix->cols; i++) {
        sum += matrix->data[i] * matrix->data[i];
    }
    
    return sqrtf(sum);
}

float math_matrix_norm_inf(const math_matrix_t *matrix) {
    if (!matrix) {
        return 0.0f;
    }
    
    float max_sum = 0.0f;
    for (uint32_t i = 0; i < matrix->rows; i++) {
        float row_sum = 0.0f;
        for (uint32_t j = 0; j < matrix->cols; j++) {
            row_sum += fabsf(matrix->data[i * matrix->cols + j]);
        }
        if (row_sum > max_sum) {
            max_sum = row_sum;
        }
    }
    
    return max_sum;
}

void math_matrix_print(const math_matrix_t *matrix, const char *name) {
    if (!matrix) {
        printf("Matrix %s is NULL\n", name ? name : "");
        return;
    }
    
    printf("Matrix %s [%u x %u]:\n", name ? name : "", matrix->rows, matrix->cols);
    
    for (uint32_t i = 0; i < matrix->rows; i++) {
        printf("  ");
        for (uint32_t j = 0; j < matrix->cols; j++) {
            printf("%9.4f ", matrix->data[i * matrix->cols + j]);
        }
        printf("\n");
    }
}
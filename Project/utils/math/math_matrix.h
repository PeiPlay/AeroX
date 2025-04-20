#ifndef __MATH_MATRIX_H__
#define __MATH_MATRIX_H__

#include <stdint.h>
#include "arm_math.h"
#include "math_const.h"

#ifdef __cplusplus
extern "C" {
#endif

// 矩阵结构定义
typedef struct {
    uint32_t rows;        // 行数
    uint32_t cols;        // 列数
    float *data;          // 数据指针，行主序存储
    uint8_t is_allocated; // 是否是动态分配的内存
} math_matrix_t;

// 矩阵创建和内存管理
math_matrix_t* math_matrix_create(uint32_t rows, uint32_t cols);
math_matrix_t* math_matrix_create_from_array(float *data, uint32_t rows, uint32_t cols);
math_matrix_t* math_matrix_create_zeros(uint32_t rows, uint32_t cols);
math_matrix_t* math_matrix_create_ones(uint32_t rows, uint32_t cols);
math_matrix_t* math_matrix_create_identity(uint32_t size);
void math_matrix_destroy(math_matrix_t *matrix);

// 矩阵索引和访问
float math_matrix_get(const math_matrix_t *matrix, uint32_t row, uint32_t col);
void math_matrix_set(math_matrix_t *matrix, uint32_t row, uint32_t col, float value);
uint32_t math_matrix_index(const math_matrix_t *matrix, uint32_t row, uint32_t col);
void math_matrix_get_row(const math_matrix_t *matrix, uint32_t row, float *dst);
void math_matrix_get_col(const math_matrix_t *matrix, uint32_t col, float *dst);

// 矩阵基本操作
void math_matrix_copy(const math_matrix_t *src, math_matrix_t *dst);
void math_matrix_transpose(const math_matrix_t *src, math_matrix_t *dst);
void math_matrix_add(const math_matrix_t *a, const math_matrix_t *b, math_matrix_t *result);
void math_matrix_subtract(const math_matrix_t *a, const math_matrix_t *b, math_matrix_t *result);
void math_matrix_multiply(const math_matrix_t *a, const math_matrix_t *b, math_matrix_t *result);
void math_matrix_scale(const math_matrix_t *src, float scalar, math_matrix_t *result);

// 矩阵高级操作
float math_matrix_determinant(const math_matrix_t *matrix);
uint8_t math_matrix_inverse(const math_matrix_t *src, math_matrix_t *dst);
uint8_t math_matrix_lu_decompose(const math_matrix_t *src, math_matrix_t *L, math_matrix_t *U, math_matrix_t *P);
uint8_t math_matrix_solve(const math_matrix_t *A, const math_matrix_t *b, math_matrix_t *x);


// 矩阵范数计算
float math_matrix_norm_frobenius(const math_matrix_t *matrix);
float math_matrix_norm_inf(const math_matrix_t *matrix);

// 打印矩阵（用于调试）
void math_matrix_print(const math_matrix_t *matrix, const char *name);

#ifdef __cplusplus
}
#endif // __cplusplus
#ifdef __cplusplus

namespace MathUtils {
// C++类封装
class Matrix {
private:
    math_matrix_t* m_matrix;
    bool m_owner;

public:
    // 构造函数
    Matrix(uint32_t rows, uint32_t cols);
    explicit Matrix(float* data, uint32_t rows, uint32_t cols, bool copy = true); // 添加explicit关键字避免隐式转换
    Matrix(const Matrix& other);
    Matrix(math_matrix_t* matrix, bool takeOwnership = false);
    
    // 析构函数
    ~Matrix();
    
    // 赋值操作符
    Matrix& operator=(const Matrix& other);
    
    // 访问元素
    float& operator()(uint32_t row, uint32_t col);
    float operator()(uint32_t row, uint32_t col) const;
    
    // 矩阵运算
    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    Matrix operator*(float scalar) const;
    
    // 添加新的方法以允许结果存放在用户指定的Matrix中
    bool add(const Matrix& other, Matrix& result) const;
    bool subtract(const Matrix& other, Matrix& result) const;
    bool multiply(const Matrix& other, Matrix& result) const;
    bool scale(float scalar, Matrix& result) const;
    bool transpose(Matrix& result) const;
    
    // 矩阵操作
    Matrix transpose() const;
    bool inverse(Matrix& result) const;
    float determinant() const;
    
    // 获取矩阵尺寸
    uint32_t rows() const { return m_matrix->rows; }
    uint32_t cols() const { return m_matrix->cols; }
    
    // 获取底层C结构
    math_matrix_t* getInternal() const { return m_matrix; }

    // 静态工厂方法
    static Matrix zeros(uint32_t rows, uint32_t cols);
    static Matrix ones(uint32_t rows, uint32_t cols);
    static Matrix identity(uint32_t size);
};

} // namespace MathUtils

#endif /* __cplusplus */

#endif /* __MATH_MATRIX_H__ */
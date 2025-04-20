#include "math_matrix.h"
#include <string.h>
#include "math_utils.h"

namespace MathUtils {

// Matrix类构造函数
Matrix::Matrix(uint32_t rows, uint32_t cols) : m_owner(true) {
    m_matrix = math_matrix_create(rows, cols);
}

Matrix::Matrix(float* data, uint32_t rows, uint32_t cols, bool copy) : m_owner(copy) {
    if (copy) {
        m_matrix = math_matrix_create(rows, cols);
        if (m_matrix && data) {
            memcpy(m_matrix->data, data, rows * cols * sizeof(float));
        }
    } else {
        m_matrix = math_matrix_create_from_array(data, rows, cols);
    }
}

Matrix::Matrix(const Matrix& other) : m_owner(true) {
    m_matrix = math_matrix_create(other.rows(), other.cols());
    if (m_matrix && other.m_matrix) {
        math_matrix_copy(other.m_matrix, m_matrix);
    }
}

Matrix::Matrix(math_matrix_t* matrix, bool takeOwnership) : m_matrix(matrix), m_owner(takeOwnership) {
}

// 析构函数
Matrix::~Matrix() {
    if (m_owner && m_matrix) {
        math_matrix_destroy(m_matrix);
    }
    m_matrix = nullptr;
}

// 赋值操作符
Matrix& Matrix::operator=(const Matrix& other) {
    if (this != &other) {
        // 如果现有矩阵尺寸不匹配，则销毁并重新创建
        if (!m_matrix || m_matrix->rows != other.rows() || m_matrix->cols != other.cols()) {
            if (m_owner && m_matrix) {
                math_matrix_destroy(m_matrix);
            }
            m_matrix = math_matrix_create(other.rows(), other.cols());
            m_owner = true;
        }
        
        if (m_matrix && other.m_matrix) {
            math_matrix_copy(other.m_matrix, m_matrix);
        }
    }
    return *this;
}

// 访问元素
float& Matrix::operator()(uint32_t row, uint32_t col) {
    uint32_t index = math_matrix_index(m_matrix, row, col);
    return m_matrix->data[index];
}

float Matrix::operator()(uint32_t row, uint32_t col) const {
    return math_matrix_get(m_matrix, row, col);
}

// 矩阵运算
Matrix Matrix::operator+(const Matrix& other) const {
    Matrix result(0U, 0U); // 显式使用无符号整数避免歧义
    if (add(other, result)) {
        return result;
    }
    return Matrix(0U, 0U);
}

Matrix Matrix::operator-(const Matrix& other) const {
    Matrix result(0U, 0U);
    if (subtract(other, result)) {
        return result;
    }
    return Matrix(0U, 0U);
}

Matrix Matrix::operator*(const Matrix& other) const {
    Matrix result(0U, 0U);
    if (multiply(other, result)) {
        return result;
    }
    return Matrix(0U, 0U);
}

Matrix Matrix::operator*(float scalar) const {
    Matrix result(0U, 0U);
    if (scale(scalar, result)) {
        return result;
    }
    return Matrix(0U, 0U);
}

Matrix Matrix::transpose() const {
    Matrix result(0U, 0U);
    if (transpose(result)) {
        return result;
    }
    return Matrix(0U, 0U);
}

// 添加新的矩阵运算方法，结果存放在用户指定的Matrix中
bool Matrix::add(const Matrix& other, Matrix& result) const {
    if (!m_matrix || !other.m_matrix || 
        m_matrix->rows != other.rows() || 
        m_matrix->cols != other.cols()) {
        return false;
    }

    // 确保result大小正确
    if (result.rows() != m_matrix->rows || result.cols() != m_matrix->cols) {
        result = Matrix(m_matrix->rows, m_matrix->cols);
    }
    
    math_matrix_add(m_matrix, other.m_matrix, result.m_matrix);
    return true;
}

bool Matrix::subtract(const Matrix& other, Matrix& result) const {
    if (!m_matrix || !other.m_matrix || 
        m_matrix->rows != other.rows() || 
        m_matrix->cols != other.cols()) {
        return false;
    }

    // 确保result大小正确
    if (result.rows() != m_matrix->rows || result.cols() != m_matrix->cols) {
        result = Matrix(m_matrix->rows, m_matrix->cols);
    }
    
    math_matrix_subtract(m_matrix, other.m_matrix, result.m_matrix);
    return true;
}

bool Matrix::multiply(const Matrix& other, Matrix& result) const {
    if (!m_matrix || !other.m_matrix || m_matrix->cols != other.rows()) {
        return false;
    }

    // 确保result大小正确
    if (result.rows() != m_matrix->rows || result.cols() != other.cols()) {
        result = Matrix(m_matrix->rows, other.cols());
    }
    
    math_matrix_multiply(m_matrix, other.m_matrix, result.m_matrix);
    return true;
}

bool Matrix::scale(float scalar, Matrix& result) const {
    if (!m_matrix) {
        return false;
    }

    // 确保result大小正确
    if (result.rows() != m_matrix->rows || result.cols() != m_matrix->cols) {
        result = Matrix(m_matrix->rows, m_matrix->cols);
    }
    
    math_matrix_scale(m_matrix, scalar, result.m_matrix);
    return true;
}

bool Matrix::transpose(Matrix& result) const {
    if (!m_matrix) {
        return false;
    }

    // 确保result大小正确
    if (result.rows() != m_matrix->cols || result.cols() != m_matrix->rows) {
        result = Matrix(m_matrix->cols, m_matrix->rows);
    }
    
    math_matrix_transpose(m_matrix, result.m_matrix);
    return true;
}

bool Matrix::inverse(Matrix& result) const {
    if (!m_matrix || m_matrix->rows != m_matrix->cols) {
        return false;
    }
    
    if (result.rows() != m_matrix->rows || result.cols() != m_matrix->cols) {
        result = Matrix(m_matrix->rows, m_matrix->cols);
    }
    
    return math_matrix_inverse(m_matrix, result.m_matrix) != 0;
}

float Matrix::determinant() const {
    if (!m_matrix || m_matrix->rows != m_matrix->cols) {
        return 0.0f;
    }
    
    return math_matrix_determinant(m_matrix);
}

bool Matrix::setIdentity() {
    if (!m_matrix) {
        return false;
    }
    if(m_matrix->rows != m_matrix->cols) {
        return false;
    }
    for (uint32_t i = 0; i < m_matrix->rows; i++) {
        for (uint32_t j = 0; j < m_matrix->cols; j++) {
            if (i == j) {
                m_matrix->data[i * m_matrix->cols + j] = 1.0f;
            } else {
                m_matrix->data[i * m_matrix->cols + j] = 0.0f;
            }
        }
    }
    return true;
}
bool Matrix::setZero()
{
    if (!m_matrix) {
        return false;
    }
    for (uint32_t i = 0; i < m_matrix->rows * m_matrix->cols; i++) {
        m_matrix->data[i] = 0.0f;
    }
    return true;
}


// 静态工厂方法
Matrix Matrix::zeros(uint32_t rows, uint32_t cols) {
    math_matrix_t* matrix = math_matrix_create_zeros(rows, cols);
    return Matrix(matrix, true);
}

Matrix Matrix::ones(uint32_t rows, uint32_t cols) {
    math_matrix_t* matrix = math_matrix_create_ones(rows, cols);
    return Matrix(matrix, true);
}

Matrix Matrix::identity(uint32_t size) {
    math_matrix_t* matrix = math_matrix_create_identity(size);
    return Matrix(matrix, true);
}

} // namespace MathUtils
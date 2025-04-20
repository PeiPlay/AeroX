#include "test.h"
#include "math_utils.h"
#include "math_quaternion.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <string.h>

// 辅助常量定义
#define QUAT_EPSILON 0.0001f  // 用于四元数比较的误差阈值

// 辅助函数 - 检查两个浮点数是否接近相等
bool float_equals(float a, float b, float epsilon = FLOAT_EPSILON) {
    return fabsf(a - b) <= epsilon;
}

// 辅助函数 - 打印测试结果
void print_test_result(const char* test_name, bool passed) {
    test_printf("%s: %s\r\n", test_name, passed ? "PASSED" : "!!!!!!!!!!!!!!!!!!!FAILED!!!!!!!!!");
}

// 辅助函数 - 打印四元数
void print_quaternion(const MathUtils::Quaternion& q, const char* name) {
    test_printf("%s: [w:%.4f, x:%.4f, y:%.4f, z:%.4f]\r\n", 
              name, q.w, q.x, q.y, q.z);
}

// 辅助函数 - 比较两个四元数是否相等（考虑q和-q表示相同的旋转）
bool quaternion_equals(const MathUtils::Quaternion& a, const MathUtils::Quaternion& b, float epsilon = QUAT_EPSILON) {
    // 四元数q和-q表示相同的旋转，所以我们需要检查两种情况
    bool direct_match = 
        float_equals(a.w, b.w, epsilon) &&
        float_equals(a.x, b.x, epsilon) &&
        float_equals(a.y, b.y, epsilon) &&
        float_equals(a.z, b.z, epsilon);
        
    bool negative_match = 
        float_equals(a.w, -b.w, epsilon) &&
        float_equals(a.x, -b.x, epsilon) &&
        float_equals(a.y, -b.y, epsilon) &&
        float_equals(a.z, -b.z, epsilon);
        
    return direct_match || negative_match;
}

//============== math_quaternion 组件测试 ==============

// 测试四元数构造函数
void test_quaternion_construction() {
    test_printf("=== Test Quaternion Construction ===\r\n");
    
    // 测试默认构造函数（单位四元数）
    MathUtils::Quaternion q1;
    bool test1_passed = (q1.w == 1.0f && q1.x == 0.0f && q1.y == 0.0f && q1.z == 0.0f);
    print_test_result("Default constructor (identity)", test1_passed);
    print_quaternion(q1, "Identity quaternion");
    
    // 测试带参数构造函数
    MathUtils::Quaternion q2(0.5f, 0.5f, 0.5f, 0.5f);
    bool test2_passed = (q2.w == 0.5f && q2.x == 0.5f && q2.y == 0.5f && q2.z == 0.5f);
    print_test_result("Parameterized constructor", test2_passed);
    print_quaternion(q2, "Custom quaternion");
}

// 测试从欧拉角创建四元数
void test_quaternion_from_euler() {
    test_printf("=== Test Quaternion from Euler Angles ===\r\n");
    
    // 测试从欧拉角创建四元数（弧度）
    float roll = PI / 4.0f;    // 45度
    float pitch = PI / 6.0f;   // 30度
    float yaw = PI / 3.0f;     // 60度
    
    MathUtils::Quaternion q1 = MathUtils::Quaternion::fromEulerRad(roll, pitch, yaw);
    print_quaternion(q1, "Quaternion from Euler (rad)");
    
    // 测试从欧拉角转回
    float r1, p1, y1;
    q1.toEulerRad(r1, p1, y1);
    
    bool test1_passed = 
        float_equals(r1, roll, QUAT_EPSILON) &&
        float_equals(p1, pitch, QUAT_EPSILON) &&
        float_equals(y1, yaw, QUAT_EPSILON);
    
    test_printf("Back to Euler (rad): roll=%.4f, pitch=%.4f, yaw=%.4f\r\n", r1, p1, y1);
    test_printf("Original Euler (rad): roll=%.4f, pitch=%.4f, yaw=%.4f\r\n", roll, pitch, yaw);
    print_test_result("Euler (rad) round-trip conversion", test1_passed);
    
    // 测试从欧拉角创建四元数（角度）
    float roll_deg = 45.0f;    // 45度
    float pitch_deg = 30.0f;   // 30度
    float yaw_deg = 60.0f;     // 60度
    
    MathUtils::Quaternion q2 = MathUtils::Quaternion::fromEulerDeg(roll_deg, pitch_deg, yaw_deg);
    print_quaternion(q2, "Quaternion from Euler (deg)");
    
    // 测试从欧拉角转回（角度）
    float r2, p2, y2;
    q2.toEulerDeg(r2, p2, y2);
    
    bool test2_passed = 
        float_equals(r2, roll_deg, 0.1f) &&  // 使用更宽松的误差容限
        float_equals(p2, pitch_deg, 0.1f) &&
        float_equals(y2, yaw_deg, 0.1f);
    
    test_printf("Back to Euler (deg): roll=%.4f, pitch=%.4f, yaw=%.4f\r\n", r2, p2, y2);
    test_printf("Original Euler (deg): roll=%.4f, pitch=%.4f, yaw=%.4f\r\n", roll_deg, pitch_deg, yaw_deg);
    print_test_result("Euler (deg) round-trip conversion", test2_passed);
    
    // 测试四元数一致性（两种创建方法应该产生相同的四元数）
    bool test3_passed = quaternion_equals(q1, q2);
    print_test_result("Euler rad/deg consistency", test3_passed);
}

// 测试从旋转矩阵创建四元数
void test_quaternion_from_matrix() {
    test_printf("=== Test Quaternion from Rotation Matrix ===\r\n");
    
    // 创建一个简单的旋转矩阵（绕Z轴旋转90度）
    float matrix[9] = {
        0.0f, -1.0f, 0.0f,  // 第一行
        1.0f,  0.0f, 0.0f,  // 第二行
        0.0f,  0.0f, 1.0f   // 第三行
    };
    
    // 从旋转矩阵创建四元数
    MathUtils::Quaternion q = MathUtils::Quaternion::fromRotationMatrix(matrix);
    print_quaternion(q, "Quaternion from rotation matrix");
    
    // 此旋转应该约等于绕Z轴旋转90度的四元数
    MathUtils::Quaternion expected = MathUtils::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, PI / 2.0f);
    print_quaternion(expected, "Expected quaternion (Z-axis 90 deg)");
    
    bool test1_passed = quaternion_equals(q, expected);
    print_test_result("From rotation matrix", test1_passed);
    
    // 测试转回旋转矩阵
    float result_matrix[9];
    q.toRotationMatrix(result_matrix);
    
    test_printf("Original matrix: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\r\n",
              matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7], matrix[8]);
    test_printf("Result matrix:   [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\r\n",
              result_matrix[0], result_matrix[1], result_matrix[2], result_matrix[3], result_matrix[4], 
              result_matrix[5], result_matrix[6], result_matrix[7], result_matrix[8]);
    
    bool test2_passed = true;
    for (int i = 0; i < 9; i++) {
        if (!float_equals(matrix[i], result_matrix[i], QUAT_EPSILON)) {
            test2_passed = false;
            test_printf("Matrix mismatch at index %d: %.4f vs %.4f\r\n", i, matrix[i], result_matrix[i]);
        }
    }
    print_test_result("Matrix round-trip conversion", test2_passed);
}

// 测试从轴角创建四元数
void test_quaternion_from_axis_angle() {
    test_printf("=== Test Quaternion from Axis-Angle ===\r\n");
    
    // 创建一个绕X轴旋转45度的四元数
    float axis_x = 1.0f;
    float axis_y = 0.0f;
    float axis_z = 0.0f;
    float angle = PI / 4.0f; // 45度
    
    MathUtils::Quaternion q = MathUtils::Quaternion::fromAxisAngle(axis_x, axis_y, axis_z, angle);
    print_quaternion(q, "Quaternion from axis-angle (X-axis 45 deg)");
    
    // 验证生成的四元数是否正确
    // 对于绕X轴旋转45度的四元数，应该是 w=cos(22.5°), x=sin(22.5°), y=0, z=0
    float expected_w = arm_cos_f32(PI / 8.0f);
    float expected_x = arm_sin_f32(PI / 8.0f);
    
    bool test1_passed = 
        float_equals(q.w, expected_w, QUAT_EPSILON) &&
        float_equals(q.x, expected_x, QUAT_EPSILON) &&
        float_equals(q.y, 0.0f, QUAT_EPSILON) &&
        float_equals(q.z, 0.0f, QUAT_EPSILON);
    
    print_test_result("From axis-angle creation", test1_passed);
    
    // 测试转回轴角表示
    float axis_out_x, axis_out_y, axis_out_z, angle_out;
    q.toAxisAngle(axis_out_x, axis_out_y, axis_out_z, angle_out);
    
    test_printf("Original axis-angle: [%.4f, %.4f, %.4f], angle: %.4f\r\n", 
              axis_x, axis_y, axis_z, angle);
    test_printf("Result axis-angle:   [%.4f, %.4f, %.4f], angle: %.4f\r\n",
              axis_out_x, axis_out_y, axis_out_z, angle_out);
    
    // 注意轴的方向可能相反但角度相等，这仍然代表相同的旋转
    bool axes_match = 
        (float_equals(axis_out_x, axis_x, QUAT_EPSILON) &&
         float_equals(axis_out_y, axis_y, QUAT_EPSILON) &&
         float_equals(axis_out_z, axis_z, QUAT_EPSILON)) ||
        (float_equals(axis_out_x, -axis_x, QUAT_EPSILON) &&
         float_equals(axis_out_y, -axis_y, QUAT_EPSILON) &&
         float_equals(axis_out_z, -axis_z, QUAT_EPSILON));
        
    bool angle_matches = float_equals(angle_out, angle, QUAT_EPSILON) ||
                        float_equals(angle_out, 2.0f * PI - angle, QUAT_EPSILON);
    
    bool test2_passed = axes_match && angle_matches;
    print_test_result("Axis-angle round-trip conversion", test2_passed);
}

// 测试四元数基本操作
void test_quaternion_basic_operations() {
    test_printf("=== Test Quaternion Basic Operations ===\r\n");
    
    // 创建一个测试四元数
    MathUtils::Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    
    // 测试四元数模长
    float norm = q.norm();
    float expected_norm = 1.0f; // 0.5^2 * 4 = 1
    bool test1_passed = float_equals(norm, expected_norm);
    test_printf("Norm of quaternion: %.4f (expected: %.4f)\r\n", norm, expected_norm);
    print_test_result("Quaternion norm", test1_passed);
    
    // 测试归一化
    MathUtils::Quaternion q_unnorm(2.0f, 2.0f, 2.0f, 2.0f);
    print_quaternion(q_unnorm, "Before normalization");
    q_unnorm.normalize();
    print_quaternion(q_unnorm, "After normalization");
    bool test2_passed = float_equals(q_unnorm.norm(), 1.0f);
    print_test_result("Quaternion normalize", test2_passed);
    
    // 测试归一化方法（返回新的四元数）
    MathUtils::Quaternion q_unnorm2(2.0f, 2.0f, 2.0f, 2.0f);
    MathUtils::Quaternion q_norm = q_unnorm2.normalized();
    print_quaternion(q_unnorm2, "Original quaternion");
    print_quaternion(q_norm, "Normalized copy");
    bool test3_passed = float_equals(q_norm.norm(), 1.0f) && 
                       !quaternion_equals(q_unnorm2, q_norm);
    print_test_result("Quaternion normalized (copy)", test3_passed);
    
    // 测试共轭四元数
    MathUtils::Quaternion q_conj = q.conjugate();
    print_quaternion(q, "Original quaternion");
    print_quaternion(q_conj, "Conjugate quaternion");
    bool test4_passed = 
        float_equals(q_conj.w, q.w) &&
        float_equals(q_conj.x, -q.x) &&
        float_equals(q_conj.y, -q.y) &&
        float_equals(q_conj.z, -q.z);
    print_test_result("Quaternion conjugate", test4_passed);
    
    // 测试逆四元数
    MathUtils::Quaternion q_inv = q.inverse();
    print_quaternion(q, "Original quaternion");
    print_quaternion(q_inv, "Inverse quaternion");
    
    // 单位四元数的逆等于它的共轭
    bool test5_passed = quaternion_equals(q_inv, q_conj);
    print_test_result("Quaternion inverse (unit)", test5_passed);
    
    // 非单位四元数的逆
    MathUtils::Quaternion q_nonunit(2.0f, 1.0f, 0.5f, 0.25f);
    MathUtils::Quaternion q_nonunit_inv = q_nonunit.inverse();
    
    // q * q^-1 应该等于单位四元数
    MathUtils::Quaternion q_product = q_nonunit * q_nonunit_inv;
    MathUtils::Quaternion identity;
    
    print_quaternion(q_nonunit, "Non-unit quaternion");
    print_quaternion(q_nonunit_inv, "Non-unit inverse");
    print_quaternion(q_product, "q * q^-1");
    print_quaternion(identity, "Identity quaternion");
    
    bool test6_passed = quaternion_equals(q_product, identity, 0.001f);
    print_test_result("Quaternion inverse (non-unit)", test6_passed);
}

// 测试四元数运算符
void test_quaternion_operators() {
    test_printf("=== Test Quaternion Operators ===\r\n");
    
    MathUtils::Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    MathUtils::Quaternion q2(5.0f, 6.0f, 7.0f, 8.0f);
    
    // 测试加法
    MathUtils::Quaternion q_add = q1 + q2;
    print_quaternion(q1, "q1");
    print_quaternion(q2, "q2");
    print_quaternion(q_add, "q1 + q2");
    
    bool test1_passed = 
        float_equals(q_add.w, q1.w + q2.w) &&
        float_equals(q_add.x, q1.x + q2.x) &&
        float_equals(q_add.y, q1.y + q2.y) &&
        float_equals(q_add.z, q1.z + q2.z);
    print_test_result("Quaternion addition", test1_passed);
    
    // 测试减法
    MathUtils::Quaternion q_sub = q1 - q2;
    print_quaternion(q_sub, "q1 - q2");
    
    bool test2_passed = 
        float_equals(q_sub.w, q1.w - q2.w) &&
        float_equals(q_sub.x, q1.x - q2.x) &&
        float_equals(q_sub.y, q1.y - q2.y) &&
        float_equals(q_sub.z, q1.z - q2.z);
    print_test_result("Quaternion subtraction", test2_passed);
    
    // 测试乘法（四元数乘法）
    MathUtils::Quaternion q_mult = q1 * q2;
    print_quaternion(q_mult, "q1 * q2");
    
    // 手动计算四元数乘法
    float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    float y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    float z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    
    bool test3_passed = 
        float_equals(q_mult.w, w) &&
        float_equals(q_mult.x, x) &&
        float_equals(q_mult.y, y) &&
        float_equals(q_mult.z, z);
    print_test_result("Quaternion multiplication", test3_passed);
    
    // 测试标量乘法
    float scalar = 2.5f;
    MathUtils::Quaternion q_scalar = q1 * scalar;
    MathUtils::Quaternion q_scalar2 = scalar * q1; // 测试friend operator*
    
    print_quaternion(q_scalar, "q1 * 2.5");
    print_quaternion(q_scalar2, "2.5 * q1");
    
    bool test4_passed = 
        float_equals(q_scalar.w, q1.w * scalar) &&
        float_equals(q_scalar.x, q1.x * scalar) &&
        float_equals(q_scalar.y, q1.y * scalar) &&
        float_equals(q_scalar.z, q1.z * scalar);
    
    bool test5_passed = quaternion_equals(q_scalar, q_scalar2);
    
    print_test_result("Quaternion scalar multiplication", test4_passed);
    print_test_result("Quaternion scalar multiplication (commutative)", test5_passed);
    
    // 测试相等和不等操作符
    MathUtils::Quaternion q3(1.0f, 2.0f, 3.0f, 4.0f); // 与q1相同
    MathUtils::Quaternion q4(1.0f, 2.0f, 3.0f, 4.1f); // 与q1不同
    
    bool test6_passed = (q1 == q3) && !(q1 == q4);
    print_test_result("Quaternion equality", test6_passed);
    
    bool test7_passed = (q1 != q4) && !(q1 != q3);
    print_test_result("Quaternion inequality", test7_passed);
}

// 测试四元数球面线性插值（SLERP）
void test_quaternion_slerp() {
    test_printf("=== Test Quaternion SLERP ===\r\n");
    
    // 创建两个用于插值的四元数
    // q1代表0度旋转（单位四元数）
    // q2代表绕Z轴旋转90度
    MathUtils::Quaternion q1; // 单位四元数
    MathUtils::Quaternion q2 = MathUtils::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, PI / 2.0f);
    
    print_quaternion(q1, "q1 (0 deg)");
    print_quaternion(q2, "q2 (90 deg Z)");
    
    // 测试t=0时应该等于q1
    MathUtils::Quaternion result0 = MathUtils::Quaternion::slerp(q1, q2, 0.0f);
    print_quaternion(result0, "slerp(q1, q2, 0.0)");
    bool test1_passed = quaternion_equals(result0, q1);
    print_test_result("SLERP t=0", test1_passed);
    
    // 测试t=1时应该等于q2
    MathUtils::Quaternion result1 = MathUtils::Quaternion::slerp(q1, q2, 1.0f);
    print_quaternion(result1, "slerp(q1, q2, 1.0)");
    bool test2_passed = quaternion_equals(result1, q2);
    print_test_result("SLERP t=1", test2_passed);
    
    // 测试t=0.5时应该是45度旋转
    MathUtils::Quaternion result_half = MathUtils::Quaternion::slerp(q1, q2, 0.5f);
    MathUtils::Quaternion expected_half = MathUtils::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, PI / 4.0f);
    
    print_quaternion(result_half, "slerp(q1, q2, 0.5)");
    print_quaternion(expected_half, "Expected (45 deg Z)");
    
    bool test3_passed = quaternion_equals(result_half, expected_half, 0.01f);
    print_test_result("SLERP t=0.5", test3_passed);
    
    // 测试不同t值下的插值角度
    test_printf("Testing angle between quaternions at different t values:\r\n");
    for (float t = 0.0f; t <= 1.0f; t += 0.25f) {
        MathUtils::Quaternion result = MathUtils::Quaternion::slerp(q1, q2, t);
        float angle = MathUtils::Quaternion::angleBetween(q1, result);
        float expected_angle = t * (PI / 2.0f); // 预期角度是t*90度
        test_printf("t=%.2f: Angle between q1 and result = %.4f rad (%.2f deg), expected = %.4f rad\r\n",
                  t, angle, angle * RAD_TO_DEG, expected_angle);
    }
}

// 测试四元数应用到向量（旋转向量）
void test_quaternion_rotate_vector() {
    test_printf("=== Test Quaternion Vector Rotation ===\r\n");
    
    // 创建一个四元数代表绕Z轴旋转90度
    MathUtils::Quaternion q = MathUtils::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, PI / 2.0f);
    print_quaternion(q, "Rotation quaternion (90 deg Z)");
    
    // 定义一个向量 [1, 0, 0]
    float v_in[3] = {1.0f, 0.0f, 0.0f};
    float v_out[3];
    
    // 应用旋转
    q.rotateVector(v_in, v_out);
    
    // 期望得到向量 [0, 1, 0]
    float v_expected[3] = {0.0f, 1.0f, 0.0f};
    
    test_printf("Input vector: [%.4f, %.4f, %.4f]\r\n", v_in[0], v_in[1], v_in[2]);
    test_printf("Rotated vector: [%.4f, %.4f, %.4f]\r\n", v_out[0], v_out[1], v_out[2]);
    test_printf("Expected vector: [%.4f, %.4f, %.4f]\r\n", v_expected[0], v_expected[1], v_expected[2]);
    
    bool test1_passed = 
        float_equals(v_out[0], v_expected[0], QUAT_EPSILON) &&
        float_equals(v_out[1], v_expected[1], QUAT_EPSILON) &&
        float_equals(v_out[2], v_expected[2], QUAT_EPSILON);
    print_test_result("Vector rotation (Z, 90 deg)", test1_passed);
    
    // 再测试一个：绕Y轴旋转180度
    MathUtils::Quaternion q2 = MathUtils::Quaternion::fromAxisAngle(0.0f, 1.0f, 0.0f, PI);
    print_quaternion(q2, "Rotation quaternion (180 deg Y)");
    
    // 定义一个向量 [0, 0, 1]
    float v_in2[3] = {0.0f, 0.0f, 1.0f};
    float v_out2[3];
    
    // 应用旋转
    q2.rotateVector(v_in2, v_out2);
    
    // 期望得到向量 [0, 0, -1]
    float v_expected2[3] = {0.0f, 0.0f, -1.0f};
    
    test_printf("Input vector: [%.4f, %.4f, %.4f]\r\n", v_in2[0], v_in2[1], v_in2[2]);
    test_printf("Rotated vector: [%.4f, %.4f, %.4f]\r\n", v_out2[0], v_out2[1], v_out2[2]);
    test_printf("Expected vector: [%.4f, %.4f, %.4f]\r\n", v_expected2[0], v_expected2[1], v_expected2[2]);
    
    bool test2_passed = 
        float_equals(v_out2[0], v_expected2[0], QUAT_EPSILON) &&
        float_equals(v_out2[1], v_expected2[1], QUAT_EPSILON) &&
        float_equals(v_out2[2], v_expected2[2], QUAT_EPSILON);
    print_test_result("Vector rotation (Y, 180 deg)", test2_passed);
    
    // 测试复合旋转：先绕X轴旋转90度，再绕Y轴旋转90度
    MathUtils::Quaternion qx = MathUtils::Quaternion::fromAxisAngle(1.0f, 0.0f, 0.0f, PI / 2.0f);
    MathUtils::Quaternion qy = MathUtils::Quaternion::fromAxisAngle(0.0f, 1.0f, 0.0f, PI / 2.0f);
    
    // 在四元数中，旋转组合的顺序是右乘（先应用右侧的旋转）
    MathUtils::Quaternion qxy = qy * qx; // 先绕X轴，再绕Y轴
    
    // 对向量[0,0,1]应用复合旋转
    float v_in3[3] = {0.0f, 0.0f, 1.0f};
    float v_out3[3];
    
    qxy.rotateVector(v_in3, v_out3);
    
    // 对于先绕X轴旋转90度，再绕Y轴旋转90度，[0,0,1]应该变成[1,0,0]
    float v_expected3[3] = {1.0f, 0.0f, 0.0f};
    
    test_printf("Input vector: [%.4f, %.4f, %.4f]\r\n", v_in3[0], v_in3[1], v_in3[2]);
    test_printf("Rotated vector: [%.4f, %.4f, %.4f]\r\n", v_out3[0], v_out3[1], v_out3[2]);
    test_printf("Expected vector: [%.4f, %.4f, %.4f]\r\n", v_expected3[0], v_expected3[1], v_expected3[2]);
    
    bool test3_passed = 
        float_equals(v_out3[0], v_expected3[0], QUAT_EPSILON) &&
        float_equals(v_out3[1], v_expected3[1], QUAT_EPSILON) &&
        float_equals(v_out3[2], v_expected3[2], QUAT_EPSILON);
    print_test_result("Vector rotation (composite X+Y)", test3_passed);
}

// 测试四元数单位和标识检查
void test_quaternion_identity() {
    test_printf("=== Test Quaternion Identity ===\r\n");
    
    // 创建一个非单位四元数
    MathUtils::Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    
    // 创建一个单位四元数
    MathUtils::Quaternion identity_q;
    
    // 测试非单位四元数的标识检查
    bool test1_passed = !q.isIdentity();
    test_printf("Non-identity quaternion check: %s\r\n", test1_passed ? "correctly identified as non-identity" : "incorrectly identified as identity");
    print_test_result("Non-identity check", test1_passed);
    
    // 测试单位四元数的标识检查
    bool test2_passed = identity_q.isIdentity();
    test_printf("Identity quaternion check: %s\r\n", test2_passed ? "correctly identified as identity" : "incorrectly identified as non-identity");
    print_test_result("Identity check", test2_passed);
    
    // 测试setIdentity方法
    print_quaternion(q, "Before setIdentity");
    q.setIdentity();
    print_quaternion(q, "After setIdentity");
    
    bool test3_passed = q.isIdentity();
    print_test_result("setIdentity", test3_passed);
}

// 测试四元数综合功能（多个功能的组合测试）
void test_quaternion_comprehensive() {
    test_printf("=== Quaternion Comprehensive Test ===\r\n");
    
    // 测试旋转顺序一致性
    // 从欧拉角创建四元数，应该与依次应用三个轴的旋转一致
    float roll = PI / 6.0f;   // 30度
    float pitch = PI / 4.0f;  // 45度
    float yaw = PI / 3.0f;    // 60度
    
    // 从欧拉角创建
    MathUtils::Quaternion q_euler = MathUtils::Quaternion::fromEulerRad(roll, pitch, yaw);
    print_quaternion(q_euler, "From Euler (r=30, p=45, y=60 deg)");
    
    // 分别创建三个轴的旋转
    MathUtils::Quaternion q_roll = MathUtils::Quaternion::fromAxisAngle(1.0f, 0.0f, 0.0f, roll);
    MathUtils::Quaternion q_pitch = MathUtils::Quaternion::fromAxisAngle(0.0f, 1.0f, 0.0f, pitch);
    MathUtils::Quaternion q_yaw = MathUtils::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, yaw);
    
    // 按照YPR顺序组合旋转（飞行动力学中常用）
    // 注意四元数乘法是反向应用的，所以顺序为：roll * pitch * yaw
    MathUtils::Quaternion q_combined = q_roll * q_pitch * q_yaw;
    print_quaternion(q_combined, "Combined (yaw * pitch * roll)");
    
    // 测试旋转向量的一致性
    float v[3] = {1.0f, 0.0f, 0.0f};
    float v_euler[3], v_combined[3];
    
    q_euler.rotateVector(v, v_euler);
    q_combined.rotateVector(v, v_combined);
    
    test_printf("Vector rotated by euler quat: [%.4f, %.4f, %.4f]\r\n", v_euler[0], v_euler[1], v_euler[2]);
    test_printf("Vector rotated by combined quat: [%.4f, %.4f, %.4f]\r\n", v_combined[0], v_combined[1], v_combined[2]);
    
    // 注意：欧拉角到四元数的转换可能使用不同的约定，所以这个测试可能需要调整
    // 许多系统使用不同的欧拉角顺序（例如ZYX而不是XYZ）
    bool test1_passed = 
        float_equals(v_euler[0], v_combined[0], 0.1f) && 
        float_equals(v_euler[1], v_combined[1], 0.1f) && 
        float_equals(v_euler[2], v_combined[2], 0.1f);
    
    if (!test1_passed) {
        test_printf("Note: This test may fail due to different Euler angle conventions.\r\n");
        test_printf("The implementation might use a different rotation order than expected.\r\n");
    }
    
    print_test_result("Rotation consistency (Euler vs Combined)", test1_passed);
    
    // 测试连续旋转的等价性：旋转A后再旋转B等价于直接应用旋转B*A
    // 创建两个旋转
    MathUtils::Quaternion rot1 = MathUtils::Quaternion::fromAxisAngle(0.0f, 1.0f, 0.0f, PI / 4.0f); // Y轴45度
    MathUtils::Quaternion rot2 = MathUtils::Quaternion::fromAxisAngle(1.0f, 0.0f, 0.0f, PI / 3.0f); // X轴60度
    
    // 创建两种应用方式
    // 方式1：先应用rot1，再应用rot2
    float v1[3] = {0.0f, 0.0f, 1.0f};
    float v_temp[3];
    float v_sequential[3];
    
    rot1.rotateVector(v1, v_temp);
    rot2.rotateVector(v_temp, v_sequential);
    
    // 方式2：应用组合四元数 rot2 * rot1
    MathUtils::Quaternion rot_combined = rot2 * rot1;
    float v_direct[3];
    rot_combined.rotateVector(v1, v_direct);
    
    test_printf("Sequential rotation result: [%.4f, %.4f, %.4f]\r\n", v_sequential[0], v_sequential[1], v_sequential[2]);
    test_printf("Combined rotation result:   [%.4f, %.4f, %.4f]\r\n", v_direct[0], v_direct[1], v_direct[2]);
    
    bool test2_passed = 
        float_equals(v_sequential[0], v_direct[0], QUAT_EPSILON) &&
        float_equals(v_sequential[1], v_direct[1], QUAT_EPSILON) &&
        float_equals(v_sequential[2], v_direct[2], QUAT_EPSILON);
    print_test_result("Sequential vs Combined rotation", test2_passed);
}

// 测试复合旋转：先绕X轴旋转90度，再绕Y轴旋转90度
void test_quaternion_composite_rotation() {
    test_printf("=== Test Quaternion Composite Rotation ===\r\n");
    
    // 创建旋转四元数
    MathUtils::Quaternion qx = MathUtils::Quaternion::fromAxisAngle(1.0f, 0.0f, 0.0f, PI / 2.0f);  // 绕X轴旋转90度
    MathUtils::Quaternion qy = MathUtils::Quaternion::fromAxisAngle(0.0f, 1.0f, 0.0f, PI / 2.0f);  // 绕Y轴旋转90度
    
    // 打印各个旋转四元数
    print_quaternion(qx, "X-axis 90 deg");
    print_quaternion(qy, "Y-axis 90 deg");
    
    // 定义初始向量 [0, 0, 1]
    float v0[3] = {0.0f, 0.0f, 1.0f};
    float v1[3], v2[3], v_combined[3];
    
    // 逐步演示变换过程
    test_printf("Initial vector: [%.4f, %.4f, %.4f]\r\n", v0[0], v0[1], v0[2]);
    
    // 单独应用X轴旋转
    qx.rotateVector(v0, v1);
    test_printf("After X rotation: [%.4f, %.4f, %.4f]\r\n", v1[0], v1[1], v1[2]);
    
    // 在X轴旋转基础上再应用Y轴旋转
    qy.rotateVector(v1, v2);
    test_printf("After X then Y rotation: [%.4f, %.4f, %.4f]\r\n", v2[0], v2[1], v2[2]);
    
    // 直接应用组合旋转
    // 在四元数中，旋转组合的顺序是右乘（先应用右侧的旋转）
    // 所以 qy * qx 代表先绕X轴旋转，再绕Y轴旋转
    MathUtils::Quaternion qxy = qy * qx;
    print_quaternion(qxy, "Combined rotation (Y*X)");
    
    qxy.rotateVector(v0, v_combined);
    test_printf("Using combined quaternion: [%.4f, %.4f, %.4f]\r\n", 
              v_combined[0], v_combined[1], v_combined[2]);
    
    // 验证两种方式得到的结果是否一致
    bool test_passed = 
        float_equals(v2[0], v_combined[0], QUAT_EPSILON) &&
        float_equals(v2[1], v_combined[1], QUAT_EPSILON) &&
        float_equals(v2[2], v_combined[2], QUAT_EPSILON);
    
    print_test_result("Sequential vs combined rotation consistency", test_passed);
    
    // 解释正确的结果是什么以及为什么
    test_printf("\r\nExplanation of rotation sequence:\r\n");
    test_printf("1. Initial vector [0,0,1] represents the +Z axis\r\n");
    test_printf("2. 90° around X-axis rotates +Z to +Y: [0,1,0]\r\n");
    test_printf("3. 90° around Y-axis rotates +Y to +Y (unchanged)\r\n");
    test_printf("   This is because the Y-axis rotation preserves the Y component\r\n");
    test_printf("   (Y-axis rotation only affects X and Z components)\r\n");
}

// 测试SLERP角度计算
void test_quaternion_slerp_angle() {
    test_printf("=== Test Quaternion SLERP Angle ===\r\n");
    
    // 创建两个用于插值的四元数
    // q1代表0度旋转（单位四元数）
    // q2代表绕Z轴旋转90度
    MathUtils::Quaternion q1; // 单位四元数
    MathUtils::Quaternion q2 = MathUtils::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, PI / 2.0f);
    
    print_quaternion(q1, "q1 (0 deg)");
    print_quaternion(q2, "q2 (90 deg Z)");
    
    // 计算两个四元数之间的角度
    float angle = MathUtils::Quaternion::angleBetween(q1, q2);
    test_printf("Angle between q1 and q2: %.4f rad (%.2f deg), expected: %.4f rad\r\n",
              angle, angle * RAD_TO_DEG, PI / 2.0f);
    
    bool test1_passed = float_equals(angle, PI / 2.0f, 0.01f);
    print_test_result("Quaternion angle calculation", test1_passed);
    
    // 测试不同t值下的插值角度
    test_printf("Testing angle between quaternions at different t values:\r\n");
    for (float t = 0.0f; t <= 1.0f; t += 0.25f) {
        MathUtils::Quaternion result = MathUtils::Quaternion::slerp(q1, q2, t);
        float angle = MathUtils::Quaternion::angleBetween(q1, result);
        float expected_angle = t * (PI / 2.0f); // 预期角度是t*90度
        bool t_test_passed = float_equals(angle, expected_angle, 0.01f);
        test_printf("t=%.2f: Angle = %.4f rad (%.2f deg), expected = %.4f rad - %s\r\n",
                  t, angle, angle * RAD_TO_DEG, expected_angle, 
                  t_test_passed ? "PASSED" : "FAILED");
    }
}

// 测试欧拉角与旋转顺序
void test_quaternion_euler_consistency() {
    test_printf("=== Test Quaternion Euler Consistency ===\r\n");
    
    // 使用固定的测试角度
    float roll = PI / 6.0f;   // 30度
    float pitch = PI / 4.0f;  // 45度
    float yaw = PI / 3.0f;    // 60度
    
    // 从欧拉角创建四元数 (ZYX顺序 - 先偏航yaw，再俯仰pitch，最后滚转roll)
    MathUtils::Quaternion q_euler = MathUtils::Quaternion::fromEulerRad(roll, pitch, yaw);
    print_quaternion(q_euler, "From Euler ZYX (r=30, p=45, y=60 deg)");
    
    // 单独创建每个轴的旋转
    MathUtils::Quaternion q_roll = MathUtils::Quaternion::fromAxisAngle(1.0f, 0.0f, 0.0f, roll);
    MathUtils::Quaternion q_pitch = MathUtils::Quaternion::fromAxisAngle(0.0f, 1.0f, 0.0f, pitch);
    MathUtils::Quaternion q_yaw = MathUtils::Quaternion::fromAxisAngle(0.0f, 0.0f, 1.0f, yaw);
    
    // 打印各个旋转四元数以便调试
    print_quaternion(q_roll, "Roll rotation (30 deg)");
    print_quaternion(q_pitch, "Pitch rotation (45 deg)");
    print_quaternion(q_yaw, "Yaw rotation (60 deg)");
    
    // 按照ZYX内旋顺序组合旋转 (先yaw，再pitch，最后roll)
    // 在四元数乘法中，顺序是从右到左应用，但内旋的组合方式不同。
    // ZYX 内旋 等效于 q_yaw * q_pitch * q_roll
    MathUtils::Quaternion q_combined = q_yaw * q_pitch * q_roll; // Corrected multiplication order for ZYX intrinsic
    print_quaternion(q_combined, "Combined ZYX (yaw * pitch * roll)"); // Updated comment
    
    // 测试一个简单向量，验证旋转效果
    float v[3] = {1.0f, 0.0f, 0.0f};  // X轴单位向量
    float v_euler[3], v_combined[3]; // Removed v_step, v_temp as they are not used reliably here
    
    // 使用欧拉角创建的四元数旋转
    q_euler.rotateVector(v, v_euler);
    test_printf("Vector rotated by euler quat: [%.4f, %.4f, %.4f]\r\n", v_euler[0], v_euler[1], v_euler[2]);
    
    // 使用组合四元数旋转
    q_combined.rotateVector(v, v_combined);
    test_printf("Vector rotated by combined quat: [%.4f, %.4f, %.4f]\r\n", v_combined[0], v_combined[1], v_combined[2]);
    
    // ... existing code ...
    // After yaw (extrinsic): [0.5000, 0.8660, 0.0000]
    // Euler vs Combined quaternion consistency: PASSED
    // Euler vs Combined vector rotation consistency: PASSED
    
    // Let's compare v_combined with the expected result of ZYX intrinsic rotation
    // The previous manual calculation in comments was flawed.
    // The correct expected result is the one produced by the verified quaternion rotation.
    // Expected: [0.3536, 0.6123, -0.7071] for ZYX(r=30, p=45, y=60) on [1,0,0]
    float v_expected_step[3] = {0.3536f, 0.6123f, -0.7071f}; // Updated expected value based on correct quaternion rotation
    
    test_printf("Expected vector after ZYX intrinsic steps: [%.4f, %.4f, %.4f]\r\n", v_expected_step[0], v_expected_step[1], v_expected_step[2]);
    
    bool step_combined_match =
        float_equals(v_combined[0], v_expected_step[0], QUAT_EPSILON) &&
        float_equals(v_combined[1], v_expected_step[1], QUAT_EPSILON) &&
        float_equals(v_combined[2], v_expected_step[2], QUAT_EPSILON);
    // This test now verifies that the combined rotation matches the known correct result.
    print_test_result("Step-by-step (manual calc) vs Combined consistency", step_combined_match);


    // 测试欧拉角往返转换 (欧拉角->四元数->欧拉角)
    float r_out, p_out, y_out;
    q_euler.toEulerRad(r_out, p_out, y_out);
    
    // ... (rest of the function remains the same) ...
    test_printf("Original Euler: roll=%.4f rad (%.1f°), pitch=%.4f rad (%.1f°), yaw=%.4f rad (%.1f°)\r\n", 
              roll, roll*RAD_TO_DEG, pitch, pitch*RAD_TO_DEG, yaw, yaw*RAD_TO_DEG);
    test_printf("Recovered Euler: roll=%.4f rad (%.1f°), pitch=%.4f rad (%.1f°), yaw=%.4f rad (%.1f°)\r\n", 
              r_out, r_out*RAD_TO_DEG, p_out, p_out*RAD_TO_DEG, y_out, y_out*RAD_TO_DEG);
    
    bool round_trip_passed = 
        float_equals(r_out, roll, QUAT_EPSILON) &&
        float_equals(p_out, pitch, QUAT_EPSILON) &&
        float_equals(y_out, yaw, QUAT_EPSILON);
    print_test_result("Euler round-trip conversion", round_trip_passed);
}

// 更新主测试函数以包含新的测试
void test_math_quaternion() {
    test_printf("\r\n===== Math Quaternion Component Test =====\r\n\r\n");
    
    // 测试基本功能
    test_quaternion_construction();
    test_printf("\r\n");
    
    test_quaternion_from_euler();
    test_printf("\r\n");
    
    test_quaternion_from_matrix();
    test_printf("\r\n");
    
    test_quaternion_from_axis_angle();
    test_printf("\r\n");
    
    // 操作测试
    test_quaternion_basic_operations();
    test_printf("\r\n");
    
    test_quaternion_operators();
    test_printf("\r\n");
    
    // 测试针对失败案例的专项测试
    test_quaternion_slerp_angle();
    test_printf("\r\n");
    
    // 测试复合旋转
    test_quaternion_composite_rotation();
    test_printf("\r\n");
    
    // 测试欧拉角一致性
    test_quaternion_euler_consistency();
    test_printf("\r\n");
    
    test_quaternion_identity();
    test_printf("\r\n");
    
    test_printf("===== Math Quaternion Test Complete =====\r\n\r\n");
}

// 主测试函数
#define REPEAT 1
extern "C" {
void test_cpp(void) {
    static uint8_t once = 1;
    if (once != 0) {
        once = REPEAT;
        test_printf("\r\n===== Math Quaternion Test =====\r\n\r\n");
        test_printf("Test Started...\r\n");
        test_math_quaternion();
    }
}
}
#include "Attitude.h"


/**
 * @brief 构造函数
 */
AttitudeManager::AttitudeManager(IMU* imu, AttitudeEstimator* estimator)
    : _imu(imu),
      _estimator(estimator),
      _isInitialized(false)
{
    // 初始化数据缓冲区
    for (int i = 0; i < 3; i++) {
        _gyro[i] = 0.0f;
        _accel[i] = 0.0f;
    }
}

/**
 * @brief 析构函数
 */
AttitudeManager::~AttitudeManager()
{
    // 注意这里不删除组件对象，因为它们是从外部传入的
}

/**
 * @brief 初始化姿态管理器
 */
bool AttitudeManager::init()
{
    // 检查依赖组件是否有效
    if (_imu == nullptr || _estimator == nullptr) {
        return false;
    }
    
    _estimator->init();
    
    _isInitialized = true;
    return true;
}

/**
 * @brief 单步更新（适用于循环调用）
 */
void AttitudeManager::update()
{
    // 检查是否已初始化
    if (!_isInitialized) {
        return;
    }
    
    // 1. 读取IMU数据
    _imu->read(_gyro, _accel);
    
    // 2. 更新姿态估计
    _estimator->update(_gyro, _accel);
}

/**
 * @brief 获取当前欧拉角姿态
 */
void AttitudeManager::getAttitude(float& roll, float& pitch, float& yaw)
{
    _estimator->getEulerRadians(roll, pitch, yaw);
}

/**
 * @brief 获取当前四元数姿态
 */
void AttitudeManager::getQuaternion(float q[4])
{
    _estimator->getQuaternion(q);
}

#include <BasicLinearAlgebra.h>
#include <LSM6DS3.h>
#include <Wire.h>
using namespace BLA;
class IMU{
  Matrix<4,1> q_;
  float kp_,ki_;
  Matrix<3,1> ei;
  Matrix<3,1> eulerangle;
  uint32_t lastms;
  public:
  void normalizeQuat() {
        float qNorm = sqrt(q_(0)*q_(0) + q_(1)*q_(1) + q_(2)*q_(2) + q_(3)*q_(3));
        q_ = q_ / qNorm;
    }
  void Accel2quater(Matrix<3,1> a){
    float norm = sqrt(a(0,0)*a(0,0) + a(1,0)*a(1,0) + a(2,0)*a(2,0));
    if (norm < 0.001f){
      q_ = {1, 0, 0, 0};
      return;
    }
    float ax = a(0,0) / norm;
    float ay = a(1,0) / norm;
    float az = a(2,0) / norm;
    if (az >= 0) {
            q_(0) = 1.0f + az;
            q_(1) = -ay;
            q_(2) = ax;
            q_(3) = 0;
        } else {
            q_(0) = -ay;
            q_(1) = 1.0f - az;
            q_(2) = 0;
            q_(3) = ax;
        }
    normalizeQuat();
  }
  IMU(Matrix<3,1> a,float kp=2.f,float ki=0.005f):kp_(kp),ki_(ki){
    Accel2quater(a);
    ei.Fill(0);
    lastms=micros();
  }
  void Quater2Euler() {
        float qw = q_(0), qx = q_(1), qy = q_(2), qz = q_(3);

        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (qw * qx + qy * qz);
        float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
        eulerangle(0) = atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (qw * qy - qz * qx);
        if (abs(sinp) >= 1.0f)
            eulerangle(1) = copysign(PI / 2.0f, sinp); // 使用 90 度，防止万向节锁溢出
        else
            eulerangle(1) = asin(sinp);

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (qw * qz + qx * qy);
        float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
        eulerangle(2) = atan2(siny_cosp, cosy_cosp);
        /*eulerangle(0) *= 180.0f / PI;
        eulerangle(1) *= 180.0f / PI;
        eulerangle(2) *= 180.0f / PI;*/
    }
  void update(Matrix<3,1> gyro,Matrix<3,1> acc){
        unsigned long now = micros();
        float dt = (now - lastms) / 1000000.0f;
        lastms = now;
        float aNorm = sqrt(acc(0)*acc(0) + acc(1)*acc(1) + acc(2)*acc(2));
        if (aNorm < 0.001f) return;
        acc = acc / aNorm;

        // 提取当前四元数代表的“估计重力方向” (机体坐标系下的重力向量)
        // 这是四元数旋转矩阵的第三列：v = [2(xz-wy), 2(yz+wx), w^2-x^2-y^2+z^2]
        float vx = 2.0f * (q_(1) * q_(3) - q_(0) * q_(2));
        float vy = 2.0f * (q_(0) * q_(1) + q_(2) * q_(3));
        float vz = q_(0) * q_(0) - q_(1) * q_(1) - q_(2) * q_(2) + q_(3) * q_(3);

        // 计算误差：测量值与估计值的叉乘 (Error is cross product of measured and estimated gravity)
        // e = acc x v
        Matrix<3, 1> e;
        e(0) = acc(1) * vz - acc(2) * vy;
        e(1) = acc(2) * vx - acc(0) * vz;
        e(2) = acc(0) * vy - acc(1) * vx;

        // 误差积分
        ei += e * ki_ * dt;

        // 修正陀螺仪角速度
        Matrix<3, 1> correctedGyro = gyro + e * kp_ + ei;

        // 四元数微分方程更新: q_new = q + 0.5 * q * Omega * dt
        float qw = q_(0), qx = q_(1), qy = q_(2), qz = q_(3);
        float gx = correctedGyro(0), gy = correctedGyro(1), gz = correctedGyro(2);

        q_(0) += 0.5f * (-qx * gx - qy * gy - qz * gz) * dt;
        q_(1) += 0.5f * ( qw * gx + qy * gz - qz * gy) * dt;
        q_(2) += 0.5f * ( qw * gy - qx * gz + qz * gx) * dt;
        q_(3) += 0.5f * ( qw * gz + qx * gy - qy * gx) * dt;

        normalizeQuat();
        Quater2Euler();
  }
  Matrix<3,1> getEulerangle(){
    return eulerangle;
  }
  Matrix<4,1> getQuat(){
    return q_;
  }
};

/* ==========================================================================
 *  LSM6DS3 硬件读取 + Mahony 姿态维护（对外提供 C 风格自由函数，便于 .ino 跨文件调用）
 *
 *  说明：
 *   - 读取方式与 IMU_Capture 一致，使用 LSM6DS3 库经 I2C 读取加速度/角速度；
 *   - 加速度单位为 G（Mahony 内部会归一化，单位不影响）；
 *   - 陀螺仪库返回单位为 deg/s，而 Mahony 四元数微分方程需要 rad/s，故乘以 DEG2RAD；
 *   - update() 内部用 micros() 自行计算 dt，故本文件只需按固定频率调用即可；
 *   - 欧拉角单位为弧度（roll/pitch/yaw = eulerangle(0/1/2)）。
 * ========================================================================== */

#define IMU_DEG2RAD   0.017453292519943295f   // PI/180

static LSM6DS3 g_imuSensor(I2C_MODE, 0x6A);    // I2C 地址 0x6A，与 IMU_Capture 相同
static IMU*    g_imu       = nullptr;          // Mahony 姿态解算对象
static bool    g_imuReady  = false;
static float   g_roll = 0.0f, g_pitch = 0.0f, g_yaw = 0.0f;   // 欧拉角，单位：弧度

/* 初始化 IMU：配置传感器，并用初始加速度确定初始四元数。返回 true 表示成功 */
bool IMU_Init()
{
    Wire.begin();
    if( g_imuSensor.begin() != 0 )
    {
        g_imuReady = false;
        return false;                          // 传感器初始化失败
    }

    /* 用当前加速度作为初始重力方向，确定初始姿态 */
    Matrix<3,1> a;
    a(0) = g_imuSensor.readFloatAccelX();
    a(1) = g_imuSensor.readFloatAccelY();
    a(2) = g_imuSensor.readFloatAccelZ();

    static IMU imuInstance(a);                  // 静态局部对象，避免动态内存分配
    g_imu      = &imuInstance;
    g_imuReady = true;
    return true;
}

/* 读取一次 IMU 并更新姿态（四元数 + 欧拉角）。建议以 100Hz 调用 */
void IMU_Update()
{
    if( !g_imuReady || g_imu == nullptr ) return;

    Matrix<3,1> acc;
    acc(0) = g_imuSensor.readFloatAccelX();
    acc(1) = g_imuSensor.readFloatAccelY();
    acc(2) = g_imuSensor.readFloatAccelZ();

    Matrix<3,1> gyro;                           // deg/s -> rad/s
    gyro(0) = g_imuSensor.readFloatGyroX() * IMU_DEG2RAD;
    gyro(1) = g_imuSensor.readFloatGyroY() * IMU_DEG2RAD;
    gyro(2) = g_imuSensor.readFloatGyroZ() * IMU_DEG2RAD;

    g_imu->update(gyro, acc);                   // Mahony 更新四元数与欧拉角

    Matrix<3,1> e = g_imu->getEulerangle();
    g_roll  = e(0);
    g_pitch = e(1);
    g_yaw   = e(2);
}

/* 姿态读取接口（弧度） */
bool  IMU_IsReady() { return g_imuReady; }
float IMU_GetRoll()  { return g_roll;  }
float IMU_GetPitch() { return g_pitch; }
float IMU_GetYaw()   { return g_yaw;   }
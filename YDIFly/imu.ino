#include <BasicLinearAlgebra.h>
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
        float dt = (now - lastMicros) / 1000000.0f;
        lastMicros = now;

        // 2. 归一化加速度计测量值
        float aNorm = sqrt(acc(0)*acc(0) + acc(1)*acc(1) + acc(2)*acc(2));
        if (aNorm < 0.001f) return;
        acc = acc / aNorm;

        // 3. 提取当前四元数代表的“估计重力方向” (机体坐标系下的重力向量)
        // 这是四元数旋转矩阵的第三列：v = [2(xz-wy), 2(yz+wx), w^2-x^2-y^2+z^2]
        float vx = 2.0f * (q_(1) * q_(3) - q_(0) * q_(2));
        float vy = 2.0f * (q_(0) * q_(1) + q_(2) * q_(3));
        float vz = q_(0) * q_(0) - q_(1) * q_(1) - q_(2) * q_(2) + q_(3) * q_(3);

        // 4. 计算误差：测量值与估计值的叉乘 (Error is cross product of measured and estimated gravity)
        // e = acc x v
        Matrix<3, 1> e;
        e(0) = acc(1) * vz - acc(2) * vy;
        e(1) = acc(2) * vx - acc(0) * vz;
        e(2) = acc(0) * vy - acc(1) * vx;

        // 5. 误差积分
        eInt_ += e * ki_ * dt;

        // 6. 修正陀螺仪角速度
        Matrix<3, 1> correctedGyro = gyro + e * kp_ + eInt_;

        // 7. 四元数微分方程更新: q_new = q + 0.5 * q * Omega * dt
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
}
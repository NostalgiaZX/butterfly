/**
 * @file    : pid.ino
 * @brief   : 通用PID控制器类
 *            功能：基础PID、输出限幅、积分抗饱和、微分滤波、
 *                  测量值微分（避免给定值突变引发微分冲击）、前馈补偿
 */

class PID {
private:
    float kp_, ki_, kd_;
    float setpoint_;
    float output_min_, output_max_;
    float integral_;
    float last_measurement_;
    float last_error_;
    float last_d_term_;
    float d_filter_alpha_;     // 微分低通滤波系数，1.0=不滤波，趋近0=强滤波
    float feedforward_;
    bool  d_on_measurement_;   // true：对测量值求导（推荐），false：对误差求导
    bool  initialized_;

public:
    /**
     * @param kp             比例系数
     * @param ki             积分系数
     * @param kd             微分系数
     * @param out_min        输出下限
     * @param out_max        输出上限
     * @param d_filter_alpha 微分低通滤波系数 (0~1]，默认1.0不滤波
     * @param d_on_meas      微分作用于测量值（true，推荐）或误差（false）
     */
    PID(float kp, float ki, float kd,
        float out_min, float out_max,
        float d_filter_alpha = 1.0f,
        bool  d_on_meas      = true)
        : kp_(kp), ki_(ki), kd_(kd),
          output_min_(out_min), output_max_(out_max),
          d_filter_alpha_(d_filter_alpha), d_on_measurement_(d_on_meas),
          setpoint_(0.0f), integral_(0.0f),
          last_measurement_(0.0f), last_error_(0.0f), last_d_term_(0.0f),
          feedforward_(0.0f), initialized_(false) {}

    /* ---------- 参数设置 ---------- */

    void setSetpoint(float sp)                        { setpoint_ = sp; }
    void setGains(float kp, float ki, float kd)       { kp_ = kp; ki_ = ki; kd_ = kd; }
    void setOutputLimits(float min_v, float max_v)    { output_min_ = min_v; output_max_ = max_v; }
    void setFeedforward(float ff)                     { feedforward_ = ff; }
    void setDerivativeFilter(float alpha)             { d_filter_alpha_ = constrain(alpha, 1e-4f, 1.0f); }
    void setDerivativeOnMeasurement(bool enable)      { d_on_measurement_ = enable; }

    /* ---------- 状态读取 ---------- */

    float getSetpoint()  const { return setpoint_; }
    float getIntegral()  const { return integral_; }
    float getKp()        const { return kp_; }
    float getKi()        const { return ki_; }
    float getKd()        const { return kd_; }

    /* ---------- 重置 ---------- */

    void reset() {
        integral_    = 0.0f;
        last_d_term_ = 0.0f;
        initialized_ = false;
    }

    /* ---------- 核心计算 ---------- */

    /**
     * @param measurement 当前测量值
     * @param dt          距上次调用的时间间隔，单位：秒
     * @return            PID输出（已限幅）
     */
    float compute(float measurement, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint_ - measurement;

        // P 项
        float p_term = kp_ * error;

        // I 项 + 积分抗饱和（截断式限幅）
        integral_ += ki_ * error * dt;
        integral_  = constrain(integral_, output_min_, output_max_);

        // D 项 + 低通滤波
        float d_term = 0.0f;
        if (initialized_) {
            float raw_d;
            if (d_on_measurement_) {
                // 对测量值求导，避免给定值阶跃时产生微分冲击
                raw_d = -kd_ * (measurement - last_measurement_) / dt;
            } else {
                raw_d = kd_ * (error - last_error_) / dt;
            }
            d_term       = d_filter_alpha_ * raw_d + (1.0f - d_filter_alpha_) * last_d_term_;
            last_d_term_ = d_term;
        }

        last_measurement_ = measurement;
        last_error_       = error;
        initialized_      = true;

        return constrain(p_term + integral_ + d_term + feedforward_, output_min_, output_max_);
    }
};

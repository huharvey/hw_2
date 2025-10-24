#include "pid.hpp"
#include <cmath> // For std::fabs

Pid::Pid(const PidParams &params) : params_(params), integral_(0.0f), last_error_(0.0f) {
}

void Pid::reset() {
    integral_ = 0.0f;
    last_error_ = 0.0f;
}

float Pid::pidCalc(const float ref, const float fdb) {
    float error = ref - fdb;

    // kp
    float p_out = params_.kp * error;

    // ki
    integral_ += error;
    // 防止过饱和
    if (integral_ > params_.integral_max) {
        integral_ = params_.integral_max;
    } else if (integral_ < -params_.integral_max) {
        integral_ = -params_.integral_max;
    }

    float i_out = params_.ki * integral_;

    // kd
    float derivative = error - last_error_;
    float d_out = params_.kd * derivative;


    float output = p_out + i_out + d_out;

    // --- 输出限幅 ---
    if (output > params_.output_max) {
        output = params_.output_max;
    } else if (output < -params_.output_max) {
        output = -params_.output_max;
    }

    last_error_ = error;

    return output;
}
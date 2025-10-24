#ifndef PID_H
#define PID_H

struct PidParams {
    float kp;
    float ki;
    float kd;
    float output_max;   
    float integral_max; 
};

class Pid {
public:
    // 初始化PID
    Pid(const PidParams &params);
    ~Pid() = default;

    // 计算PID输出
    float pidCalc(const float ref, const float fdb);

    // 重置积分项和上一次误差
    void reset();

private:
    PidParams params_; 

    float integral_;   
    float last_error_; 
};

#endif // PID_H
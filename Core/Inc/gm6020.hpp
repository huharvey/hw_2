#ifndef GM6020_H
#define GM6020_H

#include <cstdint>
#include "HW_fdcan.hpp"


#define ENCODER_RESOLUTION 8192.0f // 编码器分辨率
#define PI 3.1415926535f

extern FDCAN_HandleTypeDef hfdcan1;

class GM6020 {
public:

    GM6020(uint8_t id);
    ~GM6020() = default;

    // 在CAN接收中断回调函数中调用此方法来更新电机状态
    // 它会检查收到的CAN ID是否是这个电机的反馈ID
    void decode(uint32_t can_id, uint8_t data[8]);
    void encode(int16_t cmd1, int16_t cmd2, int16_t cmd3, int16_t cmd4);

    // 获取电机数据的函数
    float getAngleRad();  
    float getVelocityRPM(); // 转速

private:
    uint8_t id_;
    uint32_t feedback_id_; // 电机期望的反馈CAN ID
    
    int16_t raw_angle_;
    int16_t raw_velocity_;
    int16_t raw_current_;
    uint8_t temperature_;
    
    float angle_rad_;
    float velocity_rpm_;
};

#endif // GM6020_H
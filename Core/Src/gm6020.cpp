/*
 * @Author: huharvey huharveyy@gmail.com
 * @Date: 2025-10-20 17:44:07
 * @LastEditors: huharvey huharveyy@gmail.com
 * @LastEditTime: 2025-10-20 19:15:13
 * @FilePath: \hw_2\Core\Src\gm6020.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "gm6020.hpp"

GM6020::GM6020(uint8_t id) : id_(id) {
    feedback_id_ = 0x204 + id_;
}
void GM6020::encode(int16_t cmd1, int16_t cmd2, int16_t cmd3, int16_t cmd4) {
    uint8_t tx_data[8];

    tx_data[0] = (cmd1 >> 8) & 0xFF;
    tx_data[1] = cmd1 & 0xFF;
    tx_data[2] = (cmd2 >> 8) & 0xFF;
    tx_data[3] = cmd2 & 0xFF;
    tx_data[4] = (cmd3 >> 8) & 0xFF;
    tx_data[5] = cmd3 & 0xFF;
    tx_data[6] = (cmd4 >> 8) & 0xFF;
    tx_data[7] = cmd4 & 0xFF;

    FdcanSendMsg(&hfdcan1, tx_data, 0x1FE, 8); 
}

void GM6020::decode(uint32_t can_id, uint8_t data[8]) {

    if (can_id != feedback_id_) {
        return;
    }

    raw_angle_    = (int16_t)((data[0] << 8) | data[1]); 
    raw_velocity_ = (int16_t)((data[2] << 8) | data[3]); 
    raw_current_  = (int16_t)((data[4] << 8) | data[5]); 
    temperature_  = data[6]; 

    angle_rad_ = (raw_angle_ / ENCODER_RESOLUTION) * (2.0f * PI);
    velocity_rpm_ = raw_velocity_; //电机转速
}

float GM6020::getAngleRad() {
    return angle_rad_;
}


float GM6020::getVelocityRPM() {
    return velocity_rpm_;
}
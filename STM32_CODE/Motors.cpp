/*
 * Motors.cpp
 *
 *  Created on: Oct 13, 2025
 *      Author: neilp
 */

#include "Motors.hpp"


Motors::Motors(TIM_HandleTypeDef* tim, Channels ch)
	: tim_(tim), ch_(ch) {}


void Motors::resetIntegral() {
    integral = 0.0f;
}


void Motors::setLeft(uint8_t speed){
    __HAL_TIM_SET_COMPARE(tim_, ch_.left_in1, speed);
    __HAL_TIM_SET_COMPARE(tim_, ch_.left_in2, 0); // always forward
}

void Motors::setRight(uint8_t speed){
    __HAL_TIM_SET_COMPARE(tim_, ch_.right_in1, speed); //
    __HAL_TIM_SET_COMPARE(tim_, ch_.right_in2, 0); //
}

void Motors::stop(){
    __HAL_TIM_SET_COMPARE(tim_, ch_.left_in1, 0);
    __HAL_TIM_SET_COMPARE(tim_, ch_.left_in2, 0);
    __HAL_TIM_SET_COMPARE(tim_, ch_.right_in1, 0);
    __HAL_TIM_SET_COMPARE(tim_, ch_.right_in2, 0);
}

int16_t Motors::pidController(float kp, float ki, float kd, int16_t error, int16_t previous_error, float dt) {
    float P_out = kp * error;
    integral += error * dt;
    float I_out = ki * integral;
    float D_out = kd * (error - previous_error) / dt;

    float output = P_out + I_out + D_out;
    return static_cast<int16_t>(output);
}

/*
 * sensors.cpp
 *  Created on: Oct 19, 2025
 *      Author: neilp
 */

#include "sensors.h"


sensors::sensors(const sensors::Inputs &inputs)
	: _inputs(inputs) {}
sensors::~sensors() = default;

uint8_t sensors::getValues()
{
	uint8_t mask = 0;
	mask |= (uint8_t)(HAL_GPIO_ReadPin(GPIOA, _inputs.s0) == GPIO_PIN_SET) << 0;
	mask |= (uint8_t)(HAL_GPIO_ReadPin(GPIOA, _inputs.s1) == GPIO_PIN_SET) << 1;
	mask |= (uint8_t)(HAL_GPIO_ReadPin(GPIOA, _inputs.s2) == GPIO_PIN_SET) << 2;
	mask |= (uint8_t)(HAL_GPIO_ReadPin(GPIOA, _inputs.s3) == GPIO_PIN_SET) << 3;
	mask |= (uint8_t)(HAL_GPIO_ReadPin(GPIOA, _inputs.s4) == GPIO_PIN_SET) << 4;
	mask |= (uint8_t)(HAL_GPIO_ReadPin(GPIOA, _inputs.s5) == GPIO_PIN_SET) << 5;
	return mask;
}

int16_t sensors::pidController(float kp, float ki, float kd, int16_t error, int16_t previous_error, float dt) {
    float P_out = kp * error;
    integral += error * dt;
    float I_out = ki * integral;
    float D_out = kd * (error - previous_error) / dt;

    float output = P_out + I_out + D_out;
    return static_cast<int16_t>(output);
}


int16_t sensors::getShiftErr(const std::bitset<8>& sensorBits) {
    // Map sensors 0..5 → weights [-5, -3, -1, +1, +3, +5]
    // => negative = line is left, positive = line is right, 0 = centered.
    constexpr int kScale = 5;  // tune: error units ≈ [-500..+500]
    int total = 0;
    int count = 0;

    for (int i = 0; i < 6; ++i) {
        if (sensorBits.test(i)) {
            total += (i * 2 - 5);  // -5,-3,-1, +1,+3,+5
            ++count;
        }
    }

    // Keep a little memory to bias "search" when line is temporarily lost.
    static int16_t lastErr = 0;

    if (count == 0) {
        // No sensors see the line. Nudge in the last known direction
        // (helps with recovery instead of freezing at zero).
        const int lost_bias = 6 * 0;  // *kScale tune: small push, matches max weight (~±6)
        return (lastErr == 0) ? 0 : (lastErr > 0 ? +lost_bias : -lost_bias);
    }

    // Average of active weights, scaled for PID resolution.
    int32_t err = static_cast<int32_t>(total) * kScale / count;

    if (err > INT16_MAX) err = INT16_MAX;
    if (err < INT16_MIN) err = INT16_MIN;

    lastErr = static_cast<int16_t>(err);
    return lastErr;
}




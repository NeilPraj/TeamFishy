/*
 * Motors.h
 *
 *  Created on: Oct 13, 2025
 *      Author: neilp
 */

#ifndef SRC_MOTORS_H_
#define SRC_MOTORS_H_


#include <cstdint>
#include <algorithm>
#include "main.h"


class Motors {
public:
	struct Channels {
		uint32_t left_in1;
		uint32_t left_in2;
		uint32_t right_in1;
		uint32_t right_in2;
	};


	Motors(TIM_HandleTypeDef* tim, Channels ch);
	~Motors() = default;

	void setLeft(uint8_t speed);

	void setRight(uint8_t speed);

	void stop();

	int16_t pidController(float kp, float ki, float kd, int16_t error, int16_t previous_error, float dt);

	void resetIntegral();

	private:
		TIM_HandleTypeDef* tim_;
		Channels ch_;
		float integral = 0.0f;
};



#endif /* SRC_MOTORS_H_ */

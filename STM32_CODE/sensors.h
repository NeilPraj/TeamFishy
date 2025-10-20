#ifndef SRC_SENSORS_H_
#define SRC_SENSORS_H_

#include <cstdint>
#include "main.h"
#include <bitset>
#include <climits>


class sensors {
public:
    struct Inputs{
        uint32_t s0;
        uint32_t s1;
        uint32_t s2;
        uint32_t s3;
        uint32_t s4;
        uint32_t s5;
    };

    explicit sensors(const sensors::Inputs& inputs);
    virtual ~sensors();

    uint8_t getValues(void);

    int16_t pidController(float kp, float ki, float kd, int16_t error, int16_t previous_error, float dt);

    int16_t getShiftErr(const std::bitset<8>& sensorBits);

private:
    Inputs _inputs;
	float integral = 0.0f;
};

#endif /* SRC_SENSORS_H_ */

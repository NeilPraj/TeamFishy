/*
 * imu.h - MPU-6500 gyro/accel driver (bit-banged SPI mode 0) for PIC18 + XC8
 *
 * Usage:
 *   imu_status_t st = imu_init();     // call with motors stopped, robot still
 *   ...
 *   imu_update(elapsed_ticks);        // once per control loop pass
 *   int32_t yaw = imu_yaw_mdeg();     // milli-degrees since last reset
 */
#ifndef IMU_H
#define IMU_H
 
#include <stdint.h>
#include <stdbool.h>
 
typedef enum {
    IMU_OK = 0,
    IMU_ERR_WHOAMI,     /* WHO_AM_I mismatch: wiring, CS, power, or wrong part */
    IMU_ERR_CAL         /* gyro calibration kept failing: robot was moving */
} imu_status_t;
 
/* Configure pins, reset + configure the MPU-6500, verify WHO_AM_I, calibrate
 * gyro Z bias. Robot must be stationary. Takes roughly 1 s. */
imu_status_t imu_init(void);
 
/* Re-measure gyro Z bias (robot must be stationary) and zero yaw.
 * Returns false (and keeps the old bias) if the robot was not still. */
bool imu_calibrate_gyro_z(void);
 
/* Sample the sensor and integrate yaw. Pass the number of 4.096 ms control
 * ticks elapsed since the last call (normally 1). No-op if init failed. */
void imu_update(uint8_t elapsed_ticks);
 
bool     imu_ok(void);
uint8_t  imu_whoami(void);
 
int32_t  imu_yaw_mdeg(void);          /* integrated heading, milli-degrees */
void     imu_yaw_reset(void);         /* zero yaw (also clears the remainder) */
 
int16_t  imu_gyro_z_raw(void);        /* last raw sample */
int16_t  imu_gyro_z_bias(void);       /* calibrated bias */
int32_t  imu_gyro_z_corrected(void);  /* raw - bias, int32 so no overflow */
int16_t  imu_accel_x_raw(void);
int16_t  imu_accel_y_raw(void);
 
#endif /* IMU_H */
 
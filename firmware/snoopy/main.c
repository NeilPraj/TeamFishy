/*
 * SNOOPY turnkey line follower main.c - v4 track-checked
 * Target: PIC18F47Q43 @ 64 MHz HFINTOSC
 *
 * Board mapping is based on the supplied schematic and SNOOPY v0.22 MCC project.
 *
 * This version adds:
 *   - Explicit black-line/white-background sensor interpretation
 *   - Seven-sensor PD line following
 *   - Closed-loop left/right motor mismatch trim from wheel encoders
 *   - MPU-6500 SPI bring-up without requiring an MCC SPI component
 *   - WHO_AM_I verification (expects 0x70)
 *   - Gyro-Z bias calibration and relative yaw integration
 *   - IMU/encoder-assisted lost-line recovery decisions
 *   - Recovery envelope checked against the official Round 3 track spec
 *
 * IMPORTANT PHYSICAL ASSUMPTIONS TO VERIFY ON THE ROBOT:
 *   1. SENSOR1 is leftmost and SENSOR7 is rightmost. If not, set
 *      SENSOR_ORDER_REVERSED to 1.
 *   2. Encoder1 is the LEFT wheel and Encoder2 is the RIGHT wheel. If not,
 *      set ENCODERS_SWAPPED to 1.
 *   3. Positive motor commands move both wheels forward. If one motor is
 *      reversed mechanically, swap that motor's P1/P2 mapping in motors_set().
 *
 * The encoder trim intentionally learns wheel mismatch only while the robot
 * is approximately straight. It does NOT force equal wheel speed during a
 * commanded turn, which would fight the line controller.
 */

#define _XTAL_FREQ 64000000UL

#include "mcc_generated_files/system/system.h"
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#pragma warning disable 520

/* ========================= USER TUNING ========================= */

/* The original v0.22 firmware documents the raw CNY70 receiver behavior:
 *   PORTCx HIGH = little/no reflection = BLACK
 *   PORTCx LOW  = strong reflection    = WHITE
 * Its (~PORTC & 0xFE) macro was a REFLECTION display where 1 meant white,
 * not a black-line-active mask. For a black line on white background, raw
 * HIGH is therefore the line-active state. Set this to 0 only for a white
 * line on a dark background. */
#define FOLLOW_BLACK_LINE         1u
#define SENSOR_ORDER_REVERSED     0u
#define ENCODERS_SWAPPED          0u

#define BASE_PWM                  600     /* 0..1023 */
#define MAX_PWM                   900
#define MAX_CORRECTION            450
#define LOST_LINE_TURN_PWM        350

/* Integer line-following PD gains. Error is approximately -30..+30. */
#define KP                        8
#define KD                        12

/* Encoder mismatch trim. Updated only while essentially driving straight. */
#define SPEED_WINDOW_TICKS        10u     /* 10 * 4.096 ms = 40.96 ms */
#define STRAIGHT_ERROR_LIMIT      5
#define STRAIGHT_CORR_LIMIT       90
#define SPEED_TRIM_STEP           5       /* PWM counts per pulse mismatch */
#define SPEED_TRIM_MAX            120

/* Lost-line dead-reckoning thresholds. These are conservative starting values. */
#define RECOVERY_YAW_SOFT_MDEG    90000L  /* 90 degrees: begin softening search */
#define RECOVERY_YAW_MAX_MDEG     170000L /* 170 degrees: allows ~150 deg heading change at a 30 deg acute corner */
#define RECOVERY_REACQUIRE_MAX_MDEG 175000L /* reject near-180 deg reacquisition that risks wrong-way travel */
#define RECOVERY_PULSE_SOFT       8u
#define RECOVERY_PULSE_MAX        24u

/* 74HC595 pattern: bits 7..1 correspond to seven IR emitters. */
#define OPTO_ALL_ON               0xFEu

/* TMR0 = 4.096 ms in the supplied MCC project. */
#define DEBUG_TICKS               122u    /* ~0.50 s */
#define BUTTON_DEBOUNCE_TICKS     12u     /* ~49 ms */

/* ========================= MPU-6500 REGISTERS ================== */

#define MPU_SMPLRT_DIV            0x19u
#define MPU_CONFIG                0x1Au
#define MPU_GYRO_CONFIG           0x1Bu
#define MPU_ACCEL_CONFIG          0x1Cu
#define MPU_ACCEL_CONFIG2         0x1Du
#define MPU_INT_PIN_CFG           0x37u
#define MPU_INT_ENABLE            0x38u
#define MPU_ACCEL_XOUT_H          0x3Bu
#define MPU_GYRO_XOUT_H           0x43u
#define MPU_USER_CTRL             0x6Au
#define MPU_PWR_MGMT_1            0x6Bu
#define MPU_PWR_MGMT_2            0x6Cu
#define MPU_WHO_AM_I              0x75u
#define MPU_WHO_AM_I_VALUE        0x70u

/* ±500 dps => 65.5 LSB/(deg/s). */
#define MPU_GYRO_FS_500DPS        0x08u
#define MPU_ACCEL_FS_4G           0x08u

/* ========================= BOARD PIN MAP ======================= */

#define BUTTON_PRESSED()          (PORTAbits.RA3 == 0)

#define SERIN_HIGH()              (LATAbits.LATA4 = 1)
#define SERIN_LOW()               (LATAbits.LATA4 = 0)
#define SCLK_HIGH()               (LATAbits.LATA5 = 1)
#define SCLK_LOW()                (LATAbits.LATA5 = 0)
#define RCLK_HIGH()               (LATAbits.LATA6 = 1)
#define RCLK_LOW()                (LATAbits.LATA6 = 0)
#define SCLRN_HIGH()              (LATCbits.LATC0 = 1)

#define LED_ON()                  (LATBbits.LATB6 = 0)
#define LED_OFF()                 (LATBbits.LATB6 = 1)

#define ENCODER1_LEVEL()          (PORTDbits.RD4)
#define ENCODER2_LEVEL()          (PORTDbits.RD6)

#define IMU_SCK_HIGH()            (LATDbits.LATD0 = 1)
#define IMU_SCK_LOW()             (LATDbits.LATD0 = 0)
#define IMU_MOSI_HIGH()           (LATDbits.LATD1 = 1)
#define IMU_MOSI_LOW()            (LATDbits.LATD1 = 0)
#define IMU_MISO_LEVEL()          (PORTDbits.RD2)
#define IMU_CS_HIGH()             (LATDbits.LATD3 = 1)
#define IMU_CS_LOW()              (LATDbits.LATD3 = 0)
#define IMU_INT_LEVEL()           (PORTEbits.RE0)

/* ========================= STATE ================================ */

volatile uint8_t control_tick = 0u;
volatile uint8_t debug_tick = 0u;
volatile uint8_t debug_report_due = 0u;

volatile uint32_t encoder1_count = 0u;
volatile uint32_t encoder2_count = 0u;
static uint8_t enc1_last = 0u;
static uint8_t enc2_last = 0u;

static int16_t last_error = 0;
static int16_t last_correction = 0;
static int8_t last_line_side = 1;
static bool robot_running = false;
static bool line_was_valid = false;

static int16_t speed_trim = 0;
static uint8_t speed_window_tick = 0u;
static uint32_t speed_left_prev = 0u;
static uint32_t speed_right_prev = 0u;
static uint16_t speed_left_delta = 0u;
static uint16_t speed_right_delta = 0u;

static bool imu_ok = false;
static uint8_t imu_whoami = 0u;
static int16_t gyro_z_raw = 0;
static int16_t gyro_z_bias = 0;
static int16_t accel_x_raw = 0;
static int16_t accel_y_raw = 0;
static int32_t yaw_mdeg = 0L;

static int32_t recovery_start_yaw_mdeg = 0L;
static uint32_t recovery_start_left_count = 0u;
static uint32_t recovery_start_right_count = 0u;

/* Last motor commands are useful for diagnostics and recovery state. */
static int16_t last_left_cmd = 0;
static int16_t last_right_cmd = 0;

/* ========================= SMALL HELPERS ======================== */

static int16_t clamp_s16(int16_t x, int16_t lo, int16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static int32_t abs_s32(int32_t x)
{
    return (x < 0L) ? -x : x;
}

static void uart_putc(char c)
{
    UART1_Write((uint8_t)c);
}

static void uart_puts(const char *s)
{
    while (*s != '\0') uart_putc(*s++);
}

static void uart_hex_nibble(uint8_t n)
{
    n &= 0x0Fu;
    uart_putc((char)((n < 10u) ? ('0' + n) : ('A' + n - 10u)));
}

static void uart_hex8(uint8_t x)
{
    uart_hex_nibble((uint8_t)(x >> 4));
    uart_hex_nibble(x);
}

static void uart_u32(uint32_t v)
{
    char buf[11];
    uint8_t i = 0u;
    do {
        buf[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while ((v != 0u) && (i < sizeof(buf)));
    while (i != 0u) uart_putc(buf[--i]);
}

static void uart_i32(int32_t x)
{
    uint32_t v;
    if (x < 0L) {
        uart_putc('-');
        v = (uint32_t)(-x);
    } else {
        v = (uint32_t)x;
    }
    uart_u32(v);
}

static void uart_i16(int16_t x)
{
    uart_i32((int32_t)x);
}

/* ========================= IR EMITTER SHIFT REG ================ */

static void opto_write(uint8_t pattern)
{
    uint8_t i;
    RCLK_LOW();
    SCLRN_HIGH();

    for (i = 0u; i < 8u; i++) {
        SCLK_LOW();
        if ((pattern & 0x80u) != 0u) SERIN_HIGH();
        else SERIN_LOW();
        pattern <<= 1;
        SCLK_HIGH();
    }
    RCLK_HIGH();
}

/* ========================= SENSOR READING ======================= */

static uint8_t line_sensor_bits(void)
{
    /* Normalize to: bit=1 means "this sensor sees the line".
     * Raw receiver HIGH = black, LOW = white on this board. */
    uint8_t s = (uint8_t)((PORTC >> 1) & 0x7Fu);

#if !FOLLOW_BLACK_LINE
    s ^= 0x7Fu;
#endif

#if SENSOR_ORDER_REVERSED
    {
        uint8_t r = 0u;
        uint8_t i;
        for (i = 0u; i < 7u; i++) {
            if ((s & (1u << i)) != 0u)
                r |= (uint8_t)(1u << (6u - i));
        }
        s = r;
    }
#endif
    return s;
}

static bool line_error_get(uint8_t sensors, int16_t *error)
{
    static const int8_t weight[7] = {-30, -20, -10, 0, 10, 20, 30};
    int16_t sum = 0;
    uint8_t count = 0u;
    uint8_t i;

    for (i = 0u; i < 7u; i++) {
        if ((sensors & (1u << i)) != 0u) {
            sum += weight[i];
            count++;
        }
    }

    if (count == 0u) return false;
    *error = (int16_t)(sum / (int16_t)count);
    return true;
}

/* ========================= MOTOR OUTPUT ========================= */

static void motors_set(int16_t left, int16_t right)
{
    left  = clamp_s16(left,  -1023, 1023);
    right = clamp_s16(right, -1023, 1023);

    last_left_cmd = left;
    last_right_cmd = right;

    /* PWM2 = left motor. */
    if (left > 0) {
        PWM2S1P1 = (uint16_t)left;
        PWM2S1P2 = 0u;
    } else if (left < 0) {
        PWM2S1P1 = 0u;
        PWM2S1P2 = (uint16_t)(-left);
    } else {
        PWM2S1P1 = 0u;
        PWM2S1P2 = 0u;
    }

    /* PWM1 = right motor. */
    if (right > 0) {
        PWM1S1P1 = (uint16_t)right;
        PWM1S1P2 = 0u;
    } else if (right < 0) {
        PWM1S1P1 = 0u;
        PWM1S1P2 = (uint16_t)(-right);
    } else {
        PWM1S1P1 = 0u;
        PWM1S1P2 = 0u;
    }

    PWM1CONbits.LD = 1;
    PWM2CONbits.LD = 1;
}

static void motors_stop(void)
{
    motors_set(0, 0);
}

/* ========================= ENCODERS ============================= */

static void encoders_poll(void)
{
    uint8_t e1 = (ENCODER1_LEVEL() != 0u) ? 1u : 0u;
    uint8_t e2 = (ENCODER2_LEVEL() != 0u) ? 1u : 0u;

    if ((e1 != 0u) && (enc1_last == 0u)) encoder1_count++;
    if ((e2 != 0u) && (enc2_last == 0u)) encoder2_count++;

    enc1_last = e1;
    enc2_last = e2;
}

static uint32_t left_encoder_count_get(void)
{
#if ENCODERS_SWAPPED
    return encoder2_count;
#else
    return encoder1_count;
#endif
}

static uint32_t right_encoder_count_get(void)
{
#if ENCODERS_SWAPPED
    return encoder1_count;
#else
    return encoder2_count;
#endif
}

/* Learn a persistent left/right PWM trim only when the line controller is
 * asking for nearly straight travel. This avoids fighting intentional turns. */
static void speed_balance_update(void)
{
    uint32_t left_now;
    uint32_t right_now;
    uint32_t dl;
    uint32_t dr;
    int16_t mismatch;

    if (++speed_window_tick < SPEED_WINDOW_TICKS) return;
    speed_window_tick = 0u;

    left_now = left_encoder_count_get();
    right_now = right_encoder_count_get();
    dl = left_now - speed_left_prev;
    dr = right_now - speed_right_prev;
    speed_left_prev = left_now;
    speed_right_prev = right_now;

    speed_left_delta = (dl > 65535u) ? 65535u : (uint16_t)dl;
    speed_right_delta = (dr > 65535u) ? 65535u : (uint16_t)dr;

    if (!robot_running) return;

    if ((last_error >= -STRAIGHT_ERROR_LIMIT) &&
        (last_error <=  STRAIGHT_ERROR_LIMIT) &&
        (last_correction >= -STRAIGHT_CORR_LIMIT) &&
        (last_correction <=  STRAIGHT_CORR_LIMIT) &&
        (last_left_cmd > 0) && (last_right_cmd > 0)) {

        mismatch = (int16_t)speed_left_delta - (int16_t)speed_right_delta;

        /* Positive mismatch means left wheel is faster: reduce left / boost right. */
        speed_trim -= (int16_t)(mismatch * SPEED_TRIM_STEP);
        speed_trim = clamp_s16(speed_trim, -SPEED_TRIM_MAX, SPEED_TRIM_MAX);
    }
}

/* ========================= MPU-6500 BIT-BANG SPI ================ */

/* Mode 0 SPI: CPOL=0, CPHA=0. Intentionally slow and simple for bring-up. */
static uint8_t imu_spi_xfer(uint8_t tx)
{
    uint8_t rx = 0u;
    uint8_t i;

    for (i = 0u; i < 8u; i++) {
        IMU_SCK_LOW();
        if ((tx & 0x80u) != 0u) IMU_MOSI_HIGH();
        else IMU_MOSI_LOW();
        __delay_us(1);

        IMU_SCK_HIGH();
        rx <<= 1;
        if (IMU_MISO_LEVEL() != 0u) rx |= 1u;
        __delay_us(1);

        tx <<= 1;
    }
    IMU_SCK_LOW();
    return rx;
}

static void imu_write_reg(uint8_t reg, uint8_t value)
{
    IMU_CS_LOW();
    (void)imu_spi_xfer((uint8_t)(reg & 0x7Fu));
    (void)imu_spi_xfer(value);
    IMU_CS_HIGH();
}

static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t v;
    IMU_CS_LOW();
    (void)imu_spi_xfer((uint8_t)(reg | 0x80u));
    v = imu_spi_xfer(0xFFu);
    IMU_CS_HIGH();
    return v;
}

static void imu_read_bytes(uint8_t start_reg, uint8_t *dst, uint8_t n)
{
    uint8_t i;
    IMU_CS_LOW();
    (void)imu_spi_xfer((uint8_t)(start_reg | 0x80u));
    for (i = 0u; i < n; i++) dst[i] = imu_spi_xfer(0xFFu);
    IMU_CS_HIGH();
}

static void imu_sample(void)
{
    uint8_t b[14];

    if (!imu_ok) return;

    /* One burst gets accel, temperature, and gyro coherently. */
    imu_read_bytes(MPU_ACCEL_XOUT_H, b, 14u);

    accel_x_raw = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    accel_y_raw = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
    gyro_z_raw  = (int16_t)(((uint16_t)b[12] << 8) | b[13]);

    /* ±500 dps sensitivity = 65.5 LSB/dps.
     * TMR0 dt = 4.096 ms.
     * delta_mdeg = raw * 1000/65.5 * 0.004096
     *            = raw * 4096 / 65500.
     */
    yaw_mdeg += (((int32_t)gyro_z_raw - (int32_t)gyro_z_bias) * 4096L) / 65500L;
}

static void imu_calibrate_gyro_z(void)
{
    int32_t sum = 0L;
    uint16_t i;
    uint8_t b[6];
    int16_t z;

    uart_puts("Keep robot still: calibrating gyro...\r\n");

    for (i = 0u; i < 256u; i++) {
        imu_read_bytes(MPU_GYRO_XOUT_H, b, 6u);
        z = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
        sum += (int32_t)z;
        __delay_ms(2);
    }

    gyro_z_bias = (int16_t)(sum / 256L);
    yaw_mdeg = 0L;
}

static bool imu_init(void)
{
    IMU_CS_HIGH();
    IMU_SCK_LOW();
    IMU_MOSI_LOW();
    __delay_ms(20);

    /* Device reset. */
    imu_write_reg(MPU_PWR_MGMT_1, 0x80u);
    __delay_ms(100);

    /* PLL with X gyro reference; enable all axes. */
    imu_write_reg(MPU_PWR_MGMT_1, 0x01u);
    imu_write_reg(MPU_PWR_MGMT_2, 0x00u);
    __delay_ms(10);

    /* Disable the I2C interface so the SPI bus owns the device. */
    imu_write_reg(MPU_USER_CTRL, 0x10u);

    /* 1 kHz internal rate / (1+3) = 250 Hz, close to 244.14 Hz control loop. */
    imu_write_reg(MPU_SMPLRT_DIV, 0x03u);
    imu_write_reg(MPU_CONFIG, 0x03u);          /* gyro DLPF ~41 Hz */
    imu_write_reg(MPU_GYRO_CONFIG, MPU_GYRO_FS_500DPS);
    imu_write_reg(MPU_ACCEL_CONFIG, MPU_ACCEL_FS_4G);
    imu_write_reg(MPU_ACCEL_CONFIG2, 0x03u);   /* accel DLPF */
    imu_write_reg(MPU_INT_PIN_CFG, 0x00u);
    imu_write_reg(MPU_INT_ENABLE, 0x01u);      /* DATA_RDY interrupt */

    imu_whoami = imu_read_reg(MPU_WHO_AM_I);
    if (imu_whoami != MPU_WHO_AM_I_VALUE) return false;

    imu_calibrate_gyro_z();
    return true;
}

/* ========================= DEAD-RECKONING RECOVERY ============== */

static void recovery_begin(void)
{
    recovery_start_yaw_mdeg = yaw_mdeg;
    recovery_start_left_count = left_encoder_count_get();
    recovery_start_right_count = right_encoder_count_get();
}

static void lost_line_recovery(void)
{
    int32_t yaw_delta = yaw_mdeg - recovery_start_yaw_mdeg;
    int32_t yaw_mag = abs_s32(yaw_delta);
    uint32_t left_pulses = left_encoder_count_get() - recovery_start_left_count;
    uint32_t right_pulses = right_encoder_count_get() - recovery_start_right_count;
    uint32_t avg_pulses = (left_pulses + right_pulses) / 2u;
    int16_t turn = LOST_LINE_TURN_PWM;

    /* First use the remembered side to continue the likely corner. The official
     * Round 3 track permits acute angles down to 30 degrees, which can require
     * roughly a 150 degree heading change through a sharp vertex. When the IMU
     * is healthy, yaw is therefore the PRIMARY recovery limit; encoder pulse
     * counts are only the fallback when the IMU is unavailable. This prevents
     * an arbitrary encoder resolution from prematurely aborting a valid sharp
     * turn. */
    {
        bool soft_limit;
        bool max_limit;

        if (imu_ok) {
            soft_limit = (yaw_mag >= RECOVERY_YAW_SOFT_MDEG);
            max_limit  = (yaw_mag >= RECOVERY_YAW_MAX_MDEG);
        } else {
            soft_limit = (avg_pulses >= RECOVERY_PULSE_SOFT);
            max_limit  = (avg_pulses >= RECOVERY_PULSE_MAX);
        }

        if (max_limit) {
            turn = (int16_t)(LOST_LINE_TURN_PWM / 2);
            if (last_line_side < 0)
                motors_set(turn, -turn);
            else
                motors_set(-turn, turn);
            return;
        }

        if (soft_limit)
            turn = (int16_t)((LOST_LINE_TURN_PWM * 3) / 4);
    }

    if (last_line_side < 0)
        motors_set(-turn, turn);
    else
        motors_set(turn, -turn);
}

/* ========================= CONTROL LOOP ========================= */

static void line_follow_step(void)
{
    uint8_t sensors;
    int16_t error;
    int16_t derivative;
    int16_t correction;
    int16_t left;
    int16_t right;
    bool valid;

    if (!robot_running) {
        line_was_valid = false;
        motors_stop();
        return;
    }

    sensors = line_sensor_bits();
    valid = line_error_get(sensors, &error);

    /* If we are recovering with a healthy gyro, do not accept a line first
     * encountered after an almost-180-degree rotation. That is much more likely
     * to be the previous/wrong-way segment than a legitimate <=150-degree acute
     * corner on the official track. */
    if (valid && !line_was_valid && imu_ok) {
        if (abs_s32(yaw_mdeg - recovery_start_yaw_mdeg) >= RECOVERY_REACQUIRE_MAX_MDEG)
            valid = false;
    }

    if (!valid) {
        if (line_was_valid) recovery_begin();
        line_was_valid = false;
        lost_line_recovery();
        return;
    }

    /* Do not kick the D term on the first sample after startup or
     * reacquiring a lost line. */
    if (!line_was_valid) {
        last_error = error;
        derivative = 0;
    } else {
        derivative = (int16_t)(error - last_error);
        last_error = error;
    }
    line_was_valid = true;

    if (error < 0) last_line_side = -1;
    else if (error > 0) last_line_side = 1;

    correction = (int16_t)((KP * error) + (KD * derivative));
    correction = clamp_s16(correction, -MAX_CORRECTION, MAX_CORRECTION);
    last_correction = correction;

    /* Steering plus learned motor mismatch compensation. */
    left  = (int16_t)(BASE_PWM + correction + speed_trim);
    right = (int16_t)(BASE_PWM - correction - speed_trim);

    left  = clamp_s16(left,  0, MAX_PWM);
    right = clamp_s16(right, 0, MAX_PWM);

    motors_set(left, right);
}

/* ========================= BUTTON =============================== */

static void button_update(void)
{
    static uint8_t stable_ticks = 0u;
    static bool latched = false;

    if (BUTTON_PRESSED()) {
        if (stable_ticks < BUTTON_DEBOUNCE_TICKS) stable_ticks++;

        if ((stable_ticks >= BUTTON_DEBOUNCE_TICKS) && !latched) {
            latched = true;
            robot_running = !robot_running;
            last_error = 0;
            last_correction = 0;
            line_was_valid = false;

            if (robot_running) {
                speed_left_prev = left_encoder_count_get();
                speed_right_prev = right_encoder_count_get();
                speed_window_tick = 0u;
                recovery_begin();  /* makes starting off-line deterministic */
                LED_ON();
                uart_puts("RUN\r\n");
            } else {
                motors_stop();
                LED_OFF();
                uart_puts("STOP\r\n");
            }
        }
    } else {
        stable_ticks = 0u;
        latched = false;
    }
}

/* ========================= TIMER CALLBACK ======================= */

static void control_timer_isr(void)
{
    control_tick = 1u;

    if (++debug_tick >= DEBUG_TICKS) {
        debug_tick = 0u;
        debug_report_due = 1u;
    }
}

/* ========================= ADDED FEATURE PINS =================== */

static void added_feature_pins_init(void)
{
    /* Port D/E were not configured by the original MCC pin manager. */
    ANSELD = 0x00u;
    ANSELE = 0x00u;
    INLVLD = 0xFFu;
    INLVLE = 0xFFu;

    /* MPU SPI: RD0=SCLK, RD1=MOSI, RD2=MISO, RD3=CS. */
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 1;
    TRISDbits.TRISD3 = 0;

    /* Wheel encoders. */
    TRISDbits.TRISD4 = 1;
    TRISDbits.TRISD6 = 1;

    /* MPU data-ready interrupt input. The code currently samples at the fixed
     * control rate, while this pin remains available for a later ISR upgrade. */
    TRISEbits.TRISE0 = 1;

    IMU_SCK_LOW();
    IMU_MOSI_LOW();
    IMU_CS_HIGH();

    enc1_last = (ENCODER1_LEVEL() != 0u) ? 1u : 0u;
    enc2_last = (ENCODER2_LEVEL() != 0u) ? 1u : 0u;
}

/* ========================= TELEMETRY ============================ */

static void debug_report(void)
{
    uint8_t s = line_sensor_bits();
    int16_t e;

    uart_puts("S=");
    uart_hex8(s);
    uart_puts(" E=");
    if (line_error_get(s, &e)) uart_i16(e);
    else uart_puts("LOST");

    uart_puts(" ENC(L/R)=");
    uart_u32(left_encoder_count_get());
    uart_putc('/');
    uart_u32(right_encoder_count_get());

    uart_puts(" d=");
    uart_u32(speed_left_delta);
    uart_putc('/');
    uart_u32(speed_right_delta);

    uart_puts(" trim=");
    uart_i16(speed_trim);

    uart_puts(" Gz=");
    uart_i16(gyro_z_raw - gyro_z_bias);
    uart_puts(" yaw_mdeg=");
    uart_i32(yaw_mdeg);

    uart_puts(robot_running ? " RUN\r\n" : " STOP\r\n");
}

/* ========================= MAIN ================================= */

int main(void)
{
    SYSTEM_Initialize();

    INLVLA = 0xFFu;
    INLVLB = 0xFFu;
    INLVLC = 0xFFu;

    added_feature_pins_init();

    /* Keep motors stopped during IMU reset/calibration. */
    PWM1CONbits.EN = 1;
    PWM2CONbits.EN = 1;
    motors_stop();
    LED_OFF();

    SCLRN_HIGH();
    opto_write(OPTO_ALL_ON);

    /* Register the control callback and enable interrupts before printing so
     * the MCC UART TX ring buffer can drain during IMU startup/calibration. */
    TMR0_PeriodMatchCallbackRegister(control_timer_isr);
    INTCON0bits.GIEH = 1;
    INTCON0bits.GIEL = 1;

    uart_puts("\r\nSNOOPY line follower v4 track-checked\r\n");
    uart_puts(FOLLOW_BLACK_LINE ? "Following BLACK line (raw sensor HIGH)\r\n" : "Following WHITE line (raw sensor LOW)\r\n");

    imu_ok = imu_init();
    uart_puts("MPU-6500 WHO_AM_I=0x");
    uart_hex8(imu_whoami);
    uart_puts(imu_ok ? " OK\r\n" : " FAIL - IMU features disabled\r\n");

    speed_left_prev = left_encoder_count_get();
    speed_right_prev = right_encoder_count_get();

    uart_puts("Press SW2 to RUN/STOP\r\n");

    while (1) {
        /* Fast polling catches encoder transitions between control ticks. */
        encoders_poll();

        if (control_tick != 0u) {
            control_tick = 0u;

            button_update();

            /* Sample gyro/accel every 4.096 ms. If WHO_AM_I failed, this is a no-op. */
            imu_sample();

            /* Update persistent wheel-speed mismatch trim from encoder windows. */
            speed_balance_update();

            line_follow_step();

            if (debug_report_due != 0u) {
                debug_report_due = 0u;
                debug_report();
            }
        }
    }
}

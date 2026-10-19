/*
 * imu.c - MPU-6500 driver, bit-banged SPI mode 0 (CPOL=0, CPHA=0)
 *
 * Pins: RD0=SCLK  RD1=MOSI  RD2=MISO  RD3=CS  RE0=INT (input, unused for now)
 */

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 64000000UL      
#endif

#include <xc.h>
#include "imu.h"

/* ========================= CONFIG ================================= */

/* Yaw scale: gyro at +/-500 dps = 65.5 LSB/dps, sample period = 4.096 ms.
 *   delta_mdeg = raw * (1000 / 65.5) * 0.004096 = raw * 4096 / 65500 */
#define IMU_YAW_NUM             4096L
#define IMU_YAW_DEN             65500L

/* Max ticks integrated in one update. Keeps (raw-bias)*NUM*ticks inside int32:
 * 65534 * 4096 * 4 = 1.07e9 < 2.147e9. */
#define IMU_MAX_CATCHUP         4u

/* Calibration */
#define IMU_CAL_SAMPLES         128u    /* power of two */
#define IMU_CAL_SAMPLE_MS       4       /* matches 250 Hz sensor rate */
#define IMU_CAL_MAX_SPREAD      80      /* max-min raw LSB (~1.2 dps) allowed */
#define IMU_CAL_MAX_BIAS        2000    /* |bias| above ~30 dps is not credible */
#define IMU_CAL_ATTEMPTS        3u
#define IMU_WHOAMI_ATTEMPTS     3u

/* ========================= REGISTERS ============================== */

#define MPU_SMPLRT_DIV          0x19u
#define MPU_CONFIG              0x1Au
#define MPU_GYRO_CONFIG         0x1Bu
#define MPU_ACCEL_CONFIG        0x1Cu
#define MPU_ACCEL_CONFIG2       0x1Du
#define MPU_INT_PIN_CFG         0x37u
#define MPU_INT_ENABLE          0x38u
#define MPU_ACCEL_XOUT_H        0x3Bu
#define MPU_GYRO_XOUT_H         0x43u
#define MPU_SIGNAL_PATH_RESET   0x68u
#define MPU_USER_CTRL           0x6Au
#define MPU_PWR_MGMT_1          0x6Bu
#define MPU_PWR_MGMT_2          0x6Cu
#define MPU_WHO_AM_I            0x75u
#define MPU_WHO_AM_I_VALUE      0x70u

#define MPU_GYRO_FS_500DPS      0x08u
#define MPU_ACCEL_FS_4G         0x08u

/* ========================= PINS =================================== */

#define IMU_SCK_HIGH()          (LATDbits.LATD0 = 1)
#define IMU_SCK_LOW()           (LATDbits.LATD0 = 0)
#define IMU_MOSI_HIGH()         (LATDbits.LATD1 = 1)
#define IMU_MOSI_LOW()          (LATDbits.LATD1 = 0)
#define IMU_MISO_LEVEL()        (PORTDbits.RD2)
#define IMU_CS_HIGH()           (LATDbits.LATD3 = 1)
#define IMU_CS_LOW()            (LATDbits.LATD3 = 0)

/* ========================= STATE ================================== */

static bool     s_ok = false;
static uint8_t  s_whoami = 0u;
static int16_t  s_gyro_z_raw = 0;
static int16_t  s_gyro_z_bias = 0;
static int16_t  s_accel_x_raw = 0;
static int16_t  s_accel_y_raw = 0;
static int32_t  s_yaw_mdeg = 0L;
static int32_t  s_yaw_frac = 0L;   /* remainder in units of 1/IMU_YAW_DEN mdeg */

/* ========================= PINS INIT ============================== */

static void imu_pins_init(void)
{
    /* Latch idle levels BEFORE enabling the outputs so CS never glitches low. */
    IMU_CS_HIGH();
    IMU_SCK_LOW();
    IMU_MOSI_LOW();

    ANSELD &= (uint8_t)~0x0Fu;          /* RD0..RD3 digital */
    ANSELE &= (uint8_t)~0x01u;          /* RE0 digital */

    TRISDbits.TRISD0 = 0;               /* SCLK out */
    TRISDbits.TRISD1 = 0;               /* MOSI out */
    TRISDbits.TRISD2 = 1;               /* MISO in  */
    TRISDbits.TRISD3 = 0;               /* CS   out */
    TRISEbits.TRISE0 = 1;               /* INT  in  */
}

/* ========================= SPI ==================================== */

static uint8_t spi_xfer(uint8_t tx)
{
    uint8_t rx = 0u;
    uint8_t i;

    for (i = 0u; i < 8u; i++) {
        IMU_SCK_LOW();
        if ((tx & 0x80u) != 0u) IMU_MOSI_HIGH();
        else                    IMU_MOSI_LOW();
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

static void write_reg(uint8_t reg, uint8_t value)
{
    IMU_CS_LOW();
    (void)spi_xfer((uint8_t)(reg & 0x7Fu));
    (void)spi_xfer(value);
    IMU_CS_HIGH();
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t v;
    IMU_CS_LOW();
    (void)spi_xfer((uint8_t)(reg | 0x80u));
    v = spi_xfer(0xFFu);
    IMU_CS_HIGH();
    return v;
}

static void read_bytes(uint8_t start_reg, uint8_t *dst, uint8_t n)
{
    uint8_t i;
    IMU_CS_LOW();
    (void)spi_xfer((uint8_t)(start_reg | 0x80u));
    for (i = 0u; i < n; i++) dst[i] = spi_xfer(0xFFu);
    IMU_CS_HIGH();
}

static int16_t be16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

/* ========================= CALIBRATION ============================ */

static bool calibrate_once(int16_t *bias_out)
{
    int32_t sum = 0L;
    int16_t zmin = 32767;
    int16_t zmax = -32768;
    int16_t z;
    int16_t bias;
    uint8_t b[2];
    uint16_t i;

    for (i = 0u; i < IMU_CAL_SAMPLES; i++) {
        read_bytes((uint8_t)(MPU_GYRO_XOUT_H + 4u), b, 2u);   /* GYRO_ZOUT */
        z = be16(b[0], b[1]);
        sum += (int32_t)z;
        if (z < zmin) zmin = z;
        if (z > zmax) zmax = z;
        __delay_ms(IMU_CAL_SAMPLE_MS);
    }

    /* Reject if the robot moved (large spread) */
    if (((int32_t)zmax - (int32_t)zmin) > IMU_CAL_MAX_SPREAD) return false;

    bias = (int16_t)(sum / (int32_t)IMU_CAL_SAMPLES);
    if ((bias > IMU_CAL_MAX_BIAS) || (bias < -IMU_CAL_MAX_BIAS)) return false;

    *bias_out = bias;
    return true;
}

bool imu_calibrate_gyro_z(void)
{
    uint8_t attempt;
    int16_t bias;

    for (attempt = 0u; attempt < IMU_CAL_ATTEMPTS; attempt++) {
        if (calibrate_once(&bias)) {
            s_gyro_z_bias = bias;
            imu_yaw_reset();
            return true;
        }
    }
    return false;
}

/* ========================= INIT =================================== */

imu_status_t imu_init(void)
{
    uint8_t attempt;

    s_ok = false;
    imu_pins_init();
    __delay_ms(20);

    /* Full device reset, then reset the analog/digital signal paths. */
    write_reg(MPU_PWR_MGMT_1, 0x80u);
    __delay_ms(100);
    write_reg(MPU_SIGNAL_PATH_RESET, 0x07u);
    __delay_ms(100);

    /* Lock the part into SPI mode (disable I2C interface). */
    write_reg(MPU_USER_CTRL, 0x10u);

    /* Fail fast: confirm we are talking to the right chip before configuring. */
    s_whoami = 0u;
    for (attempt = 0u; attempt < IMU_WHOAMI_ATTEMPTS; attempt++) {
        s_whoami = read_reg(MPU_WHO_AM_I);
        if (s_whoami == MPU_WHO_AM_I_VALUE) break;
        __delay_ms(10);
    }
    if (s_whoami != MPU_WHO_AM_I_VALUE) return IMU_ERR_WHOAMI;

    /* Wake with PLL (X gyro ref), enable all axes. */
    write_reg(MPU_PWR_MGMT_1, 0x01u);
    write_reg(MPU_PWR_MGMT_2, 0x00u);
    __delay_ms(10);

    /* 1 kHz / (1+3) = 250 Hz output rate, near the 244 Hz control loop. */
    write_reg(MPU_SMPLRT_DIV, 0x03u);
    write_reg(MPU_CONFIG, 0x03u);              /* gyro DLPF ~41 Hz */
    write_reg(MPU_GYRO_CONFIG, MPU_GYRO_FS_500DPS);
    write_reg(MPU_ACCEL_CONFIG, MPU_ACCEL_FS_4G);
    write_reg(MPU_ACCEL_CONFIG2, 0x03u);       /* accel DLPF */
    write_reg(MPU_INT_PIN_CFG, 0x00u);
    write_reg(MPU_INT_ENABLE, 0x01u);          /* DATA_RDY (pin unused for now) */
    __delay_ms(50);                            /* let the DLPF settle */

    if (!imu_calibrate_gyro_z()) return IMU_ERR_CAL;

    s_ok = true;
    return IMU_OK;
}

/* ========================= RUNTIME ================================ */

void imu_update(uint8_t elapsed_ticks)
{
    uint8_t b[14];
    int32_t d;

    if (!s_ok || (elapsed_ticks == 0u)) return;
    if (elapsed_ticks > IMU_MAX_CATCHUP) elapsed_ticks = IMU_MAX_CATCHUP;

    /* One burst gives accel, temp and gyro coherently. */
    read_bytes(MPU_ACCEL_XOUT_H, b, 14u);

    s_accel_x_raw = be16(b[0],  b[1]);
    s_accel_y_raw = be16(b[2],  b[3]);
    s_gyro_z_raw  = be16(b[12], b[13]);

    /* Integrate with remainder carry so small rates are not truncated away.
     * Missed ticks are covered by scaling with elapsed_ticks. */
    d = ((int32_t)s_gyro_z_raw - (int32_t)s_gyro_z_bias)
        * IMU_YAW_NUM * (int32_t)elapsed_ticks;

    s_yaw_frac += d;
    s_yaw_mdeg += s_yaw_frac / IMU_YAW_DEN;
    s_yaw_frac %= IMU_YAW_DEN;
}

/* ========================= ACCESSORS ============================== */

bool    imu_ok(void)               { return s_ok; }
uint8_t imu_whoami(void)           { return s_whoami; }
int32_t imu_yaw_mdeg(void)         { return s_yaw_mdeg; }
void    imu_yaw_reset(void)        { s_yaw_mdeg = 0L; s_yaw_frac = 0L; }
int16_t imu_gyro_z_raw(void)       { return s_gyro_z_raw; }
int16_t imu_gyro_z_bias(void)      { return s_gyro_z_bias; }
int32_t imu_gyro_z_corrected(void) { return (int32_t)s_gyro_z_raw - (int32_t)s_gyro_z_bias; }
int16_t imu_accel_x_raw(void)      { return s_accel_x_raw; }
int16_t imu_accel_y_raw(void)      { return s_accel_y_raw; }

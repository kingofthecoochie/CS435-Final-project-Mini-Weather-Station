#include "compass.h"
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ----------------------------
// QMC5883L REGISTER DEFINITIONS
// ----------------------------
#define QMC5883L_ADDR        (0x0D << 1)

#define REG_X_LSB   0x00
#define REG_X_MSB   0x01
#define REG_Y_LSB   0x02
#define REG_Y_MSB   0x03
#define REG_Z_LSB   0x04
#define REG_Z_MSB   0x05

#define REG_STATUS  0x06
#define REG_TEMP_L  0x07
#define REG_TEMP_H  0x08

#define REG_CONFIG  0x09
#define REG_CONFIG2 0x0A
#define REG_RESET   0x0B
#define REG_CHIP_ID 0x0D

// CONFIG settings
#define MODE_CONTINUOUS 0x01
#define ODR_50HZ        0x04
#define RANGE_2GAUSS    0x00
#define OSR_512         0x00

// ----------------------------
// INTERNAL CALIBRATION
// ----------------------------
static int16_t x_min =  32767;
static int16_t x_max = -32768;
static int16_t y_min =  32767;
static int16_t y_max = -32768;

static void write_reg(I2C &i2c, char reg, char value)
{
    char cmd[2] = {reg, value};
    i2c.write(QMC5883L_ADDR, cmd, 2);
}

static bool read_bytes(I2C &i2c, char reg, char *buf, int len)
{
    if (i2c.write(QMC5883L_ADDR, &reg, 1) != 0) return false;
    if (i2c.read(QMC5883L_ADDR, buf, len) != 0) return false;
    return true;
}

// ----------------------------
// Initialize Compass
// ----------------------------
bool compass_init(I2C &i2c)
{
    // Reset device
    write_reg(i2c, REG_RESET, 0x01);
    thread_sleep_for(10);

    // Set continuous mode @ 50Hz, 2Gauss, oversampling 512
    write_reg(i2c, REG_CONFIG, OSR_512 | RANGE_2GAUSS | ODR_50HZ | MODE_CONTINUOUS);
    write_reg(i2c, REG_CONFIG2, 0x00);  // Normal mode

    return true;
}

// ----------------------------
// Read heading in degrees
// ----------------------------
bool compass_read_heading(I2C &i2c, float *heading_deg)
{
    char raw[6];

    if (!read_bytes(i2c, REG_X_LSB, raw, 6))
        return false;

    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);

    // Auto-calibration
    if (x < x_min) x_min = x;
    if (x > x_max) x_max = x;
    if (y < y_min) y_min = y;
    if (y > y_max) y_max = y;

    // Avoid divide-by-zero during early values
    if (x_max == x_min || y_max == y_min)
        return false;

    // Normalize
    float x_norm = (float)(x - (x_max + x_min) / 2) / (float)(x_max - x_min);
    float y_norm = (float)(y - (y_max + y_min) / 2) / (float)(y_max - y_min);

    float hdg = atan2f(y_norm, x_norm) * 180.0f / M_PI;
    if (hdg < 0) hdg += 360.0f;

    *heading_deg = hdg;
    return true;
}

// ----------------------------
// Convert degree ? N, NE, E, ...
// ----------------------------
const char* compass_direction_from_deg(float deg)
{
    static const char* dirs[] =
    {
        "N", "NE", "E", "SE", "S", "SW", "W", "NW"
    };

    int index = (int)((deg + 22.5f) / 45.0f) % 8;
    return dirs[index];
}
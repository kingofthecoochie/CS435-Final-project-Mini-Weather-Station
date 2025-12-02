#pragma once
#include "mbed.h"

// Initialize QMC5883L compass
bool compass_init(I2C &i2c);

// Read heading in degree and Returns true if valid reading.
bool compass_read_heading(I2C &i2c, float *heading_deg);

// Convert heading degrees to cardinal direction string ("N","NE",...)
const char* compass_direction_from_deg(float deg);

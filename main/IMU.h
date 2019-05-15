#ifndef IMU_H
#define IMU_H

#include "i2c_t3.h"  // I2C library


// MPU = Motion Processing Unit
// Register information source from the document
// MPU-9250 Register Map and Descriptions Revision 4.2 18/09/2013
//
// Accelerometer - 16bit resolution
// Gyroscope     - 16bit resolution
// Magnet        - 16bit resolution access via the auxiliary I2C bus
//
// The class intentionally reads one register at a time. Reading all registers together
// is available in other public classes.
//

//---------- ENUMs -----------------------

enum mpu9250_gyro_range
{
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
};

enum mpu9250_accel_range
{
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
};

// --------------------------------------------------------------

void getMotion10Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz, int16_t* t);
void getMotion10(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz, float* t);

void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
bool writeRegister(uint8_t subAddress, uint8_t data);

int begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange);

#endif

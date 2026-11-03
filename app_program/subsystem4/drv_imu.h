#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} IMU_DATA;

bool imu_init(void);
bool imu_read(IMU_DATA *data);
bool imu_calibrate(void);
bool imu_read_calibrated(IMU_DATA *data);
bool imu_read_filtered(IMU_DATA *data);
bool imu_read_raw_filtered(IMU_DATA *data);

#endif
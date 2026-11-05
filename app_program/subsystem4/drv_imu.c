#include "drv_imu.h"

#include <tk/tkernel.h>
#include "../../device/include/dev_i2c.h"

#define IMU_ADDRESS 0x19
#define IMU_CTRL_REG1 0x20
#define IMU_OUT_X_L 0x28
#define IMU_CALIBRATION_SAMPLES 100
#define IMU_FILTER_SAMPLES 5

static ID imu_i2c;
static int16_t x_offset = 0;
static int16_t y_offset = 0;
static int16_t z_offset = 0;

bool imu_init(void)
{
    imu_i2c = tk_opn_dev((UB *)"iica", TD_UPDATE);

    if (imu_i2c < E_OK)
    {
        return false;
    }

    return i2c_write_reg(
        imu_i2c,
        IMU_ADDRESS,
        IMU_CTRL_REG1,
        0x27
    ) == E_OK;
}

bool imu_read(IMU_DATA *data)
{
    uint8_t reg = IMU_OUT_X_L | 0x80;
    uint8_t raw[6];
    T_I2C_EXEC exec;
    SZ asz;

    exec.sadr = IMU_ADDRESS;
    exec.snd_size = 1;
    exec.snd_data = &reg;
    exec.rcv_size = 6;
    exec.rcv_data = raw;

    ER result = tk_swri_dev(
        imu_i2c,
        TDN_I2C_EXEC,
        &exec,
        sizeof(T_I2C_EXEC),
        &asz
    );

    if (result < E_OK)
    {
        return false;
    }

    data->x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    data->y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    data->z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

    return true;
}

bool imu_calibrate(void)
{
    IMU_DATA data;
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    int32_t z_sum = 0;
    int successful_samples = 0;

    for (int sample = 0;
         sample < IMU_CALIBRATION_SAMPLES;
         sample++)
    {
        if (imu_read(&data))
        {
            x_sum += data.x;
            y_sum += data.y;
            z_sum += data.z;
            successful_samples++;
        }

        tk_dly_tsk(20);
    }

    if (successful_samples == 0)
    {
        return false;
    }

    x_offset = (int16_t)(x_sum / successful_samples);
    y_offset = (int16_t)(y_sum / successful_samples);
    z_offset = (int16_t)(z_sum / successful_samples);

    return true;
}

bool imu_read_calibrated(IMU_DATA *data)
{
    if (!imu_read(data))
    {
        return false;
    }

    data->x -= x_offset;
    data->y -= y_offset;
    data->z -= z_offset;

    return true;
}

bool imu_read_filtered(IMU_DATA *data)
{
    IMU_DATA sample;
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    int32_t z_sum = 0;
    int successful_samples = 0;

    for (int i = 0; i < IMU_FILTER_SAMPLES; i++)
    {
        if (imu_read_calibrated(&sample))
        {
            x_sum += sample.x;
            y_sum += sample.y;
            z_sum += sample.z;
            successful_samples++;
        }

        tk_dly_tsk(20);
    }

    if (successful_samples == 0)
    {
        return false;
    }

    data->x = (int16_t)(x_sum / successful_samples);
    data->y = (int16_t)(y_sum / successful_samples);
    data->z = (int16_t)(z_sum / successful_samples);

    return true;
}

bool imu_read_raw_filtered(IMU_DATA *data)
{
    IMU_DATA sample;
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    int32_t z_sum = 0;
    int successful_samples = 0;

    for (int i = 0; i < IMU_FILTER_SAMPLES; i++)
    {
        if (imu_read(&sample))
        {
            x_sum += sample.x;
            y_sum += sample.y;
            z_sum += sample.z;
            successful_samples++;
        }

        tk_dly_tsk(20);
    }

    if (successful_samples == 0)
    {
        return false;
    }

    data->x = (int16_t)(x_sum / successful_samples);
    data->y = (int16_t)(y_sum / successful_samples);
    data->z = (int16_t)(z_sum / successful_samples);

    return true;
}
#ifndef ESP32_ICM20948_H
#define ESP32_ICM20948_H

    #ifdef __cplusplus
extern "C"{
    #endif

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

#define ICM20948_I2C_ADDR_LOW   0x68
#define ICM20948_I2C_ADDR_HIGH  0x69
#define ICM20948_WHO_AM_I_REG   0x00
#define ICM20948_WHO_AM_I_ID    0xEA

typedef enum icm20948_accel_range_t
{
    ICM20948_ACEL_RANGE_2G      = 0,
    ICM20948_ACCEL_RANGE_4G,
    ICM20948_ACCEL_RANGE_8G,
    ICM20948_ACCEL_RANGE_16G,

};

typedef enum icm20948_gyro_range_t
{
    ICM20948_GYRO_RANGE_250DPS      = 0,
    ICM20948_GYRO_RANGE_500DPS,
    ICM20948_GYRO_RANGE_1000DPS,
    ICM20948_GYRO_RANGE_2000DPS,

};;

typedef struct icm20948_handle_t
{
    i2c_port_t              port;
    uint8_t                 address;
    icm20948_accel_range_t  accel_fs;
    icm20948_gyro_range_t   gyro_fs;
};;

esp_err_t icm20948_init( icm20948_handle_t* h,
                         i2c_port_t port,
                         uint8_t address,
                         icm20948_accel_range_t accel_fs,
                         icm20948_gyro_range_t gyro_fs);

esp_err_t icm20948_get_whoami( icm20948_handle_t* h,
                               uint8_t* id);

esp_err_t icm20948_read_acccel_raw( icm20948_handle_t* h,
                                    int16_t* ax,
                                    int16_t* ay,
                                    int16_t* az);

esp_err_t icm20948_read_acccel_g( icm20948_handle_t* h,
                                  float* ax,
                                  float* ay,
                                  float* az);

esp_err_t icm20948_read_gyro_raw( icm20948_handle_t* h,
                                  int16_t* gx,
                                  int16_t* gy,
                                  int16_t* gz);

esp_err_t icm20948_read_gyro_dps( icm20948_handle_t* h,
                                  float* gx,
                                  float* gy,
                                  float* gz);

esp_err_t icm20948_set_sleep( icm20948_handle_t* h,
                              bool enable);

#ifdef __cplusplus
    #ifdef __cplusplus
}
    #endif

#endif
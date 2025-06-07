#include "../include/esp32-icm20948.h"
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG             "ICM"
#define I2C_PORT        I2C_NUM_0
#define SDA_IO          23
#define SCL_IO          22
#define I2C_FREQ        100000
#define REG_BANK_SEL    0x7F
#define REG_PWR_MGMT_1  0x06
#define REG_PWR_MGMT_2  0x07

static uint16_t WriteDelay = 150 / portTICK_PERIOD_MS;

static i2c_config_t i2c_build_config(void);
static void i2c_init(void);
static esp_err_t bank(uint8_t Bank);
static esp_err_t reg_write(uint8_t Bank, uint8_t Register, uint8_t Value);
static esp_err_t reg_read(uint8_t Bank, uint8_t Register, void *d, size_t n);
static bool ping(uint8_t Address);
static esp_err_t icm_init(void);

esp_err_t icm20948_init(icm20948_handle_t* h, i2c_port_t port, uint8_t address, icm20948_accel_range_t accel_fs, icm20948_gyro_range_t gyro_fs)
{
    h->port = port;
    h->address = address;
    h->accel_fs = accel_fs;
    h->gyro_fs = gyro_fs;

    i2c_init();

    return icm_init();
}

esp_err_t icm20948_whoami(icm20948_handle_t* h, uint8_t* id)
{
    if (h == NULL || id == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return reg_read(0, ICM20948_WHO_AM_I_REG, id, 1);
}

esp_err_t icm20948_read_acccel_raw(icm20948_handle_t *h,int16_t* ax,int16_t* ay,int16_t* az)
{
    if (h == NULL || ax == NULL || ay == NULL || az == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t raw[6];
    esp_err_t err = reg_read(0, 0x2D, raw, 6);

    if (err != ESP_OK) return err;

    *ax = ((int16_t)raw[0] << 8) | raw[1];
    *ay = ((int16_t)raw[2] << 8) | raw[3];
    *az = ((int16_t)raw[4] << 8) | raw[5];

    return ESP_OK;
}

esp_err_t icm20948_read_gyro_raw(icm20948_handle_t* h,int16_t* gx,int16_t* gy,int16_t* gz)
{
    if (h == NULL || gx == NULL || gy == NULL || gz == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t raw[6];
    esp_err_t err = reg_read(0, 0x33, raw, 6);

    if (err != ESP_OK) return err;

    *gx = ((int16_t)raw[0] << 8) | raw[1];
    *gy = ((int16_t)raw[2] << 8) | raw[3];
    *gz = ((int16_t)raw[4] << 8) | raw[5];

    return ESP_OK;
}

esp_err_t icm20948_set_sleep(icm20948_handle_t* h, bool enable)
{
    uint8_t value = enable ? 0x40 : 0x01;
    return reg_write(0, REG_PWR_MGMT_1, value);
}

static i2c_config_t i2c_build_config(void)
{
    i2c_config_t Config;
    Config.mode = I2C_MODE_MASTER;
    Config.sda_io_num = SDA_IO;
    Config.scl_io_num = SCL_IO;
    Config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    Config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    Config.master.clk_speed = I2C_FREQ;
    return Config;
}

static void i2c_init(void)
{
    i2c_config_t Config = i2c_build_config();
    i2c_param_config(I2C_PORT, &Config);
    i2c_driver_install(I2C_PORT, Config.mode, 0, 0, 0);
}

static esp_err_t bank(uint8_t Bank)
{
    uint8_t Buffer[2];
    Buffer[0] = REG_BANK_SEL;
    Buffer[1] = (uint8_t)(Bank) << 4;
    return i2c_master_write_to_device(I2C_PORT, ICM20948_I2C_ADDR_HIGH, Buffer, sizeof(Buffer), WriteDelay);
}

static esp_err_t reg_write(uint8_t Bank, uint8_t Register, uint8_t Value)
{
    bank(Bank);

    uint8_t Buffer[2];
    Buffer[0] = Register;
    Buffer[1] = Value;

    return i2c_master_write_to_device(I2C_PORT, ICM20948_I2C_ADDR_HIGH, Buffer, 2, WriteDelay);
}

static esp_err_t reg_read(uint8_t Bank, uint8_t Register, void* d, size_t n)
{
    bank(Bank);
    return i2c_master_write_read_device(I2C_PORT, ICM20948_I2C_ADDR_HIGH, &Register, 1, d, n, WriteDelay);
}

static bool ping(uint8_t Address)
{
    i2c_cmd_handle_t Command = i2c_cmd_link_create();
    i2c_master_start(Command);
    i2c_master_write_byte(Command, (Address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(Command);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, Command, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(Command);
    return err == ESP_OK;
}

static esp_err_t icm_init(void)
{
    reg_write(0, REG_PWR_MGMT_1, 0x80);
    vTaskDelay(WriteDelay);
    reg_write(0, REG_PWR_MGMT_1, 0x01);
    reg_write(0, REG_PWR_MGMT_2, 0x00);
    reg_write(2, 0x14, 0x00);
    reg_write(2, 0x10, 0x00);
    return ESP_OK;
}

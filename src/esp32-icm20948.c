#include "../include/esp32-icm20948.h"
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG           "ICM"
#define ICM_ADDR      0x69
#define I2C_PORT      I2C_NUM_0
#define SDA_IO        23
#define SCL_IO        22
#define I2C_FREQ      100000
#define REG_BANK_SEL  0x7F

static uint16_t WriteDelay         = 100/portTICK_PERIOD_MS;
static uint16_t TaskDelay          = pdMS_TO_TICKS(20);
static float ACC_LSB_PER_G         = 2.0f / 32768.0f;
static float ROLL_DEG              = 57.29578f;

static esp_err_t reg8_write(uint8_t Bank, uint8_t Register, uint8_t Value);
static esp_err_t icm_init(void);
static i2c_config_t i2c_build_config(void);
static void i2c_init(void);
static esp_err_t bank(uint8_t Bank);
static esp_err_t reg8_write(uint8_t Bank, uint8_t Register, uint8_t Value);
static esp_err_t reg_read(uint8_t Bank, uint8_t Register, void *d, size_t n);
static void log_roll(float roll_deg, float ax_g, float ay_g, float az_g, int16_t ax, int16_t ay, int16_t az);
static bool ping(uint8_t Address);

esp_err_t icm20948_init(icm20948_handle_t* h,i2c_port_t port,uint8_t address,icm20948_accel_range_t accel_fs,icm20948_gyro_range_t gyro_fs)
{

}

esp_err_t icm20948_get_whoami(icm20948_handle_t* h,uint8_t* id)
{
    reg_read(0, 0x00, &id, 1);
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", id);
}

esp_err_t icm20948_read_acccel_raw(icm20948_handle_t* h,int16_t* ax,int16_t* ay,int16_t* az);

esp_err_t icm20948_read_acccel_g(icm20948_handle_t* h,float* ax,float* ay,float* az);

esp_err_t icm20948_read_gyro_raw(icm20948_handle_t* h,int16_t* gx,int16_t* gy,int16_t* gz);

esp_err_t icm20948_read_gyro_dps(icm20948_handle_t* h,float* gx,float* gy,float* gz);
esp_err_t icm20948_set_sleep(icm20948_handle_t* h,bool enable);

static esp_err_t icm_init(void)
{
    reg8_write(0, 0x06, 0x80);
    vTaskDelay(pdMS_TO_TICKS(150));
    reg8_write(0, 0x06, 0x01);
    reg8_write(0, 0x07, 0x00);
    reg8_write(2, 0x14, 0x00);
    reg8_write(2, 0x10, 0x00);

    return ESP_OK;
}

static i2c_config_t i2c_build_config(void)
{
    i2c_config_t Config =
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_IO,
        .scl_io_num = SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ
    };

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
    uint8_t Buffer[2] = { REG_BANK_SEL, (uint8_t)(Bank) << 4 };
    return i2c_master_write_to_device(I2C_PORT, ICM_ADDR, Buffer, sizeof(Buffer), WriteDelay);
}

static esp_err_t reg8_write(uint8_t Bank, uint8_t Register, uint8_t Value)
{
    bank(Bank);

    uint8_t Buffer[2] = {Register,Value};

    return i2c_master_write_to_device(I2C_PORT, ICM_ADDR, Buffer, 2, WriteDelay);
}

static esp_err_t reg_read(uint8_t Bank, uint8_t Register, void *d, size_t n)
{
    bank(Bank);

    return i2c_master_write_read_device(I2C_PORT, ICM_ADDR, &Register, 1, d, n, WriteDelay);
}

static void log_roll(float roll_deg, float ax_g, float ay_g, float az_g, int16_t ax, int16_t ay, int16_t az)
{
    ESP_LOGI(TAG, "Roll %.2f | A[g] %.5f,%.5f,%.5f (raw %6d,%6d,%6d)", roll_deg, ax_g, ay_g, az_g, ax, ay, az);
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
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#define TAG "i2c_agmt"

/* i2c bus configuration */
i2c_config_t conf = {
	.mode = I2C_MODE_MASTER,
	.sda_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SDA,
	.sda_pullup_en = GPIO_PULLUP_ENABLE,
	.scl_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SCL,
	.scl_pullup_en = GPIO_PULLUP_ENABLE,
	.master.clk_speed = 400000,
	.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
};

/* ICM 20948 configuration */
icm0948_config_i2c_t icm_config = {
	.i2c_port = I2C_NUM_0,
	.i2c_addr = ICM_20948_I2C_ADDR_AD1
};


void print_agmt(icm20948_agmt_t agmt)
{
  	ESP_LOGI(TAG, "Acc: [ %d, %d, %d ] Gyr: [ %d, %d, %d ] Mag: [ %d, %d, %d ] Tmp: [ %d ]", 
		agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z,
		agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z,
		agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z,
		agmt.tmp.val
	);
}

void app_main(void)
{
	icm20948_device_t icm;

	/* setup i2c */
	ESP_ERROR_CHECK(i2c_param_config(icm_config.i2c_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(icm_config.i2c_port, conf.mode, 0, 0, 0));
	
	/* setup ICM20948 device */
	icm20948_init_i2c(&icm, &icm_config);
		
	/* check ID */
    while (icm20948_check_id(&icm) != ICM_20948_STAT_OK)
	{
		ESP_LOGE(TAG, "check id failed");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
	ESP_LOGI(TAG, "check id passed");

	/* check whoami */
	icm20948_status_e stat = ICM_20948_STAT_ERR;
	uint8_t whoami = 0x00;
	while ((stat != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI))
	{
		whoami = 0x00;
		stat = icm20948_get_who_am_i(&icm, &whoami);
		ESP_LOGE(TAG, "whoami does not match (0x %d). Halting...", whoami);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	/* Here we are doing a SW reset to make sure the device starts in a known state */
	icm20948_sw_reset(&icm);
	vTaskDelay(250 / portTICK_PERIOD_MS);

	icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

	// Set Gyro and Accelerometer to a particular sample mode
	// optiona: SAMPLE_MODE_CONTINUOUS. SAMPLE_MODE_CYCLED
	icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS); 

	// Set full scale ranges for both acc and gyr
	icm20948_fss_t myfss;
	myfss.a = GPM_2;   // (icm20948_accel_config_fs_sel_e)
	myfss.g = DPS_250; // (icm20948_gyro_config_1_fs_sel_e)
	icm20948_set_full_scale(&icm, sensors, myfss);

	// Set up DLPF configuration
	icm20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = ACC_D473BW_N499BW;
	myDLPcfg.g = GYR_D361BW4_N376BW5;
	icm20948_set_dlpf_cfg(&icm, sensors, myDLPcfg);

	// Choose whether or not to use DLPF
	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_ACC, false);
	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_GYR, false);

	// Now wake the sensor up
	icm20948_sleep(&icm, false);
	icm20948_low_power(&icm, false);

    /* loop */
    while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);

		icm20948_agmt_t agmt; // = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
		if (icm20948_get_agmt(&icm, &agmt) == ICM_20948_STAT_OK) {
			print_agmt(agmt);
		} else {
			ESP_LOGE(TAG, "Uh oh");
		}        
    }
}

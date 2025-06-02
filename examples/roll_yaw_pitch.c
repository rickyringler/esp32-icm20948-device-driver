#include "../include/esp32-icm20948.h"

void app_main(void)
{
    i2c_init();
    icm_init();

    uint8_t id;
    reg_read(0, 0x00, &id, 1);

    while(1)
    {
        uint8_t Raw[6];
        reg_read(0, 0x2D, Raw, 6);

        int16_t ax = (Raw[0]<<8) | Raw[1];
        int16_t ay = (Raw[2]<<8) | Raw[3];
        int16_t az = (Raw[4]<<8) | Raw[5];

        float ax_g = ax * ACC_LSB_PER_G;
        float ay_g = ay * ACC_LSB_PER_G;
        float az_g = az * ACC_LSB_PER_G;
        float roll_deg = atan2f(ay_g, az_g) * ROLL_DEG;

        vTaskDelay(TaskDelay);

    }
}
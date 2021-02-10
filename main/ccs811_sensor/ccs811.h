/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 8th 2021
 **/

#ifndef _CCS811_
#define _CCS811_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "circular_buffer/circular_buffer.h"

#define CCS811_SENSOR_ADDR                        0x5A   /*!< slave address for ccs811 sensor */
#define CCS811_DELAY                              100

#define CCS811_REG_STATUS                         0x00
#define CCS811_REG_MEAS_MODE                      0x01
#define CCS811_REG_ALG_RESULT_DATA                0x02
#define CCS811_REG_ENV_DATA                       0x05
#define CCS811_READ_CO2                           0xF3
#define CCS811_BOOTLOADER_APP_START               0xF4
#define CCS811_REG_SW_RESET                       0xFF
#define CCS811_REG_SW_RESET_VALUES_N              4
const static uint8_t CCS811_REG_SW_RESET_VALUES[CCS811_REG_SW_RESET_VALUES_N] = {0x11, 0xE5, 0x72, 0x8A};

typedef enum{
    eMode0, //Idle (Measurements are disabled in this mode)
    eMode1, //Constant power mode, IAQ measurement every second
    eMode2, //Pulse heating mode IAQ measurement every 10 seconds
    eMode3, //Low power pulse heating mode IAQ measurement every 60 seconds
    eMode4  //Constant power mode, sensor measurement every 250ms 1xx: Reserved modes (For future use)
} eDRIVE_MODE_t;

static struct CircularBuffer co2_buffer;

void soft_reset();
void init_ccs811(int thresh, int interrupt, eDRIVE_MODE_t mode);
void ccs811_task(void *arg);
void update_co2_char(void *arg);
void set_environment_vars(float temperature, float humidity);
esp_err_t get_co2(int i2c_num, int *co2);

#endif
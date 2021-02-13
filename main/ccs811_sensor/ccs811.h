/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 8th 2021
 **/

#ifndef _CCS811_
#define _CCS811_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "circular_buffer/circular_buffer.h"

#define CCS811_FISRT_ENV_VARS_T                   6  
#define CCS811_DELAY                              100
#define CCS811_HW_ID                              0x81
#define CCS811_SENSOR_ADDR                        0x5A   /*!< slave address for ccs811 sensor */

#define CCS811_REG_STATUS                         0x00
#define CCS811_REG_MEAS_MODE                      0x01
#define CCS811_REG_ALG_RESULT_DATA                0x02
#define CCS811_REG_ENV_DATA                       0x05
#define CCS811_REG_HW_ID                          0x20
#define CCS811_REG_ERROR_ID                       0xE0
#define CCS811_READ_CO2                           0xF3
#define CCS811_BOOTLOADER_APP_START               0xF4

typedef enum{
    eMode0, //Idle (Measurements are disabled in this mode)
    eMode1, //Constant power mode, IAQ measurement every second
    eMode2, //Pulse heating mode IAQ measurement every 10 seconds
    eMode3, //Low power pulse heating mode IAQ measurement every 60 seconds
    eMode4  //Constant power mode, sensor measurement every 250ms 1xx: Reserved modes (For future use)
} eDRIVE_MODE_t;

typedef enum
{
    CCS811_DEFAULT,
    CO2_SAMPLE,
    CO2_SAMPLE_FREQ,
    CO2_ENABLE,
    CO2_UPDATE_ENV_VARS,
} ccs811_event_t;

typedef struct
{
    uint32_t *co2_samp_freq;
    QueueHandle_t event_queue;
    SemaphoreHandle_t i2c_sem;
} ccs811_args_t;

typedef struct
{
    uint32_t *co2_samp_freq; // seconds
    QueueHandle_t event_queue;
} ccs811_control_args_t;

static struct CircularBuffer co2_buffer;
static ccs811_event_t ccs811_ev;

int init_ccs811(int thresh, int interrupt, eDRIVE_MODE_t mode,  SemaphoreHandle_t i2c_sem);
void ccs811_task(void *arg);
float get_co2_moving_average();
void update_co2_char(void *arg);
void set_environment_vars(float temperature, float humidity);
esp_err_t get_co2(int i2c_num, int *co2, SemaphoreHandle_t i2c_sem);
static void chrono_sample_co2(void *arg);
bool are_co2_samples();
static void chrono_set_environment(void *arg);

#endif
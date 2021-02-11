/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 7th 2021
 **/

#ifndef _SI7021_
#define _SI7021_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "circular_buffer/circular_buffer.h"

#define MIN(a,b) (((a)<(b))?(a):(b))

#define SI7021_SENSOR_ADDR 0x40   /*!< slave address for SI7021 sensor */
#define SI7021_READ_TEMP 0xF3    /*!< READ op with no stretching  F3*/
#define SI7021_READ_HUMIDITY 0xF5
#define SI7021_DELAY 30

typedef enum
{
    SI7021_DEFAULT,
    TEMP_SAMPLE,
    TEMP_SAMPLE_FREQ,
    TEMP_ENABLE,
    HUM_SAMPLE,
    HUM_SAMPLE_FREQ,
    HUM_ENABLE,
} si7021_event_t;

typedef struct
{
    uint8_t *temp_samp_freq;
    uint8_t *hum_samp_freq;
    QueueHandle_t event_queue;
    SemaphoreHandle_t i2c_sem;
} si7021_args_t;

static struct CircularBuffer tBuffer;
static struct CircularBuffer hBuffer;
static si7021_event_t si7021_ev;

void si7021_task(void *arg);
void update_temperature_char(void *arg);
void update_humidity_char(void *arg);
static void chrono_sample_temp(void *arg);
static void chrono_sample_hum(void *arg);

#endif
/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 7th 2021
 **/

#ifndef _SI7021_
#define _SI7021_

#include "circular_buffer/circular_buffer.h"

#define MIN(a,b) (((a)<(b))?(a):(b))

#define SAMPLE_FREQ_T 5
#define SAMPLE_FREQ_H 2 /*!< delay time between different test items */

#define SI7021_SENSOR_ADDR 0x40   /*!< slave address for SI7021 sensor */
#define SI7021_READ_TEMP 0xF3    /*!< READ op with no stretching  F3*/
#define SI7021_READ_HUMIDITY 0xF5
#define SENSOR_DELAY 30

static struct CircularBuffer tBuffer;
static struct CircularBuffer hBuffer;

void si7021_task(void *arg);
void update_temperature_char(void *arg);
void update_humidity_char(void *arg);

#endif
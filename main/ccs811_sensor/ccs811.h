/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 8th 2021
 **/

#ifndef _CCS811_
#define _CCS811_

#define CCS811_SENSOR_ADDR 0x5A   /*!< slave address for ccs811 sensor */
#define CCS811_READ_CO2 0xF3
#define CCS811_DELAY 100

// typedef enum{
//     eMode0, //Idle (Measurements are disabled in this mode)
//     eMode1, //Constant power mode, IAQ measurement every second
//     eMode2, //Pulse heating mode IAQ measurement every 10 seconds
//     eMode3, //Low power pulse heating mode IAQ measurement every 60 seconds
//     eMode4  //Constant power mode, sensor measurement every 250ms 1xx: Reserved modes (For future use)
// } eDRIVE_MODE_t;

// static struct CircularBuffer buffer;

// void init_ccs811();
// void ccs811_task(void *arg);
// void update_co2_char(void *arg);

#endif
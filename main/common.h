/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 11th 2021
 **/

#ifndef _COMMON_
#define _COMMON_

#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_sntp.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_task_wdt.h"

#include "gatt_table/gatt_table.h"
#include "i2c_controller/i2c_controller.h"
#include "si7021_sensor/si7021.h"
#include "ccs811_sensor/ccs811.h"
#include "wifi_controller/wifi_controller.h"
#include "ble_beacon_counter/ble_beacon_counter.h"

#define SNTP_TRIES 10

uint64_t get_time_micros(uint32_t t);
void go_low_energy_mode();
uint64_t get_seconds_among_hours(uint8_t start, uint8_t end);
static void initialize_sntp(void);
bool sntp_adjust_time(void);
void configure_vfs(void);
int log_printf(const char *fmt, va_list args);

#endif
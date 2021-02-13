/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 13th 2021
 **/

#ifndef _WIFI_CONT_
#define _WIFI_CONT_

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

int wifi_controller_start(void);
int wifi_controller_stop(void);

#endif
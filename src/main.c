/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 31th 2021
 **/

#include "common.h"
#include "gatt/gatt_table.h"

void app_main(void) {

    ESP_LOGI(LOG_TAG, "Configuring GATT server");
    configure_gatt_server();
}
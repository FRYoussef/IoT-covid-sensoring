/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 31th 2021
 **/

#include "gatt_table.h"

void app_main(void) {

    ESP_LOGI(CONFIG_LOG_TAG, "Configuring GATT server");
    configure_gatt_server();
}
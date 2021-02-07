/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 31th 2021
 **/

#include "nvs_flash.h"

//#include "gatt_table/gatt_table.h"
#include "i2c_controller/i2c_controller.h"
#include "si7021_sensor/si7021.h"

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());

    xTaskCreatePinnedToCore(&si7021_task, "si7021_task", 1024 * 3, NULL, 5, NULL, 1);

    // esp_err_t ret;

    // /* Initialize NVS. */
    // ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK( ret );
    
    // ESP_LOGI(CONFIG_LOG_TAG, "Configuring GATT server");
    // configure_gatt_server();
    // ESP_LOGI(CONFIG_LOG_TAG, "GATT server well configured");
}
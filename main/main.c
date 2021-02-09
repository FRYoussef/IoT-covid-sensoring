/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 31th 2021
 **/

#include "nvs_flash.h"

#include "gatt_table/gatt_table.h"
#include "i2c_controller/i2c_controller.h"
#include "si7021_sensor/si7021.h"
#include "ccs811_sensor/ccs811.h"

void app_main(void) {
    esp_err_t ret;

    ESP_ERROR_CHECK(i2c_master_init());

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    QueueHandle_t si7021_queue = xQueueCreate(5, sizeof(si7021_event_t));
    si7021_args_t si7021_args = {
        .temp_samp_freq = temp_ccc,
        .hum_samp_freq = hum_ccc,
        .event_queue = si7021_queue,
    };

    ESP_LOGI(CONFIG_LOG_TAG, "Configuring GATT server");
    configure_gatt_server(si7021_queue);
    ESP_LOGI(CONFIG_LOG_TAG, "GATT server well configured");

    xTaskCreatePinnedToCore(&si7021_task, "si7021_task", 1024 * 4, (void*)&si7021_args, uxTaskPriorityGet(NULL), NULL, 1);
    xTaskCreatePinnedToCore(&ccs811_task, "ccs811_task", 1024 * 2, NULL, uxTaskPriorityGet(NULL), NULL, 1);

    while(1) { vTaskDelay(1000); }
}
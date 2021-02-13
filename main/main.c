/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 31th 2021
 **/

#include "common.h"


void go_low_energy_mode(){
#ifdef CONFIG_DEEP_SLEEP
    esp_sleep_enable_timer_wakeup(get_time_micros(30));
    esp_deep_sleep_start();
#endif
}


void print_wakeup_cause(esp_sleep_wakeup_cause_t cause){
    switch (cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI(TAG, "Processor has been waked up by a timer.");
        break;
    
    default:
        ESP_LOGI(TAG, "Processor has been waked up by an undefined cause.");
        break;
    }
}


uint64_t get_time_micros(uint32_t t) {
    return 1000000 * t;
}


void app_main(void) {
#ifdef CONFIG_DEEP_SLEEP
    print_wakeup_cause(esp_sleep_get_wakeup_cause());
#endif

    // power management configuration
    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = CONFIG_MAX_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_MIN_CPU_FREQ_MHZ,
        .light_sleep_enable = true };
    esp_pm_configure(&pm_config);

    esp_err_t ret;

    ESP_ERROR_CHECK(i2c_master_init());

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    SemaphoreHandle_t i2c_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(i2c_sem);
    
    QueueHandle_t si7021_queue = xQueueCreate(5, sizeof(si7021_event_t));
    uint32_t temp_samp_freq = temp_ccc[1] << 8 | temp_ccc[0];
    uint32_t hum_samp_freq = hum_ccc[1] << 8 | hum_ccc[0];
    si7021_args_t si7021_args = {
        .temp_samp_freq = &temp_samp_freq,
        .hum_samp_freq = &hum_samp_freq,
        .event_queue = si7021_queue,
        .i2c_sem = i2c_sem,
    };
    si7021_control_args_t si7021_control = {
        .temp_samp_freq = &temp_samp_freq,
        .hum_samp_freq = &hum_samp_freq,
        .event_queue = si7021_queue,
    };

    QueueHandle_t ccs811_queue = xQueueCreate(5, sizeof(ccs811_event_t));
    uint32_t co2_samp_freq = co2_ccc[1] << 8 | co2_ccc[0];
    ccs811_args_t ccs811_args = {
        .co2_samp_freq = &co2_samp_freq,
        .event_queue = ccs811_queue,
        .i2c_sem = i2c_sem,
    };
    ccs811_control_args_t ccs811_control = {
        .co2_samp_freq = &co2_samp_freq,
        .event_queue = ccs811_queue,
    };

    ESP_LOGI(CONFIG_LOG_TAG, "Configuring GATT server");
    configure_gatt_server(&si7021_control, &ccs811_control);
    ESP_LOGI(CONFIG_LOG_TAG, "GATT server well configured");

    xTaskCreatePinnedToCore(&si7021_task, "si7021_task", 1024 * 4, (void*)&si7021_args, uxTaskPriorityGet(NULL), NULL, 1);
    xTaskCreatePinnedToCore(&ccs811_task, "ccs811_task", 1024 * 2, (void*)&ccs811_args, uxTaskPriorityGet(NULL), NULL, 0);

    while(1) { 
        vTaskDelay(pdMS_TO_TICKS(10000));
        go_low_energy_mode();
    }
}
/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 31th 2021
 **/

#include "common.h"

// Mount path for the partition
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
const char *vfs_path = "/spiflash";


void go_low_energy_mode(uint64_t micros){
#ifdef CONFIG_DEEP_SLEEP
    ESP_LOGI(CONFIG_LOG_TAG, "Going to deep sleep mode.");
    esp_sleep_enable_timer_wakeup(micros);
    esp_deep_sleep_start();
#endif
}

uint64_t get_seconds_among_hours(uint8_t start, uint8_t end){
    if(end < start)
        end += 24;

    return (uint64_t)((end - start) * 3600);
}


uint64_t get_time_micros(uint32_t t) {
    return 1000000 * t;
}


void print_wakeup_cause(esp_sleep_wakeup_cause_t cause){
    switch (cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI(CONFIG_LOG_TAG, "Restarted from deep sleep.");
        break;
    default:
        break;
    }
}


static void initialize_sntp(void) {
    ESP_LOGI(CONFIG_LOG_TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}


bool sntp_adjust_time(void) {
    if(wifi_controller_start())
        return false;

    initialize_sntp();

    int retry = 0;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < SNTP_TRIES) {
        ESP_LOGI(CONFIG_LOG_TAG, "Waiting for system time to be set... (%d/%d)", retry, SNTP_TRIES);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    wifi_controller_stop();

    if(retry > SNTP_TRIES) {
        ESP_LOGE(CONFIG_LOG_TAG, "SNTP not configured.");
        return false;
    }
    else {
        ESP_LOGI(CONFIG_LOG_TAG, "SNTP well configured.");
        return true;
    }
}


int log_printf(const char *fmt, va_list args) {
    int result;
    FILE *f = fopen("/spiflash/log.txt", "a");

    result = vfprintf(f, fmt, args);

    fclose(f);
    return result;
}


void configure_vfs(void) {
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 2,
            .format_if_mount_failed = false,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount(vfs_path, "storage", &mount_config, &s_wl_handle);

    if (err != ESP_OK) {
        ESP_LOGE(CONFIG_LOG_TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
    
    esp_log_set_vprintf(&log_printf);
}


void app_main(void) {
    uint64_t deep_micros;
    char strftime_buf[64];
    time_t now;
    struct tm timeinfo;

    // power management configuration
    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = CONFIG_MAX_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_MIN_CPU_FREQ_MHZ,
        .light_sleep_enable = true };
    esp_pm_configure(&pm_config);

    /* Initialize NVS. */
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // configure log output
    configure_vfs();

    uint32_t main_sleep;
#ifdef CONFIG_DEEP_SLEEP
    print_wakeup_cause(esp_sleep_get_wakeup_cause());

    // set time with sntp
    if(sntp_adjust_time()) {
        time(&now);
        setenv("TZ", "UTC", 1);
        tzset();
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(CONFIG_LOG_TAG, "The current date/time in Madrid is: %s", strftime_buf);
        main_sleep = get_seconds_among_hours(timeinfo.tm_hour, CONFIG_DEEP_SLEEP_START);
        main_sleep -= (timeinfo.tm_min * 60) + timeinfo.tm_sec;
        ESP_LOGI(CONFIG_LOG_TAG, "Deep mode will be launched in %f h", (float)(main_sleep/3600));
    }
    else {
        main_sleep = 8;
        ESP_LOGW(CONFIG_LOG_TAG, "Cannot connect with sntp server");
        ESP_LOGI(CONFIG_LOG_TAG, "Deep mode will be launched in %d h", main_sleep);
        main_sleep = get_seconds_among_hours(0, main_sleep);
    }
#endif

    ESP_ERROR_CHECK(i2c_master_init());

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

    // Configuring GATT server
    configure_gatt_server(&si7021_control, &ccs811_control);

    xTaskCreatePinnedToCore(&si7021_task, "si7021_task", 1024 * 3, (void*)&si7021_args, uxTaskPriorityGet(NULL), NULL, 1);
    xTaskCreatePinnedToCore(&ccs811_task, "ccs811_task", 1024 * 2, (void*)&ccs811_args, uxTaskPriorityGet(NULL), NULL, 0);

    while(1) { 
        vTaskDelay(pdMS_TO_TICKS(main_sleep*1000));
#ifdef CONFIG_DEEP_SLEEP
        time(&now);
        localtime_r(&now, &timeinfo);
        deep_micros = get_seconds_among_hours(timeinfo.tm_hour, CONFIG_DEEP_SLEEP_STOP);
        deep_micros -= (timeinfo.tm_min * 60) + timeinfo.tm_sec;
        go_low_energy_mode(get_time_micros(deep_micros));

        // close vfs
        esp_vfs_fat_spiflash_unmount(vfs_path, s_wl_handle);
#endif
    }
}
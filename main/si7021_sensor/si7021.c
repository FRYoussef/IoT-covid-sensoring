/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 2nd 2021
 **/

#include <time.h>
#include "i2c_controller/i2c_controller.h"
#include "si7021.h"

esp_err_t get_temperature(i2c_port_t i2c_num, float *temperature){
    int ret;
    uint8_t sensor_data_h, sensor_data_l;

    ret = i2c_master_read_from(SI7021_SENSOR_ADDR, SENSOR_DELAY,
        i2c_num, &sensor_data_h, &sensor_data_l, SI7021_READ_TEMP);

    uint16_t bytes;
    bytes = (sensor_data_h << 8 | sensor_data_l);

    *temperature = ( (175.72 * bytes) / 65536.0 ) - 46.85;

    return ret;
}

esp_err_t get_humidity(i2c_port_t i2c_num, float *humidity){
    int ret;
    uint8_t sensor_data_h, sensor_data_l;

    ret = i2c_master_read_from(SI7021_SENSOR_ADDR, SENSOR_DELAY,
        i2c_num, &sensor_data_h, &sensor_data_l, SI7021_READ_HUMIDITY);

    uint16_t bytes;
    bytes = (sensor_data_h << 8 | sensor_data_l);

    *humidity = MIN(100.0, bytes * 125.0 / 65536.0 - 6.0);

    return ret;
}


void si7021_task(void *arg) {
    int ret;
    float temperature, humidity;
    double meanT = 0, meanH = 0;
    uint32_t timeT = time(NULL), timeH = time(NULL);

    init_buffer(&tBuffer, CONFIG_TEMP_WINDOW_SIZE);
    init_buffer(&hBuffer, CONFIG_HUM_WINDOW_SIZE);

    ESP_LOGI(CONFIG_LOG_TAG, "Started si7021 task");
    while (1) {

        if((time(NULL) - timeT) >= SAMPLE_FREQ_T){
            timeT = time(NULL);

            for(int i = 0; i < CONFIG_TEMP_N_SAMPLES; i++) {
                ret = get_temperature(I2C_MASTER_NUM, &temperature);

                if (ret == ESP_ERR_TIMEOUT)
                    ESP_LOGE(CONFIG_LOG_TAG, "I2C Timeout for temperature sensor");
                else if (ret == ESP_OK)
                    meanT += temperature;
                else
                    ESP_LOGW(CONFIG_LOG_TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
            }

            meanT /= CONFIG_TEMP_N_SAMPLES;
            add_element(&tBuffer, meanT);
            meanT = 0;
        }

        if((time(NULL) - timeH) >= SAMPLE_FREQ_H){
            timeH = time(NULL);

            for(int i = 0; i < CONFIG_HUM_N_SAMPLES; i++) {
                ret = get_humidity(I2C_MASTER_NUM, &humidity);

                if (ret == ESP_ERR_TIMEOUT)
                    ESP_LOGE(CONFIG_LOG_TAG, "I2C Timeout for humidity measure");
                else if (ret == ESP_OK)
                    meanH += humidity;
                else
                    ESP_LOGW(CONFIG_LOG_TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
            }

            meanH /= CONFIG_HUM_N_SAMPLES;
            add_element(&hBuffer, meanH);
            meanH = 0;
        }

        vTaskDelay((MIN(SAMPLE_FREQ_T, SAMPLE_FREQ_H) * 0.25)  / portTICK_RATE_MS);
    }

    free_buffer(&tBuffer);
    free_buffer(&hBuffer);
    vTaskDelete(NULL);
}


void sendTempCallback(void *arg) {
    //printf("temp val: %.02f [ºC]\n", get_element(&tBuffer));
}


void sendHumCallback(void *arg) {
    //printf("hum val: %.02f %%\n", get_element(&hBuffer));
}

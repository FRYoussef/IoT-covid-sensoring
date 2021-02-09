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

    ret = i2c_master_read_from(SI7021_SENSOR_ADDR, SI7021_DELAY,
        i2c_num, &sensor_data_h, &sensor_data_l, SI7021_READ_TEMP);

    uint16_t bytes;
    bytes = (sensor_data_h << 8 | sensor_data_l);

    *temperature = ( (175.72 * bytes) / 65536.0 ) - 46.85;

    return ret;
}

esp_err_t get_humidity(i2c_port_t i2c_num, float *humidity){
    int ret;
    uint8_t sensor_data_h, sensor_data_l;

    ret = i2c_master_read_from(SI7021_SENSOR_ADDR, SI7021_DELAY,
        i2c_num, &sensor_data_h, &sensor_data_l, SI7021_READ_HUMIDITY);

    uint16_t bytes;
    bytes = (sensor_data_h << 8 | sensor_data_l);

    *humidity = MIN(100.0, bytes * 125.0 / 65536.0 - 6.0);

    return ret;
}


void si7021_task(void *arg) {
    int ret;
    float temperature, humidity;
    double mean = 0;
    uint32_t timeT = time(NULL), timeH = time(NULL);
    uint8_t temp_ccc[2] = {0x05, 0x00};
    uint8_t hum_ccc[2] = {0x05, 0x00};
    uint32_t sample_freq_t, sample_freq_h;

    init_buffer(&tBuffer, CONFIG_TEMP_WINDOW_SIZE);
    init_buffer(&hBuffer, CONFIG_HUM_WINDOW_SIZE);

    ESP_LOGI(CONFIG_LOG_TAG, "Started si7021 task");
    while (1) {

        // GATT var for temperature frequency sampling
        sample_freq_t = temp_ccc[1] << 8 | temp_ccc[0];
        if((time(NULL) - timeT) >= sample_freq_t){
            timeT = time(NULL);

            for(int i = 0; i < CONFIG_TEMP_N_SAMPLES; i++) {
                ret = get_temperature(I2C_MASTER_NUM, &temperature);

                if (ret == ESP_ERR_TIMEOUT)
                    ESP_LOGE(CONFIG_LOG_TAG, "I2C Timeout for temperature sensor");
                else if (ret == ESP_OK)
                    mean += temperature;
                else
                    ESP_LOGW(CONFIG_LOG_TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
            }

            mean /= CONFIG_TEMP_N_SAMPLES;
            add_element(&tBuffer, mean);
            mean = 0;
        }

        sample_freq_h = hum_ccc[1] << 8 | hum_ccc[0];
        if((time(NULL) - timeH) >= sample_freq_h){
            timeH = time(NULL);

            for(int i = 0; i < CONFIG_HUM_N_SAMPLES; i++) {
                ret = get_humidity(I2C_MASTER_NUM, &humidity);

                if (ret == ESP_ERR_TIMEOUT)
                    ESP_LOGE(CONFIG_LOG_TAG, "I2C Timeout for humidity measure");
                else if (ret == ESP_OK)
                    mean += humidity;
                else
                    ESP_LOGW(CONFIG_LOG_TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
            }

            mean /= CONFIG_HUM_N_SAMPLES;
            add_element(&hBuffer, mean);
            mean = 0;
        }

        vTaskDelay((MIN(sample_freq_t, sample_freq_h) * 0.5)  / portTICK_RATE_MS);
    }

    free_buffer(&tBuffer);
    free_buffer(&hBuffer);
    vTaskDelete(NULL);
}


void update_temperature_char(void *arg) {
    uint8_t *temp = (uint8_t *)arg;
    float mean = 0;

    for(int i = 0; i < tBuffer.counter; i++)
        mean += get_element(&tBuffer);
    
    mean /= tBuffer.counter;

    uint8_t intpart = (uint8_t)mean;
    float dec = (mean - intpart) * 100; // get just 2 decimal
    uint8_t decpart = (uint8_t)dec;

    temp[0] = intpart;
    temp[1] = decpart;
}


void update_humidity_char(void *arg) {
    uint8_t *hum = (uint8_t *)arg;
    float mean = 0;

    for(int i = 0; i < hBuffer.counter; i++)
        mean += get_element(&hBuffer);
    
    mean /= hBuffer.counter;

    uint8_t intpart = (uint8_t)mean;
    float dec = (mean - intpart) * 100; // get just 2 decimal
    uint8_t decpart = (uint8_t)dec;

    hum[0] = intpart;
    hum[1] = decpart;
}

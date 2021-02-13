/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 2nd 2021
 **/

#include "esp_timer.h"

#include "i2c_controller/i2c_controller.h"
#include "si7021_sensor/si7021.h"
#include "common.h"

esp_err_t get_temperature(i2c_port_t i2c_num, float *temperature, SemaphoreHandle_t i2c_sem){
    int ret;
    uint8_t buffer[2];
    uint8_t code[1] = {SI7021_READ_TEMP};
    BaseType_t q_ready;

    // get i2c
    do { q_ready = xSemaphoreTake(i2c_sem, 1000); } while(!q_ready);
    
    ret = i2c_master_write_read_from(SI7021_SENSOR_ADDR, SI7021_DELAY,
        i2c_num, buffer, 2, code, 1);

    // release i2c
    xSemaphoreGive(i2c_sem);

    uint16_t bytes;
    bytes = (buffer[0] << 8 | buffer[1]);

    *temperature = ( (175.72 * bytes) / 65536.0 ) - 46.85;

    return ret;
}

esp_err_t get_humidity(i2c_port_t i2c_num, float *humidity, SemaphoreHandle_t i2c_sem){
    int ret;
    uint8_t buffer[2];
    uint8_t code[1] = {SI7021_READ_HUMIDITY};
    BaseType_t q_ready;

    // get i2c
    do { q_ready = xSemaphoreTake(i2c_sem, 1000); } while(!q_ready);

    ret = i2c_master_write_read_from(SI7021_SENSOR_ADDR, SI7021_DELAY,
        i2c_num, buffer, 2, code, 1);

    // release i2c
    xSemaphoreGive(i2c_sem);

    uint16_t bytes;
    bytes = (buffer[0] << 8 | buffer[1]);

    *humidity = MIN(100.0, bytes * 125.0 / 65536.0 - 6.0);

    return ret;
}


void si7021_task(void *arg) {
    si7021_args_t *params = (si7021_args_t *) arg;
    si7021_event_t ev = SI7021_DEFAULT;
    int ret;
    float temperature, humidity;
    double mean = 0;
    esp_timer_handle_t timer_temp, timer_hum;
    BaseType_t q_ready;
    bool temp_enb = true, hum_enb = true;

    init_buffer(&tBuffer, CONFIG_TEMP_WINDOW_SIZE);
    init_buffer(&hBuffer, CONFIG_HUM_WINDOW_SIZE);

    // timers configuration
    const esp_timer_create_args_t chrono_args_t = {
        .callback = &chrono_sample_temp,
        .name = "timer_sample_temperature",
        .arg = (void *)&params->event_queue,
    };
    esp_timer_create(&chrono_args_t, &timer_temp);

    const esp_timer_create_args_t chrono_args_h = {
        .callback = &chrono_sample_hum,
        .name = "timer_sample_humidity",
        .arg = (void *)&params->event_queue,
    };
    esp_timer_create(&chrono_args_h, &timer_hum);

    esp_timer_start_periodic(timer_temp, get_time_micros(*params->temp_samp_freq));
    esp_timer_start_periodic(timer_hum, get_time_micros(*params->hum_samp_freq));
    ESP_LOGI(CONFIG_LOG_TAG, "si7021 task started");

    while (1) {
        if(ev == TEMP_SAMPLE && temp_enb){
            for(int i = 0; i < CONFIG_TEMP_N_SAMPLES; i++) {
                ret = get_temperature(I2C_MASTER_NUM, &temperature, params->i2c_sem);

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
        else if(ev == HUM_SAMPLE && hum_enb){
            for(int i = 0; i < CONFIG_HUM_N_SAMPLES; i++) {
                ret = get_humidity(I2C_MASTER_NUM, &humidity, params->i2c_sem);

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
        else if (ev == TEMP_ENABLE){
            temp_enb = !temp_enb;
            if(temp_enb)
                ESP_LOGI(CONFIG_LOG_TAG, "Temperature enabled");
            else
                ESP_LOGI(CONFIG_LOG_TAG, "Temperature disabled");
        }
        else if (ev == HUM_ENABLE){
            hum_enb = !hum_enb;
            if(hum_enb)
                ESP_LOGI(CONFIG_LOG_TAG, "Humidity enabled");
            else
                ESP_LOGI(CONFIG_LOG_TAG, "Humidity disabled");
        }
        else if (ev == TEMP_SAMPLE_FREQ) {
            esp_timer_stop(timer_temp);
            ESP_LOGI(CONFIG_LOG_TAG, "Changed temperature sample to each %d s", *params->temp_samp_freq);
            esp_timer_start_periodic(timer_temp, get_time_micros(*params->temp_samp_freq));
        }
        else if (ev == HUM_SAMPLE_FREQ) {
            esp_timer_stop(timer_hum);
            ESP_LOGI(CONFIG_LOG_TAG, "Changed humidity sample to each %d s", *params->hum_samp_freq);
            esp_timer_start_periodic(timer_hum, get_time_micros(*params->hum_samp_freq));
        }
        
        do {q_ready = xQueueReceive(params->event_queue, (void *) &ev, 2000);} while(!q_ready);
    }

    free_buffer(&tBuffer);
    free_buffer(&hBuffer);
    vTaskDelete(NULL);
}


float get_temp_moving_average(){
    float mean = 0;

    for(int i = 0; i < tBuffer.counter; i++)
        mean += get_element(&tBuffer);
    
    mean /= tBuffer.counter;
    return mean;
}


void update_temperature_char(void *arg) {
    uint8_t *temp = (uint8_t *)arg;
    float mean = get_temp_moving_average();
    uint8_t intpart = (uint8_t)mean;
    float dec = (mean - intpart) * 100; // get just 2 decimal
    uint8_t decpart = (uint8_t)dec;

    temp[0] = intpart;
    temp[1] = decpart;
}


float get_hum_moving_average(){
    float mean = 0;

    for(int i = 0; i < hBuffer.counter; i++)
        mean += get_element(&hBuffer);
    
    mean /= hBuffer.counter;
    return mean;
}


void update_humidity_char(void *arg) {
    uint8_t *hum = (uint8_t *)arg;
    float mean = get_hum_moving_average();
    uint8_t intpart = (uint8_t)mean;
    float dec = (mean - intpart) * 100; // get just 2 decimal
    uint8_t decpart = (uint8_t)dec;

    hum[0] = intpart;
    hum[1] = decpart;
}


static void chrono_sample_temp(void *arg) {
    si7021_ev = TEMP_SAMPLE;
    QueueHandle_t *queue = (QueueHandle_t*) arg;
    xQueueSendToFront(*queue, (void *) &si7021_ev, 100);
}


static void chrono_sample_hum(void *arg) {
    si7021_ev = HUM_SAMPLE;
    QueueHandle_t *queue = (QueueHandle_t*) arg;
    xQueueSendToFront(*queue, (void *) &si7021_ev, 100);
}


bool are_temp_samples() {
    return tBuffer.counter > 0;
}


bool are_hum_samples() {
    return hBuffer.counter > 0;
}
#include "esp_timer.h"

#include "ccs811_sensor/ccs811.h"
#include "i2c_controller/i2c_controller.h"
#include "si7021_sensor/si7021.h"
#include "common.h"

void set_environment_vars(float temperature, float humidity) {
    int _temp = 0, _rh = 0;
    if(temperature>0)
        _temp = (int)temperature + 0.5;  // this will round off the floating point to the nearest integer value
    else if(temperature<0) // account for negative temperatures
        _temp = (int)temperature - 0.5;
    _temp = _temp + 25;  // temperature high byte is stored as T+25°C in the sensor's memory so the value of byte is positive
    _rh = (int)humidity + 0.5;  // this will round off the floating point to the nearest integer value
    
    uint8_t envData[5];
    
    envData[0] = CCS811_REG_ENV_DATA; // env register wher to write
    envData[1] = _rh << 1;  // shift the binary number to left by 1. This is stored as a 7-bit value
    envData[2] = 0;  // most significant fractional bit. Using 0 here - gives us accuracy of +/-1%. Current firmware (2016) only supports fractional increments of 0.5
    envData[3] = _temp << 1;
    envData[4] = 0;

    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, envData, 5);
}


int init_ccs811(int thresh, int interrupt, eDRIVE_MODE_t mode) {
    uint8_t code[2];

    // check sensor availability
    uint8_t id;
    code[0] = CCS811_REG_HW_ID;

    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, code, 1);
    i2c_master_read_from(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, &id, 1);
    
    if(id != CCS811_HW_ID) {
        ESP_LOGE(CONFIG_LOG_TAG, "CCS811 is not available.");
        return 1;
    }

    code[0] = CCS811_BOOTLOADER_APP_START;
    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, code, 1);
    
    // set measurement mode
    uint8_t measurement = 0;
    measurement = (thresh << 2) | (interrupt << 3) | (mode << 4);
    code[0] = CCS811_REG_MEAS_MODE;
    code[1] = measurement;
    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, code, 2);

    // set env data
    set_environment_vars(20, 75);

    // check errors
    uint8_t buffer;
    code[0] = CCS811_REG_STATUS;

    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, code, 1);
    i2c_master_read_from(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, &buffer, 1);
    
    if((buffer & 0x1) == 1) {
        ESP_LOGE(CONFIG_LOG_TAG, "There is an error in the i2c or CCS811 sensor.");
        return 1;
    }
    return 0;
}


void ccs811_task(void *arg) {
    ccs811_args_t *params = (ccs811_args_t *) arg;
    ccs811_event_t ev = CCS811_DEFAULT;
    int ret, co2;
    double mean = 0;
    bool start = true;
    esp_timer_handle_t timer_co2, timer_env_vars;
    BaseType_t q_ready;
    bool co2_enb = true;

    // timers configuration
    const esp_timer_create_args_t chrono_args_t = {
        .callback = &chrono_sample_co2,
        .name = "timer_sample_co2",
        .arg = (void *)&params->event_queue,
    };
    esp_timer_create(&chrono_args_t, &timer_co2);

    const esp_timer_create_args_t env_args_t = {
        .callback = &chrono_set_environment,
        .name = "chrono co2 environment compensation",
        .arg = (void *)&params->event_queue,
    };
    esp_timer_create(&env_args_t, &timer_env_vars);
    
    init_buffer(&co2_buffer, CONFIG_CO2_WINDOW_SIZE);

    if(init_ccs811(0, 0, eMode4)) {
        start = false;
        ESP_LOGE(CONFIG_LOG_TAG, "ccs811 task not started");
    }

    if(start) {
        ESP_LOGI(CONFIG_LOG_TAG, "ccs811 task started");
        esp_timer_start_once(timer_env_vars, get_time_micros(CCS811_FISRT_ENV_VARS_T));
        esp_timer_start_periodic(timer_co2, get_time_micros(*params->co2_samp_freq));
    }

    while (start) {
        if(ev == CO2_SAMPLE && co2_enb){
            for(int i = 0; i < CONFIG_CO2_N_SAMPLES; i++) {
                ret = get_co2(I2C_MASTER_NUM, &co2, params->i2c_sem);

                if (ret == ESP_ERR_TIMEOUT)
                    ESP_LOGE(CONFIG_LOG_TAG, "I2C Timeout for co2 sensor");
                else if (ret == ESP_OK)
                    mean += co2;
                else
                    ESP_LOGW(CONFIG_LOG_TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
            }

            mean /= CONFIG_TEMP_N_SAMPLES;
            add_element(&co2_buffer, mean);
            ESP_LOGI(CONFIG_LOG_TAG, "co2 = %f", mean);
            mean = 0;
        }
        else if(ev == CO2_ENABLE) {
            co2_enb = !co2_enb;
            if(co2_enb)
                ESP_LOGI(CONFIG_LOG_TAG, "CO2 enabled");
            else
                ESP_LOGI(CONFIG_LOG_TAG, "CO2 disabled");
        }
        else if(ev == CO2_SAMPLE_FREQ) {
            esp_timer_stop(timer_co2);
            ESP_LOGI(CONFIG_LOG_TAG, "Changed co2 sample to each %d s", *params->co2_samp_freq);
            esp_timer_start_periodic(timer_co2, get_time_micros(*params->co2_samp_freq));
        }
        else if(ev == CO2_UPDATE_ENV_VARS) {
            if(!are_temp_samples() || !are_hum_samples()) {
                esp_timer_start_once(timer_env_vars, get_time_micros(CCS811_FISRT_ENV_VARS_T));
            }
            else {
                float temp = get_temp_moving_average();
                float hum = get_hum_moving_average();
                set_environment_vars(temp, hum);
                ESP_LOGI(CONFIG_LOG_TAG, "Updated environment variables in sensor ccs811");
                esp_timer_start_once(timer_env_vars, get_time_micros(CONFIG_CCS811_ENV_VARS_T));
            }
        }
        
        do {q_ready = xQueueReceive(params->event_queue, (void *) &ev, 2000);} while(!q_ready);
    }

    free_buffer(&co2_buffer);
    vTaskDelete(NULL);
}


esp_err_t get_co2(int i2c_num, int *co2, SemaphoreHandle_t i2c_sem){
    int ret;
    uint8_t buffer[8];
    uint8_t code[1] = {CCS811_REG_ALG_RESULT_DATA};
    BaseType_t q_ready;

    // get i2c
    do { q_ready = xSemaphoreTake(i2c_sem, 1000); } while(!q_ready);

    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, code, 1);
    ret = i2c_master_read_from(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, buffer, 8);

    // release i2c
    xSemaphoreGive(i2c_sem);

    *co2 = (((int)buffer[0] << 8) | (int)buffer[1]);
    return ret;
}


float get_co2_moving_average() {
    float mean = 0;

    for(int i = 0; i < co2_buffer.counter; i++)
        mean += get_element(&co2_buffer);
    
    mean /= co2_buffer.counter;
    return mean;
}


void update_co2_char(void *arg) {
    uint8_t *co2 = (uint8_t *)arg;

    int mean_i = (int) get_co2_moving_average();

    co2[0] = (uint8_t)((mean_i >> 8) & 0xFF);
    co2[1] = (uint8_t)(mean_i & 0xFF);
}


static void chrono_sample_co2(void *arg) {
    ccs811_ev = CO2_SAMPLE;
    QueueHandle_t *queue = (QueueHandle_t*) arg;
    xQueueSendToFront(*queue, (void *) &ccs811_ev, 100);
}


static void chrono_set_environment(void *arg) {
    ccs811_ev = CO2_UPDATE_ENV_VARS;
    QueueHandle_t *queue = (QueueHandle_t*) arg;
    xQueueSendToFront(*queue, (void *) &ccs811_ev, 100);
}


bool are_co2_samples() {
    return co2_buffer.counter > 0;
}
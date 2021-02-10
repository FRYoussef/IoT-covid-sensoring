#include "ccs811_sensor/ccs811.h"
#include "i2c_controller/i2c_controller.h"


void soft_reset() {
    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, CCS811_REG_SW_RESET);

    for(int i = 0; i < CCS811_REG_SW_RESET_VALUES_N; i++)
        i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, CCS811_REG_SW_RESET_VALUES[i]);
}


void set_environment_vars(float temperature, float humidity) {
    int _temp = 0, _rh = 0;
    if(temperature>0)
        _temp = (int)temperature + 0.5;  // this will round off the floating point to the nearest integer value
    else if(temperature<0) // account for negative temperatures
        _temp = (int)temperature - 0.5;
    _temp = _temp + 25;  // temperature high byte is stored as T+25°C in the sensor's memory so the value of byte is positive
    _rh = (int)humidity + 0.5;  // this will round off the floating point to the nearest integer value
    
    uint8_t envData[4];
    
    envData[0] = _rh << 1;  // shift the binary number to left by 1. This is stored as a 7-bit value
    envData[1] = 0;  // most significant fractional bit. Using 0 here - gives us accuracy of +/-1%. Current firmware (2016) only supports fractional increments of 0.5
    envData[2] = _temp << 1;
    envData[3] = 0;
    
    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, CCS811_REG_ENV_DATA);

    for(int i = 0; i < 4; i++)
        i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, envData[i]);
}


void init_ccs811(int thresh, int interrupt, eDRIVE_MODE_t mode) {
    soft_reset();
    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, CCS811_BOOTLOADER_APP_START);
    
    // set measurement mode
    uint8_t measurement = 0;
    measurement = (thresh << 2) | (interrupt << 3) | (mode << 4);
    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, CCS811_REG_MEAS_MODE);
    i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, measurement);

    // check errors
    uint8_t buffer[1];
    i2c_master_read_from(CCS811_SENSOR_ADDR, CCS811_DELAY,
        I2C_MASTER_NUM, buffer, 1, CCS811_REG_STATUS);
    
    if((buffer[0] & 0x1) == 1)
        ESP_LOGE(CONFIG_LOG_TAG, "There is an error in the i2c or sensor.");


    // i2c_master_write_on(CCS811_SENSOR_ADDR, I2C_MASTER_NUM, CCS811_REG_ALG_RESULT_DATA);
    // vTaskDelay(CCS811_DELAY / portTICK_RATE_MS);
    // i2c_master_read_from(CCS811_SENSOR_ADDR, CCS811_DELAY,
    //     I2C_MASTER_NUM, buffer, 1, CCS811_REG_STATUS);
    // ESP_LOGI(CONFIG_LOG_TAG, "STATUS = %d", buffer[0]);
}


void ccs811_task(void *arg) {
    int ret, co2;
    double mean = 0;

    init_buffer(&co2_buffer, CONFIG_CO2_WINDOW_SIZE);
    init_ccs811(0, 0, eMode4);

    ESP_LOGI(CONFIG_LOG_TAG, "ccs811 task started");

    while (1) {
        
        for(int i = 0; i < CONFIG_CO2_N_SAMPLES; i++) {
            ret = get_co2(I2C_MASTER_NUM, &co2);

            if (ret == ESP_ERR_TIMEOUT)
                ESP_LOGE(CONFIG_LOG_TAG, "I2C Timeout for co2 sensor");
            else if (ret == ESP_OK)
            mean += co2;
            else
                ESP_LOGW(CONFIG_LOG_TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

        mean /= CONFIG_TEMP_N_SAMPLES;
        add_element(&co2_buffer, mean);
        ESP_LOGI(CONFIG_LOG_TAG, "co2 = %f ppm", mean);
        mean = 0;
        vTaskDelay(1000);
    }

    free_buffer(&co2_buffer);
    vTaskDelete(NULL);
}


esp_err_t get_co2(int i2c_num, int *co2){
    int ret;
    uint8_t buffer[2]; // we just need first 2 bytes

    ret = i2c_master_read_from(CCS811_SENSOR_ADDR, CCS811_DELAY,
        i2c_num, buffer, 2, CCS811_REG_ALG_RESULT_DATA);

    ESP_LOGE(CONFIG_LOG_TAG, "%d %d", buffer[0], buffer[1]);
    *co2 = (((int)buffer[0] << 8) | (int)buffer[1]);
    return ret;
}


void update_co2_char(void *arg) {
    uint8_t *co2 = (uint8_t *)arg;
    float mean = 0;

    for(int i = 0; i < co2_buffer.counter; i++)
        mean += get_element(&co2_buffer);
    
    mean /= co2_buffer.counter;
    int mean_i = (int) mean;

    co2[0] = (uint8_t)((mean_i >> 8) & 0xFF);
    co2[1] = (uint8_t)(mean_i & 0xFF);
}
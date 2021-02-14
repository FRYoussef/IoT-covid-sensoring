/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 2nd 2021
 **/

#include "i2c_controller.h"

/**
 *
 * 1. Write READ_TEMP command
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack |  write READ_TEMP+ ack | stop |
 * --------|---------------------------|-----------------------|------|
 * 2. wait more than 100 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
esp_err_t i2c_master_write_read_from(uint8_t sensor_addr, int sensor_delay, 
    i2c_port_t i2c_num, uint8_t *buffer, int buffer_n, uint8_t *code, int n_code)
{
    esp_err_t ret = i2c_master_write_on(sensor_addr, i2c_num, code, n_code);

    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(sensor_delay / portTICK_RATE_MS);

    return i2c_master_read_from(sensor_addr, i2c_num, buffer, buffer_n);
}

    // // read
    // cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, CCS811_SENSOR_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    // i2c_master_read_byte(cmd, &buffer, NACK_VAL);
    // i2c_master_stop(cmd);
    // i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
esp_err_t i2c_master_read_from(uint8_t sensor_addr, i2c_port_t i2c_num, uint8_t *buffer, int buffer_n)
{
    int i = 0;
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd;
    
    for(; i < buffer_n; i++){
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, sensor_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &buffer[i], NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_TIMEOUT_MS / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    }

    return ret;
}

    // // write
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, CCS811_SENSOR_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, CCS811_REG_HW_ID, ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    // i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);

esp_err_t i2c_master_write_on(uint8_t sensor_addr, i2c_port_t i2c_num, uint8_t *code, int n_code)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, code, n_code, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

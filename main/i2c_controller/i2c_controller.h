/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: February 7th 2021
 **/

#ifndef _I2C_
#define _I2C_

#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define I2C_TIMEOUT_MS 1000

esp_err_t i2c_master_read_from(int sensor_addr, int sensor_delay, 
    i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l, uint8_t code);

esp_err_t i2c_master_write_on(int sensor_addr, i2c_port_t i2c_num, uint8_t code);

esp_err_t i2c_master_init(void);

#endif
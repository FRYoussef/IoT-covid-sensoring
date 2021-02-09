#include <time.h>
#include "ccs811_sensor/ccs811.h"
#include "i2c_controller/i2c_controller.h"

void init_ccs811() {
    int ret;
    uint8_t sensor_data_h, sensor_data_l;

    ret = i2c_master_read_from(CCS811_SENSOR_ADDR, CCS811_DELAY,
        I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l, );
}
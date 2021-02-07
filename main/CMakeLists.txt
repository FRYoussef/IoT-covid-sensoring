set(SOURCES main.c)
list(APPEND SOURCES "gatt_table/gatt_table.c")
list(APPEND SOURCES "si7021_sensor/si7021.c")
list(APPEND SOURCES "i2c_controller/i2c_controller.c")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS .)
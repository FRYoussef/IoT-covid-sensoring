set(SOURCES main.c)
list(APPEND SOURCES "gatt_table/gatt_table.c")
list(APPEND SOURCES "temperatre_sensor/temperature.c")
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS .)
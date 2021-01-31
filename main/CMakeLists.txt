set(SOURCES main.c)
list(APPEND SOURCES "gatt_table/gatt_table.c")
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS .)
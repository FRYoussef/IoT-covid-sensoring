# IoT-covid-sensoring
IoT project which sensors the temperature, air CO2, and room capacity using Bluetooth.

## Software Requirements
    - Tested under ESP-IDF 4.2
    - Using menuconfig you have to activate the following options:
        - Component config → Power Management → Support for Power Management → Enable DFS at startup
        - Component config → FreeRTOS → Tickless Idle Support
        - Component config → Bluetooth → Bluetooth

## Hardware Requirements
    - 1 x [Esp32](https://www.espressif.com/en/products/socs/esp32/overview)
    - 1 x [SI7021 sensor](https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf)
    - 1 x [CCS811 sensor](https://dfimg.dfrobot.com/nobody/wiki/7334c560756596ba0cf3f1d2102d19dd.pdf)
    - Jumper wires
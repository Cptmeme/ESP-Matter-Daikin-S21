# ESP-Matter-Daikin-S21

ESP32 project to get matter to work with my Daikin A/C with a S21 port.

## Firmware

- Heating/cooling/auto mode supported.
- Powerful mode now supported!
- Has been tested to work with the custom PCB provided in this repo connected to a Daikin FTXM25R5V1B

### TODO
- Add external temperatures

### Sources

Code copied or inspiration taken from:

- https://github.com/espressif/esp-matter (copied example code for A/C)
- https://github.com/revk/ESP32-Faikout (taken s21 protocol files and based my custom board off of the faikout boards)
- https://github.com/asund/esphome-daikin-s21 (inspiration)

## Custom PCB
Based on an esp32c6-mini-1 to allow for thread support, has been verified to work in esphome using asund's esphome-daikin-s21 code.

### WARNING!
Do not connect the pcb to the A/C and a PC over the usb-c port at the same time as this will short out your usb port!



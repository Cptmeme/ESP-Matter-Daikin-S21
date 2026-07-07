#pragma once

// This changes the "Manufacturer" field in the device settings
#define CHIP_DEVICE_CONFIG_DEVICE_VENDOR_NAME            "Daikin"

// This changes the "Model" field and the default name in the app
#define CHIP_DEVICE_CONFIG_DEVICE_PRODUCT_NAME           "Daikin A/C"

// This is the "hidden" label that some apps use as the primary display name
#define CHIP_DEVICE_CONFIG_DEFAULT_NODE_LABEL            "Daikin Thermostat"

// "Hardware version" field (Basic Information cluster). Default is "TEST_VERSION".
#define CHIP_DEVICE_CONFIG_DEFAULT_DEVICE_HARDWARE_VERSION_STRING "2.0"

// "Firmware/Software version" string shown by controllers. The numeric
// SoftwareVersion (used for OTA comparison) is CONFIG_DEVICE_SOFTWARE_VERSION_NUMBER.
#define CHIP_DEVICE_CONFIG_DEVICE_SOFTWARE_VERSION_STRING "1.0"
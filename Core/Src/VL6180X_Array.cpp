/*
 * VL6180X_Array.cpp
 *
 * Created on: Mar 3, 2024
 * Author: Lferrari
 * Refactored by: Gemini
 */
#include "VL6180X_Array.h"

// Helper function to select the I2C multiplexer channel
void select_mux_channel(I2C_HandleTypeDef* i2c, uint8_t channel) {
    if (channel > 7) return;
    uint8_t data = 1 << channel;
    // The address is 7-bit, HAL functions expect it to be shifted left by 1.
    HAL_I2C_Master_Transmit(i2c, TCA9548A_ADDRESS << 1, &data, 1, HAL_MAX_DELAY);
}


VL6180X_Array::VL6180X_Array(I2C_HandleTypeDef* i2c, int n_sensors) {
    i2c_bus = i2c; // Store the I2C bus handle
    sensor_numbers = n_sensors > MAX_SENSORS ? MAX_SENSORS : n_sensors;

    for (int i = 0; i < sensor_numbers; i++) {
        Dev[i] = VL6180X();
        Dev[i].setBus(i2c_bus);
        
        // Set i2c address via mux
        select_mux_channel(i2c_bus, i);

        // Address is default, init and configure
        // Note: All sensors share the same default address 0x29, the MUX keeps them separate.
        Dev[i].init();
        Dev[i].configureDefault();
    }
}

void VL6180X_Array::ping() {
    // Read all sensors
    for (int i = 0; i < sensor_numbers; i++) {
        select_mux_channel(i2c_bus, i);
        RangingData[i] = Dev[i].readRangeSingle();
    }
}

void VL6180X_Array::ping_single(int sensor_index) {
    if (sensor_index >= 0 && sensor_index < sensor_numbers) {
        select_mux_channel(i2c_bus, sensor_index);        
        RangingData[sensor_index] = Dev[sensor_index].readRangeSingle();
    }
}

uint16_t VL6180X_Array::get_distance(int sensor_index) {
    if (sensor_index >= 0 && sensor_index < sensor_numbers) {
        return (uint16_t)RangingData[sensor_index];
    }
    // Return a large value indicating an error or out-of-range reading
    return 8191;
}

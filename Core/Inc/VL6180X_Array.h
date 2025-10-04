/*
 * VL6180X_Array.h
 *
 * Created on: Mar 3, 2024
 * Author: Lferrari
 * Refactored by: Gemini
 */
#ifndef VL6180X_ARRAY_H
#define VL6180X_ARRAY_H

#include <VL6180X.h>

#define MAX_SENSORS 5
#define TCA9548A_ADDRESS 0x70 // Default address, can be changed by setting address pins

#include "main.h"
#include "i2c.h"


class VL6180X_Array {
private:
    int sensor_numbers;
    VL6180X Dev[MAX_SENSORS];
    I2C_HandleTypeDef* i2c_bus; // Store the bus handle

public:
    uint8_t RangingData[MAX_SENSORS];

    VL6180X_Array(I2C_HandleTypeDef* i2c, int n_sensors);
    void ping();
    void ping_single(int sensor_index);
    uint16_t get_distance(int sensor_index);
};


#endif /* VL6180X_ARRAY_H */
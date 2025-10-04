
/*
 * vl53l0x.h
 *
 *  Created on: Mar 3, 2024
 *      Author: Lferrari
 */
#ifndef VL53L0X_ARRAY_H
#define VL53L0X_ARRAY_H

// VL53l0x params:
#define VCSEL_PERIOD_PRE_RANGE	18
#define VCSEL_PERIOD_FINAL_RANGE 14
// The timing budget is crucial for real-time performance.
// The default 33000us (33ms) is too long for a fast control loop.
// A value of 4000us (4ms) allows for faster readings, suitable for wall following,
// but is still a blocking operation.
#define TimingBudgetMicroSeconds 4000
#define MAX_SENSORS 5

#include "vl53l0x_api.h"
#include "main.h"
#include "i2c.h"


class VL53L0X_array {
private:
    int sensor_numbers;
    uint16_t Xs_pins[MAX_SENSORS];
    GPIO_TypeDef* Xs_port[MAX_SENSORS];
    // ToDo - Add offset & slope
    VL53L0X_DEV Dev[MAX_SENSORS];

public:
    VL53L0X_RangingMeasurementData_t RangingData[MAX_SENSORS];

    VL53L0X_array(I2C_HandleTypeDef*, GPIO_TypeDef**, uint16_t *, int);
    // float ping();}
    void ping();
    void ping_single(int sensor_index);
    uint16_t get_distance(int sensor_index);
};


#endif

/*
 * vl53l0x.cpp
 *
 *  Created on: Mar 3, 2024
 *      Author: Lferrari
 */

#include "vl53l0x.h"

VL53L0X_array::VL53L0X_array(I2C_HandleTypeDef* i2c, GPIO_TypeDef** port, uint16_t * pins, int n_sensors) {

    sensor_numbers=n_sensors;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    for (int i = 0; i< n_sensors; i++) {
        Dev[i] = new VL53L0X_Dev_t; // Allocate memory for each Dev
        Xs_pins[i] = pins[i];
        Xs_port[i] = port[i];
        Dev[i]->I2cHandle = i2c;
        Dev[i]->I2cDevAddr = 0x52;
        HAL_GPIO_WritePin(Xs_port[i], Xs_pins[i], GPIO_PIN_RESET); // Disable XSHUT
    }

    uint8_t address = 0x30; // Starting address
    // Set device address
    for (int i = 0; i< n_sensors; i++) {
        HAL_GPIO_WritePin(Xs_port[i], Xs_pins[i], GPIO_PIN_SET); // Enable XSHUT
        HAL_Delay(20);
        VL53L0X_SetDeviceAddress(Dev[i],address);
        Dev[i]->I2cDevAddr = address; // update new address
        address=address+2;
    }
     for (int i = 0; i< n_sensors; i++) {
        //
        // VL53L0X init for Single Measurement
        //
        VL53L0X_WaitDeviceBooted( Dev[i] );
        VL53L0X_DataInit( Dev[i] );
        VL53L0X_StaticInit( Dev[i] );
        VL53L0X_PerformRefCalibration(Dev[i], &VhvSettings, &PhaseCal);
        VL53L0X_PerformRefSpadManagement(Dev[i], &refSpadCount, &isApertureSpads);
        VL53L0X_SetDeviceMode(Dev[i], VL53L0X_DEVICEMODE_SINGLE_RANGING);

        // Enable/Disable Sigma and Signal check
        VL53L0X_SetLimitCheckEnable(Dev[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        VL53L0X_SetLimitCheckEnable(Dev[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        VL53L0X_SetLimitCheckValue(Dev[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
        VL53L0X_SetLimitCheckValue(Dev[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
        VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev[i], TimingBudgetMicroSeconds);
        VL53L0X_SetVcselPulsePeriod(Dev[i], VL53L0X_VCSEL_PERIOD_PRE_RANGE, VCSEL_PERIOD_PRE_RANGE);
        VL53L0X_SetVcselPulsePeriod(Dev[i], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VCSEL_PERIOD_FINAL_RANGE);
    }

}

// float* VL53L0X_array::ping() {
void VL53L0X_array::ping() {
    // Read all sensors
    for (int i = 0; i< sensor_numbers; i++) {
       VL53L0X_PerformSingleRangingMeasurement(Dev[i], RangingData+i);
    }
}

void VL53L0X_array::ping_single(int sensor_index) {
    if (sensor_index >= 0 && sensor_index < sensor_numbers) {
       // This is a blocking call
       VL53L0X_PerformSingleRangingMeasurement(Dev[sensor_index], RangingData+sensor_index);
    }
}

uint16_t VL53L0X_array::get_distance(int sensor_index) {
    if (sensor_index >= 0 && sensor_index < sensor_numbers && RangingData[sensor_index].RangeStatus == 0) {
        return RangingData[sensor_index].RangeMilliMeter;
    }
    // Return a large value indicating an error or out-of-range reading
    return 8191;
}

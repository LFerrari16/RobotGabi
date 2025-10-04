/*
 * vl53l0x.h
 *
 *  Created on: Mar 3, 2024
 *      Author: Lferrari
 *  This is a library for the L3GD20 GYROSCOPE
 */

#include <L3GD20.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
L3GD20::L3GD20(I2C_HandleTypeDef* i2c,l3gd20Range_t range, uint8_t addr)
{
  Dev = new L3GD20_Dev_t; // Allocate memory for each Dev
  Dev->I2cHandle = i2c;
  Dev->I2cDevAddr = addr;

  rng = range;
  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id;
  read8(Dev,L3GD20_REGISTER_WHO_AM_I,&id);
  //Serial.println(id, HEX);
  if ((id != L3GD20_ID) && (id != L3GD20H_ID))
  {
  }

  /* Set CTRL_REG1 (0x20)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   7-6  DR1/0     Output data rate                                   00
   5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

  /* Switch to normal mode and enable all three channels */
  write8(Dev,L3GD20_REGISTER_CTRL_REG1, 0x0F);
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG2 (0x21)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   5-4  HPM1/0    High-pass filter mode selection                    00
   3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG3 (0x22)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG4 (0x23)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
   5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

  /* Adjust resolution if requested */
  switch(rng)
  {
    case L3DS20_RANGE_250DPS:
      write8(Dev,L3GD20_REGISTER_CTRL_REG4, 0x00);
      break;
    case L3DS20_RANGE_500DPS:
      write8(Dev,L3GD20_REGISTER_CTRL_REG4, 0x10);
      break;
    case L3DS20_RANGE_2000DPS:
      write8(Dev,L3GD20_REGISTER_CTRL_REG4, 0x20);
      break;
  }
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG5 (0x24)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
   3-2  INT1_SEL  INT1 Selection config                              00
   1-0  OUT_SEL   Out selection config                               00 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

}
/***************************************************************************
 CALIBRATE
 ***************************************************************************/
void L3GD20::calibrate(uint16_t samples)
{
  float sum_z = 0.0f;
  l3gd20Data temp_data;

  for (uint16_t i = 0; i < samples; i++) {
    temp_data = read(); // This already applies sensitivity
    sum_z += temp_data.z;
    HAL_Delay(5); // Wait a bit between samples
  }
  z_offset = sum_z / samples;
}

/***************************************************************************
/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
l3gd20Data L3GD20::read()
{ 
  uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
  l3gd20Data data;
  
  // read8(Dev, L3GD20_REGISTER_OUT_X_L,&xlo);;
  // read8(Dev, L3GD20_REGISTER_OUT_X_H,&xhi);;
  // read8(Dev, L3GD20_REGISTER_OUT_Y_L,&ylo);;
  // read8(Dev, L3GD20_REGISTER_OUT_Y_H,&yhi);;
  read8(Dev, L3GD20_REGISTER_OUT_Z_L,&zlo);;
  read8(Dev, L3GD20_REGISTER_OUT_Z_H,&zhi);;

  // Shift values to create properly formed integer (low byte first)
  // data.x = (int16_t)(xlo | (xhi << 8));
  // data.y = (int16_t)(ylo | (yhi << 8));
  data.z = (int16_t)(zlo | (zhi << 8));
  

  // Apply sensitivity before offset calculation during calibration,
  // and apply offset for normal readings.

  // Compensate values depending on the resolution
  switch(rng)
  {
    case L3DS20_RANGE_250DPS:
      // data.x *= L3GD20_SENSITIVITY_250DPS;
      // data.y *= L3GD20_SENSITIVITY_250DPS;
      data.z *= L3GD20_SENSITIVITY_250DPS;
      break;
    case L3DS20_RANGE_500DPS:
      // data.x *= L3GD20_SENSITIVITY_500DPS;
      // data.y *= L3GD20_SENSITIVITY_500DPS;
      data.z *= L3GD20_SENSITIVITY_500DPS;
      break;
    case L3DS20_RANGE_2000DPS:
      // data.x *= L3GD20_SENSITIVITY_2000DPS;
      // data.y *= L3GD20_SENSITIVITY_2000DPS;
      data.z *= L3GD20_SENSITIVITY_2000DPS;
      break;
  }

  // Apply the calculated offset
  data.z -= z_offset;

  return data; 
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/


int _I2CWrite(L3GD20_DEV Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Transmit(Dev->I2cHandle, Dev->I2cDevAddr, pdata, count, i2c_time_out);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

int _I2CRead(L3GD20_DEV Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Receive(Dev->I2cHandle, Dev->I2cDevAddr|1, pdata, count, i2c_time_out);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}


int L3GD20::write8(L3GD20_DEV Dev, uint8_t index, uint8_t data) {
    int status_int;

    _I2CBuffer[0] = index;
    _I2CBuffer[1] = data;

    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
 
    return status_int;
}


int L3GD20::read8(L3GD20_DEV Dev, uint8_t index, uint8_t *data) {
    int status_int;

    status_int = write8(Dev, index, 1);
    status_int = _I2CRead(Dev, data, 1);

    return status_int;
}


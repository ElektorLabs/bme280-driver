/*
  bme280.cpp - driver for Bosch Sensortec BME280 combined humidity and pressure sensor.

  Copyright (c) 2015 Elektor

  26/11/2015 - CPV, Initial release.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

*/

#include "bme280.h"


BME280::BME280(void)
{
  _i2c_address = 0;
  _t_fine = 0;
  _temperature = 0;
  _pressure = 0;
  _humidity = 0;
  clearCalibrationData();
}


void BME280::clearCalibrationData(void)
{
  _dig_T1 = 0;
  _dig_T2 = 0;
  _dig_T3 = 0;
  _dig_P1 = 0;
  _dig_P2 = 0;
  _dig_P3 = 0;
  _dig_P4 = 0;
  _dig_P5 = 0;
  _dig_P6 = 0;
  _dig_P7 = 0;
  _dig_P8 = 0;
  _dig_P9 = 0;
  _dig_H1 = 0;
  _dig_H2 = 0;
  _dig_H3 = 0;
  _dig_H4 = 0;
  _dig_H5 = 0;
  _dig_H6 = 0;
}


uint8_t BME280::begin(uint8_t i2cAddress)
{
  _i2c_address = i2cAddress;
  if (readId()==BME280_ID)
  {
    clearCalibrationData();
    readCalibrationData();
    return 0;
  }
  return (uint8_t)-1;
}


void BME280::busWrite(uint8_t *p_data, uint8_t data_size, uint8_t repeated_start)
{
  if (_i2c_address==BME280_I2C_ADDRESS1 || _i2c_address==BME280_I2C_ADDRESS2)
  {
    // Assume I2C bus.
    i2cWrite(_i2c_address,p_data,data_size,repeated_start);
  }
  else
  {
    // Assume SPI bus.
    // First byte is supposed to be the address of the register to write to, set R/~W bit to 0.
    p_data[0] &= 0x7f;
    spiWrite(p_data,data_size);
  }
}


void BME280::busRead(uint8_t *p_data, uint8_t data_size)
{
  if (_i2c_address==BME280_I2C_ADDRESS1 || _i2c_address==BME280_I2C_ADDRESS2)
  {
    // Assume I2C bus.
    i2cRead(_i2c_address,p_data,data_size);
  }
  else
  {
    // Assume SPI bus.
    // First byte is supposed to be the address of the register to write to, set R/~W bit to 1.
    p_data[0] |= 0x80;
    spiRead(p_data,data_size);
  }
}


uint8_t BME280::readUint8(uint8_t reg)
{
  uint8_t data;
  busWrite(&reg,1,1); // Use repeated start.
  busRead(&data,1); // Read one byte.
  return data;
}


uint16_t BME280::readUint16(uint8_t reg)
{
  uint8_t data[2];
  uint16_t value;
  busWrite(&reg,1,1); // Use repeated start.
  busRead(data,2); // Read two bytes.
  // Process as little endian, which is the case for calibration data.
  value = data[1];
  value = (value<<8) | data[0];
  return value;
}


void BME280::readCalibrationData(void)
{
  _dig_T1 = readUint16(BME280_CAL_T1);
  _dig_T2 = (int16_t) readUint16(BME280_CAL_T2);
  _dig_T3 = (int16_t) readUint16(BME280_CAL_T3);
  _dig_P1 = readUint16(BME280_CAL_P1);
  _dig_P2 = (int16_t) readUint16(BME280_CAL_P2);
  _dig_P3 = (int16_t) readUint16(BME280_CAL_P3);
  _dig_P4 = (int16_t) readUint16(BME280_CAL_P4);
  _dig_P5 = (int16_t) readUint16(BME280_CAL_P5);
  _dig_P6 = (int16_t) readUint16(BME280_CAL_P6);
  _dig_P7 = (int16_t) readUint16(BME280_CAL_P7);
  _dig_P8 = (int16_t) readUint16(BME280_CAL_P8);
  _dig_P9 = (int16_t) readUint16(BME280_CAL_P9);
  _dig_H1 = readUint8(BME280_CAL_H1);
  _dig_H2 = (int16_t) readUint16(BME280_CAL_H2);
  _dig_H3 = readUint8(BME280_CAL_H3);
  // H4 & H5 share a byte.
  uint8_t temp1 = readUint8(BME280_CAL_H4);
  uint8_t temp2 = readUint8(BME280_CAL_H45);
  uint8_t temp3 = readUint8(BME280_CAL_H5);
  _dig_H4 = (temp1<<4) | (temp2&0x0f);
  _dig_H5 = (temp3<<4) | (temp2>>4);
  _dig_H6 = (int8_t) readUint8(BME280_CAL_H6);
}


uint8_t BME280::readFrom(uint8_t reg, uint8_t data_size, uint8_t *p_data)
{
  // Set start address to read from.
  busWrite(&reg,1,1); // Use repeated start.
  // Now read the requested number of bytes.
  busRead(p_data,data_size);
  return data_size;
}


void BME280::read(void)
{
  // Get all the measurements in one burst (recommended).
  uint8_t data[BME280_MEASUREMENT_SIZE];
  readFrom(BME280_MEASUREMENT_REGISTER,BME280_MEASUREMENT_SIZE,data);
  // We assume Normal mode, so it is not necessary to reissue a Forced mode command here.

  // Process data.
  int32_t p = assembleRawValue(&data[0],1);
  int32_t t = assembleRawValue(&data[3],1);
  int32_t h = assembleRawValue(&data[6],0);

  _temperature = compensateTemperature(t); // First call this before calling the other compensate functions.
  _pressure = compensatePressure(p); // Uses value calculated by compensateTemperature.
  _humidity = compensateHumidity(h); // Uses value calculated by compensateTemperature.
}


int32_t BME280::assembleRawValue(uint8_t *p_data, uint8_t has_xlsb)
{
  // Needed to decode sensor data.
  uint32_t value = p_data[0];
  value <<= 8;
  value |= p_data[1];
  if (has_xlsb!=0)
  {
    value <<= 4;
    value |= (p_data[2]>>4);
  }
  return (int32_t) value;
}


void BME280::writeControlRegisters(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode)
{
  uint8_t data[2];
  data[0] = BME280_CTRL_HUM_REGISTER;
  data[1] = (osrs_h&0x07);
  busWrite(data,2,0);
  // Writing CTRL_MEAS validates previous write to CTRL_HUM.
  data[0] = BME280_CTRL_MEAS_REGISTER;
  data[1] = ((osrs_t&0x07)<<5) | ((osrs_p&0x07)<<2) | (mode&0x03);
  busWrite(data,2,0);
}


void BME280::writeConfigRegister(uint8_t t_sb, uint8_t filter, uint8_t spi)
{
  uint8_t data[2];
  data[0] = BME280_CONFIG_REGISTER;
  data[1] = ((t_sb&0x07)<<5) | ((filter&0x07)<<2) | (spi&0x01);
  busWrite(data,2,0);
}


void BME280::reset(void)
{
  uint8_t data[2] = { BME280_RESET_REGISTER, BME280_RESET };
  busWrite(data,2,0);
}


uint8_t BME280::readId(void)
{
  return readUint8(BME280_ID_REGISTER);
}


#if BME280_ALLOW_FLOAT!=0

// From the driver by Bosch Sensortec

//!
// @brief Reads actual temperature from uncompensated temperature
// @note returns the value in Degree centigrade
// @note Output value of "51.23" equals 51.23 DegC.
//
//  @param adc_T : value of uncompensated temperature
//
//  @return  Return the actual temperature in floating point
//
temperature_t BME280::compensateTemperature(int32_t adc_T)
{
  double v_x1_u32;
  double v_x2_u32;
  double temperature;
  
  v_x1_u32  = (((double)adc_T) / 16384.0 - ((double)_dig_T1) / 1024.0) * ((double)_dig_T2);
  v_x2_u32  = ((((double)adc_T) / 131072.0 - ((double)_dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double)_dig_T1) / 8192.0)) * ((double)_dig_T3);
  _t_fine = (int32_t)(v_x1_u32 + v_x2_u32);
  temperature  = (v_x1_u32 + v_x2_u32) / 5120.0;
  return temperature;
}


//!
// @brief Reads actual pressure from uncompensated pressure
// @note Returns pressure in Pa as double.
// @note Output value of "96386.2"
// equals 96386.2 Pa = 963.862 hPa.
//
//  @param adc_P : value of uncompensated pressure
//
//  @return  Return the actual pressure in floating point
//
pressure_t BME280::compensatePressure(int32_t adc_P)
{
  double v_x1_u32;
  double v_x2_u32;
  double pressure;
  
  v_x1_u32 = ((double)_t_fine / 2.0) - 64000.0;
  v_x2_u32 = v_x1_u32 * v_x1_u32 * ((double)_dig_P6) / 32768.0;
  v_x2_u32 = v_x2_u32 + v_x1_u32 * ((double)_dig_P5) * 2.0;
  v_x2_u32 = (v_x2_u32 / 4.0) + (((double)_dig_P4) * 65536.0);
  v_x1_u32 = (((double)_dig_P3) * v_x1_u32 * v_x1_u32 / 524288.0 + ((double)_dig_P2) * v_x1_u32) / 524288.0;
  v_x1_u32 = (1.0 + v_x1_u32 / 32768.0) * ((double)_dig_P1);
  pressure = 1048576.0 - (double)adc_P;
  // Avoid exception caused by division by zero.
  if (v_x1_u32 != 0) pressure = (pressure - (v_x2_u32 / 4096.0)) * 6250.0 / v_x1_u32;
  else return 0;
  v_x1_u32 = ((double)_dig_P9) * pressure * pressure / 2147483648.0;
  v_x2_u32 = pressure * ((double)_dig_P8) / 32768.0;
  pressure = pressure + (v_x1_u32 + v_x2_u32 + ((double)_dig_P7)) / 16.0;
  
  return pressure;
}


//!
// @brief Reads actual humidity from uncompensated humidity
// @note returns the value in relative humidity (%rH)
// @note Output value of "42.12" equals 42.12 %rH
//
//  @param adc_H : value of uncompensated humidity
//
//  @return Return the actual humidity in floating point
//
humidity_t BME280::compensateHumidity(int32_t adc_H)
{
  double var_h;
  
  var_h = (((double)_t_fine) - 76800.0);
  if (var_h != 0)
  {
    var_h = (adc_H - (((double)_dig_H4) * 64.0 + ((double)_dig_H5) / 16384.0 * var_h)) * 
      (((double)_dig_H2) / 65536.0 * (1.0 + ((double) _dig_H6) / 67108864.0 * 
      var_h * (1.0 + ((double)_dig_H3) / 67108864.0 * var_h)));
  }
  else return 0;
  var_h = var_h * (1.0 - ((double)_dig_H1)*var_h / 524288.0);
  if (var_h > 100.0) var_h = 100.0;
  else if (var_h < 0.0) var_h = 0.0;
  return var_h;
}

#else /* BME280_ALLOW_FLOAT */

// From the datasheet.
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of 5123 equals 51.23 DegC.
// _t_fine carries fine temperature as "global" value.
temperature_t BME280::compensateTemperature(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T>>3) - ((int32_t)_dig_T1<<1))) * ((int32_t)_dig_T2)) >> 11;
  var2 = (((((adc_T>>4) - ((int32_t)_dig_T1)) * ((adc_T>>4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;
  _t_fine = var1 + var2;
  T = (_t_fine * 5 + 128) >> 8;
  return T;
}


// From the datasheet.
// Returns pressure in Pa as unsigned 32 bit integer. Output value of 96386 equals 96386 Pa = 963.86 hPa
pressure_t BME280::compensatePressure(int32_t adc_P)
{
  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)_t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)_dig_P6);
  var2 = var2 + ((var1*((int32_t)_dig_P5))<<1);
  var2 = (var2>>2)+(((int32_t)_dig_P4)<<16);
  var1 = (((_dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)_dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((int32_t)_dig_P1))>>15);
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((uint32_t)var1);
  }
  else
  {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)_dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32_t)(p>>2)) * ((int32_t)_dig_P8))>>13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + _dig_P7) >> 4));
  return p;
}


// From the datasheet.
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of 47445 represents 47445/1024 = 46.333 %RH
humidity_t BME280::compensateHumidity(int32_t adc_H)
{
  int32_t v_x1_u32r;
  v_x1_u32r = (_t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_dig_H4) << 20) - (((int32_t)_dig_H5) * v_x1_u32r)) +
      ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)_dig_H6)) >> 10) * (((v_x1_u32r *
      ((int32_t)_dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
      ((int32_t)_dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)_dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r>>12);
}

#endif /* BME280_ALLOW_FLOAT */

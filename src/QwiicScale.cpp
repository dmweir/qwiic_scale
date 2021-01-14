#include <Arduino.h>
#include "QwiicScale.h"

const __FlashStringHelper* QwiicScale::strerror_f(error_code_t err) {
  switch (err) {
    case SCALE_OK:
      return F("No Error.");
    case NAU7802_I2C_ACK_ERROR:
      return F("NAU7802 sensor did not acknowledge.");
    case NAU7802_I2C_ERROR:
      return F("NAU7802 sensor did not return any data.");
    case NAU7802_TIMEOUT_ERROR:
      return F("NAU7802 timeout occured collecting samples to average.");
    case NAU7802_POWER_UP_ERROR:
      return F("NAU7802 sensor encountered an error powering up.");
    case NAU7802_CAL_AFE_ERROR:
      return F("NAU7802 sensor encountered an error calibrbating afe.");
    case SCALE_EEPROM_READ_CAL_ERROR:
      return F("Unable to read cal factor from eeprom");
    case SCALE_EEPROM_READ_OFFSET_ERROR:
      return F("Unable to read zero offset from eeprom.");
    case SCALE_NOT_CALIBRATED_ERROR:
      return F("Scale is not calibrated");
  }
}


//Call when scale is setup, level, at running temperature, with nothing on it
error_code_t QwiicScale::calculateZeroOffset(uint8_t average_size)
{
  int32_t avg_offset = 0;
  error_code_t err = getAverageReading(&avg_offset, average_size);
  if (err) {
    isCalibrated = false;
    return err;
  }
  setZeroOffset(avg_offset);
  if (useEEPROM)
    storeCalibration();
  return SCALE_OK;
}

//Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
error_code_t QwiicScale::calculateCalibrationFactor(float calibration_weight_grams, uint8_t average_size)
{
  int32_t avg_reading = 0;
  error_code_t err = getAverageReading(&avg_reading, average_size);
  if (err) {
    isCalibrated = false;
    return err;
  }
  float newCalFactor = (avg_reading - zeroOffset) / (float)calibration_weight_grams;
  setCalibrationFactor(newCalFactor);
  if (useEEPROM)
    storeCalibration();
  isCalibrated = true;
  return SCALE_OK;
}


//Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
error_code_t QwiicScale::getAverageWeight(float* avg_weight, uint8_t average_size, bool allow_negative)
{
  int32_t avg_reading = 0;

  if (!isCalibrated) {
    return SCALE_NOT_CALIBRATED_ERROR;
  }

  error_code_t err = getAverageReading(&avg_reading, average_size);
  if (err) {
    return err;
  }

  //Prevent the current reading from being less than zero offset
  //This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
  //causing the weight to be negative or jump to millions of pounds
  if (allow_negative == false)
  {
    if (avg_reading < zeroOffset)
      avg_reading = zeroOffset; //Force reading to zero
  }

  *avg_weight = (avg_reading - zeroOffset) / calibrationFactor;
  return SCALE_OK;
}

//Reads the current system settings from EEPROM
//If anything looks weird, reset setting to default value
error_code_t QwiicScale::readCalibration(void)
{
  float settingCalibrationFactor; //Value used to convert the load cell reading to lbs or kg
  long settingZeroOffset; //Zero value that is found when scale is tared

  //Look up the calibration factor
  EEPROM.get(calFactorLocation, settingCalibrationFactor);
  if ((settingCalibrationFactor == 0xFFFFFFFF) || isnan(settingCalibrationFactor))
  {
    isCalibrated = false;
    calibrationDetected = false;
    zeroOffset = 0;
    calibrationFactor = 1.0f;
    EEPROM.put(calFactorLocation, calibrationFactor);
    return SCALE_EEPROM_READ_CAL_ERROR;
  }
  else {
    calibrationFactor = settingCalibrationFactor;
  }

  //Look up the zero tare point
  EEPROM.get(zeroOffsetLocation, settingZeroOffset);
  if ((settingZeroOffset == 0xFFFFFFFF) || isnan(settingZeroOffset))
  {
    isCalibrated = false;
    calibrationDetected = false;
    zeroOffset = 0;
    calibrationFactor = 1.0f;
    EEPROM.put(zeroOffsetLocation, zeroOffset);
    EEPROM.put(calFactorLocation, calibrationFactor);
    return SCALE_EEPROM_READ_OFFSET_ERROR;
  }
  else {
    zeroOffset = settingZeroOffset;
    calibrationDetected = true;
  }


  if ((zeroOffset == 0) || ((calibrationFactor - 1.0) < 0.001)) {
    isCalibrated = false;
    calibrationDetected = false;
  }
  else
    isCalibrated = true;

  return SCALE_OK;
}

//Record the current system settings to EEPROM
void QwiicScale::storeCalibration(void)
{
  //Get various values from the library and commit them to NVM
  EEPROM.put(calFactorLocation, getCalibrationFactor());
  EEPROM.put(zeroOffsetLocation, getZeroOffset());
}

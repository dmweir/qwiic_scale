#ifndef QWIIC_SCALE_H
#define QWIIC_SCALE_H
#include <Arduino.h>
#include <EEPROM.h>
#include "NAU7802.h"

/* This class improves the error handling of the NAU7802 class from which it inherits.
  It overloads certain methods to provide unambiguous error information. These new methods require
  a pointer to a result parameter that the caller must supply. The caller must check the returned error value before
  using the result parameter to ensure that it is valid.*/

//NOTE: Make sure not to collide for error codes defined by NAU7802 Base Class
#define SCALE_OK                           0
#define SCALE_EEPROM_READ_CAL_ERROR       -1001
#define SCALE_EEPROM_READ_OFFSET_ERROR    -1002
#define SCALE_NOT_CALIBRATED_ERROR        -1003

class QwiicScale : public NAU7802
{
  public:

    QwiicScale(){};
    error_code_t calculateZeroOffset(uint8_t average_size = 64);
    error_code_t calculateCalibrationFactor(float calibration_weight, uint8_t average_size = 64);
    error_code_t getAverageWeight(float *average_weight, uint8_t average_size = 8,  bool allow_negative = true);

    //Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
    void setCalibrationFactor(float newCalFactor){calibrationFactor = newCalFactor;};
    const float getCalibrationFactor(){return calibrationFactor;};
    
    //Sets the internal variable. Useful for users who are loading values from NVM.
    void setZeroOffset(int32_t newZeroOffset){zeroOffset = newZeroOffset;};
    const int32_t getZeroOffset(){return zeroOffset;};
    
    // Error message helper
    const __FlashStringHelper* strerror_f(error_code_t err);

    // Storage and Retrieval of Calibration from EEPROM
    bool useEEPROM = true;
    void storeCalibration(void);
    error_code_t readCalibration(void);
    void QwiicScale::readEEPROM(float* cal_factor, long *offset);

    // Flag to indicate whether settings were read from eeprom. Note: May not be valid.
    bool calibrationDetected = false;
    bool isCalibrated = false;

    void setCalFactorLocation(int eeprom_location) {calFactorLocation = eeprom_location;}
    void setZeroOffsetLocation(int eeprom_location) {calFactorLocation = eeprom_location;}
    const float getCalFactorLocation() {return calFactorLocation;}
    const int getZeroOffsetLocation() {return zeroOffsetLocation;}

  private:
    //EEPROM locations to store 4-byte variables
    int calFactorLocation = 0; //Float, requires 4 bytes of EEPROM
    int zeroOffsetLocation = 10; //Must be more than 4 away from previous spot. Long, requires 4 bytes of EEPROM

    //y = mx + b
    float calibrationFactor = 1.0f; //This is m.
    int32_t zeroOffset = 0;      //This is b
};
#endif //QWIIC_SCALE_H

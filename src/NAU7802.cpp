/*
  This is an Arduino library written for the NAU7802 24-bit wheatstone
  bridge and load cell amplifier.
  By Nathan Seidle @ SparkFun Electronics, March 3nd, 2019
  Modified by David Weir, 1/11/2021

  The NAU7802 is an I2C device that converts analog signals to a 24-bit
  digital signal. This makes it possible to create your own digital scale
  either by hacking an off-the-shelf bathroom scale or by creating your
  own scale using a load cell.

  The NAU7802 is a better version of the popular HX711 load cell amplifier.
  It uses a true I2C interface so that it can share the bus with other
  I2C devices while still taking very accurate 24-bit load cell measurements
  up to 320Hz.

  https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a board from SparkFun!
  https://www.sparkfun.com/products/15242
*/

#include "NAU7802.h"

//Constructor
NAU7802::NAU7802()
{
}

//Sets up the NAU7802 for basic function
//If initialize is true (or not specified), default init and calibration is performed
//If initialize is false, then it's up to the caller to initalize and calibrate
//Returns true upon completion
error_code_t NAU7802::begin(TwoWire &wirePort, bool initialize)
{
  //Get user's options
  i2cPort = &wirePort;

  //Check if the device ack's over I2C
  if (isConnected() == false)
  {
    //There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
    if (isConnected() == false)
      return (NAU7802_I2C_ERROR);
  }

  error_code_t err = NAU7802_OK;

  if (initialize)
  {
    //Reset all registers
    if (err = reset()) {
      return err;
    }
    //Power on analog and digital sections of the scale
    if (err = powerUp()) {
      return err;
    }

    //Set LDO to 3.3V
    if (err = setLDO(NAU7802_LDO_3V3)) {
      return err;
    }

  //Set gain to 128
  if (err = setGain(NAU7802_GAIN_128)) {
    return err;
  }

  //Set samples per second to 80 hz
  if (err = setSampleRate(NAU7802_SPS_80)) {
    return err;
  }

  //Turn off CLK_CHP. From 9.1 power on sequencing.
  if (err = setRegister(NAU7802_ADC, 0x30)) {
    return err;
  }

  //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
  if (err = setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR)) {
    return err;
  }

  //Re-cal analog front end when we change gain, sample rate, or channel
  if (err = calibrateAFE()) {
    return err;
  }
}

return (err);
}

//Returns true if device is present
//Tests for device ack to I2C address
bool NAU7802::isConnected()
{
  i2cPort->beginTransmission(deviceAddress);
  if (i2cPort->endTransmission() != 0)
    return false;
  return true;
}

//Returns true if Cycle Ready bit is set (conversion is complete)
error_code_t NAU7802::available(bool *ready)
{
  uint8_t value;
  error_code_t err = getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL, &value);

  if (err) {
    return err;
  }

  *ready = bool(value);
  return NAU7802_OK;
}

//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
error_code_t NAU7802::calibrateAFE()
{
  error_code_t err = beginCalibrateAFE();
  if (err)
    return err;
  return waitForCalibrateAFE(1000);
}

//Begin asynchronous calibration of the analog front end.
// Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
error_code_t NAU7802::beginCalibrateAFE()
{
  return setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}

//Check calibration status.
NAU7802_Cal_Status NAU7802::calAFEStatus()
{
  uint8_t value;
  error_code_t err = getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2, &value);

  if (err)
    return err;

  if (value)
  {
    return NAU7802_CAL_IN_PROGRESS;
  }

  err = getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2, &value);
  if (err)
    return err;

  if (value)
  {
    return NAU7802_CAL_FAILURE;
  }

  // Calibration passed
  return NAU7802_CAL_SUCCESS;
}

//Wait for asynchronous AFE calibration to complete with optional timeout.
//If timeout is not specified (or set to 0), then wait indefinitely.
error_code_t NAU7802::waitForCalibrateAFE(uint32_t timeout_ms)
{
  uint32_t begin = millis();
  NAU7802_Cal_Status cal_status;

  while ((cal_status = calAFEStatus()) == NAU7802_CAL_IN_PROGRESS)
  {
    if ((timeout_ms > 0) && ((millis() - begin) > timeout_ms))
    {
      return NAU7802_CAL_AFE_ERROR;
    }
    delay(1);
  }

  return NAU7802_OK;
}

//Set the readings per second
//10, 20, 40, 80, and 320 samples per second is available
error_code_t NAU7802::setSampleRate(uint8_t rate)
{
  if (rate > 0b111)
    rate = 0b111; //Error check

  uint8_t value;
  error_code_t err = getRegister(NAU7802_CTRL2, &value);
  if (err)
    return err;

  value &= 0b10001111; //Clear CRS bits
  value |= rate << 4;  //Mask in new CRS bits

  return (setRegister(NAU7802_CTRL2, value));
}

//Select between 1 and 2
error_code_t NAU7802::setChannel(uint8_t channelNumber)
{
  if (channelNumber == NAU7802_CHANNEL_1)
    return (clearBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 1 (default)
  else
    return (setBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 2
}

//Power up digital and analog sections of scale
error_code_t NAU7802::powerUp()
{
  error_code_t err = setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  if (err)
    return err;

  err = setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);
  if (err)
    return err;

  //Wait for Power Up bit to be set - takes approximately 200us
  uint8_t counter = 0;
  while (1)
  {
    uint8_t value;
    err = getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL, &value);
    if (err)
      return err;

    if (value > 0)
      return NAU7802_OK; //Good to go
    delay(1);
    if (counter++ > 100)
      return NAU7802_POWER_UP_ERROR; //Error
  }
}

//Puts scale into low-power mode
error_code_t NAU7802::powerDown()
{
  error_code_t err = clearBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  if (err)
    return err;

  return (clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));
}

//Resets all registers to Power Off Defaults
error_code_t NAU7802::reset()
{
  error_code_t err = setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL); //Set RR
  if (err)
    return err;
  delay(1);
  return (clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)); //Clear RR to leave reset state
}

//Set the onboard Low-Drop-Out voltage regulator to a given value
//2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
error_code_t NAU7802::setLDO(uint8_t ldoValue)
{
  if (ldoValue > 0b111)
    ldoValue = 0b111; //Error check

  //Set the value of the LDO
  uint8_t value;
  error_code_t err = getRegister(NAU7802_CTRL1, &value);
  if (err)
    return err;

  value &= 0b11000111;    //Clear LDO bits
  value |= ldoValue << 3; //Mask in new LDO bits
  err = setRegister(NAU7802_CTRL1, value);
  if (err)
    return err;

  return (setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL)); //Enable the internal LDO
}

//Set the gain
//x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
error_code_t NAU7802::setGain(uint8_t gainValue)
{
  if (gainValue > 0b111)
    gainValue = 0b111; //Error check

  uint8_t value;
  error_code_t err = getRegister(NAU7802_CTRL1, &value);
  if (err)
    return err;

  value &= 0b11111000; //Clear gain bits
  value |= gainValue;  //Mask in new bits

  return (setRegister(NAU7802_CTRL1, value));
}

//Get the revision code of this IC
error_code_t NAU7802::getRevisionCode(uint8_t *revisionCode)
{
  error_code_t err = getRegister(NAU7802_DEVICE_REV, revisionCode);

  *revisionCode &= 0x0F;

  return err;
}

byte NAU7802::i2c_write(uint8_t registerAddress, uint8_t* value) {
  int tries = 3;
  byte ret;

  while (tries) {
    i2cPort->beginTransmission(deviceAddress);
    i2cPort->write(registerAddress);
    if (value != NULL){
      i2cPort->write(*value);
    }
    ret = i2cPort->endTransmission();

    switch (ret){
      case 1:
      case 2:
        // try again on NACK
        tries--;
        break;
      default:
        // Return everything else
        return ret;
    }
  }

  return ret;
}

//Returns 24-bit reading
//Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
error_code_t NAU7802::getReading(int32_t *result)
{
  byte ret = i2c_write(NAU7802_ADCO_B2, NULL);

  if (ret == 1){
    return NAU7802_I2C_DATA_TOO_BIG_ERROR;
  }
  else if (ret == 2){
    return NAU7802_I2C_NACK_ADDR_ERROR;
  }
  else if (ret == 3){
    return NAU7802_I2C_NACK_DATA_ERROR;
  }
  else if (ret == 4){
    return NAU7802_I2C_ERROR;
  }


  i2cPort->requestFrom((uint8_t)deviceAddress, (uint8_t)3);

  if (i2cPort->available())
  {
    uint32_t valueRaw = (uint32_t)i2cPort->read() << 16; //MSB
    valueRaw |= (uint32_t)i2cPort->read() << 8;          //MidSB
    valueRaw |= (uint32_t)i2cPort->read();               //LSB

    // the raw value coming from the ADC is a 24-bit number, so the sign bit now
    // resides on bit 23 (0 is LSB) of the uint32_t container. By shifting the
    // value to the left, I move the sign bit to the MSB of the uint32_t container.
    // By casting to a signed int32_t container I now have properly recovered
    // the sign of the original value
    int32_t valueShifted = (int32_t)(valueRaw << 8);

    // shift the number back right to recover its intended magnitude
    *result = (valueShifted >> 8);

    return NAU7802_OK;
  }

  return NAU7802_I2C_NO_DATA_ERROR;
}

error_code_t NAU7802::getAverageReading(int32_t *average, uint8_t average_size)
{
  error_code_t err;
  int32_t value;
  long total = 0;
  uint8_t samplesAquired = 0;
  bool ready = false;

  // Hard-coded for 80 hz rate for now
  unsigned long  timeout = average_size * 13;

  unsigned long startTime = millis();
  while (samplesAquired < average_size)
  {
    err = available(&ready);
    if (err) {
      return err;
    }

    if (ready == true)
    {

      if (err = getReading(&value)) {
        return err;
      }

      total += value;
      samplesAquired++;
      ready = false;
    }

    if ((millis() - startTime) > timeout)
      return NAU7802_TIMEOUT_ERROR;
  }

  *average = total / average_size;

  return NAU7802_OK;
}

//Set Int pin to be high when data is ready (default)
error_code_t NAU7802::setIntPolarityHigh()
{
  return (clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
}

//Set Int pin to be low when data is ready
error_code_t NAU7802::setIntPolarityLow()
{
  return (setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
}

//Mask & set a given bit within a register
error_code_t NAU7802::setBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value;
  error_code_t err = getRegister(registerAddress, &value);
  if (err) {
    return err;
  }

  value |= (1 << bitNumber); //Set this bit
  return setRegister(registerAddress, value);
}

//Mask & clear a given bit within a register
error_code_t NAU7802::clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value;
  error_code_t err = getRegister(registerAddress, &value);
  if (err) {
    return err;
  }

  value &= ~(1 << bitNumber); //Set this bit
  return setRegister(registerAddress, value);
}

//Return a given bit within a register
error_code_t NAU7802::getBit(uint8_t bitNumber, uint8_t registerAddress, uint8_t *registerContents)
{
  error_code_t err = getRegister(registerAddress, registerContents);

  if (err) {
    return err;
  } else {
    (*registerContents) &= (1 << bitNumber); //Clear all but this bit
    return NAU7802_OK;
  }
}

//Get contents of a register
error_code_t NAU7802::getRegister(uint8_t registerAddress, uint8_t *registerContents)
{
  byte ret = i2c_write(registerAddress, NULL);
  if (ret == 1){
    return NAU7802_I2C_DATA_TOO_BIG_ERROR;
  }
  else if (ret == 2){
    return NAU7802_I2C_NACK_ADDR_ERROR;
  }
  else if (ret == 3){
    return NAU7802_I2C_NACK_DATA_ERROR;
  }
  else if (ret == 4){
    return NAU7802_I2C_ERROR;
  }

  i2cPort->requestFrom((uint8_t)deviceAddress, (uint8_t)1);

  if (i2cPort->available()) {
    *registerContents = i2cPort->read();
    return NAU7802_OK;
  }

  return NAU7802_I2C_NO_DATA_ERROR;
}

//Send a given value to be written to given address
//Return true if successful
error_code_t NAU7802::setRegister(uint8_t registerAddress, uint8_t value)
{

  byte ret = i2c_write(registerAddress, &value);

  if (ret == 1){
    return NAU7802_I2C_DATA_TOO_BIG_ERROR;
  }
  else if (ret == 2){
    return NAU7802_I2C_NACK_ADDR_ERROR;
  }
  else if (ret == 3){
    return NAU7802_I2C_NACK_DATA_ERROR;
  }
  else if (ret == 4){
    return NAU7802_I2C_ERROR;
  }

  return NAU7802_OK;
}

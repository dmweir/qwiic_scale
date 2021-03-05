#include <Wire.h>
#include "ArduinoJson.h"
#include "QwiicScale.h"

// scale configurataion
#define SERVER_ID         0
#define SEND_RATE         10 //hz
#define SAMPLERATE        80
#define AVG_SIZE          8

// jsonrpc error codes
#define PARSE_ERROR       -32700
#define INVALID_REQUEST   -32600
#define METHOD_NOT_FOUND  -32601
#define INVALID_PARAMS    -32602
#define INTERNAL_ERROR    -32603

// operation modes
#define REQUEST 0
#define CONTINUOUS 1

// serial settings
#define BAUDRATE          115200

// global variables
QwiicScale Scale;
int sample_mode = REQUEST;

// macros
#define STRCMPI(x,y) !strcasecmp(x,y)
#define FREEZE while(1) continue;


// Dispatches on the requested jsonrpc method
void dispatch(const unsigned long id, const char *method, const JsonVariant &params)
{
  if (STRCMPI(method, "tare"))
  {
    tare(id, params);
  }
  else if (STRCMPI(method, "calibrate"))
  {
    calibrate(id, params);
  }
  else if (STRCMPI(method, "reset_calibration"))
  {
    reset_calibration(id, params);
  }
  else if (STRCMPI(method, "get_calibration"))
  {
    get_calibration(id, params);
  }
  else if (STRCMPI(method, "get_average_weight"))
  {
    get_average_weight(id, params);
  }
  else if (STRCMPI(method, "get_average_reading"))
  {
    get_average_reading(id, params);
  }
  else if (STRCMPI(method, "get_status"))
  {
    get_status(id, params);
  }
  else if (STRCMPI(method, "get_sensors"))
  {
    get_sensors(id, params);
  }
  //  else if (STRCMPI(method, "reset")) {
  //    reset_scale(id, params);
  //  }
  //  else if (STRCMPI(method, "power_up")) {
  //    power_up(id, params);
  //  }
  //  else if (STRCMPI(method, "power_down")) {
  //    power_down(id, params);
  //  }
  else if (STRCMPI(method, "change_mode"))
  {
    change_mode(id, params);
  }
  else
  {
    jsonrpc_method_not_found(id);
  }
}

// Scale RPC Methods
// All Methods must have the signature void f(uint32_t id, const JsonVariant& params)
void calibrate(const unsigned long id, const JsonVariant &params)
{
  StaticJsonDocument<256> reply;

  float weight = params["calibration_weight"] | -1.0f;
  long num_readings = params["average_size"] | -1L;

  if ((weight < 1) || (weight > 500))
  {
    jsonrpc_invalid_params(id, F("By-name parameter 'weight' is missing or outside range."));
    return;
  }
  else if ((num_readings < 1) || (num_readings > 64))
  {
    jsonrpc_invalid_params(id, F("By-name parameter 'average_size' is missing or > 64."));
    return;
  }

  error_code_t err = Scale.calculateCalibrationFactor(weight, num_readings);

  if (!err)
  {
    reply["id"] = id;
    JsonObject result = reply.createNestedObject("result");
    result["timestamp"] = millis();
    result["calibration_factor"] = Scale.getCalibrationFactor();
    result["zero_offset"] = Scale.getZeroOffset();
    serializeJson(reply, Serial);
    Serial.println();
  }
  else
  {
    jsonrpc_scale_error(id, err);
  }
}

// Tare the scale so that current value is new zero point.
void tare(const unsigned long id, const JsonVariant &params)
{
  StaticJsonDocument<128> reply;
  long num_readings = params["average_size"] | -1L;

  if ((num_readings < 1) || (num_readings > 64))
  {
    jsonrpc_invalid_params(id, F("By-name parameter 'average_size' is missing or > 64."));
    return;
  }

  error_code_t err = Scale.calculateZeroOffset(num_readings);

  if (!err)
  {
    reply["id"] = id;
    JsonObject result = reply.createNestedObject("result");
    result["timestamp"] = millis();
    result["calibration_factor"] = Scale.getCalibrationFactor();
    result["zero_offset"] = Scale.getZeroOffset();
    serializeJson(reply, Serial);
    Serial.println();
  }
  else
  {
    jsonrpc_scale_error(id, err);
  }
}

// Change the mode the microcontroller
void change_mode(const unsigned long id, const JsonVariant &params)
{

  StaticJsonDocument<128> reply;
  const char *mode = params["mode"] | "invalid";

  if (STRCMPI(mode, "invalid"))
  {
    jsonrpc_invalid_params(id, F("By-name parameter 'mode' is missing or invalid"));
    return;
  }
  if (STRCMPI(mode, "request"))
    sample_mode = REQUEST;
  else if (STRCMPI(mode, "continuous"))
    sample_mode = CONTINUOUS;

  jsonrpc_ack(id);
}

void reset_calibration(const unsigned long id, const JsonVariant &params)
{
  float cal_factor = params["cal_factor"] | 1.0f;
  long zero_offset = params["zero_offset"] | 0L;
  bool cal_state = params["is_calibrated"] | false;

  Scale.setCalibrationFactor(cal_factor);
  Scale.setZeroOffset(zero_offset);
  Scale.isCalibrated = cal_state;
  Scale.calibrationDetected = false;
  Scale.storeCalibration();

  StaticJsonDocument<128> reply;
  reply["id"] = id;
  JsonObject result = reply.createNestedObject("result");
  result["timestamp"] = millis();
  result["calibration_factor"] = Scale.getCalibrationFactor();
  result["zero_offset"] = Scale.getZeroOffset();
  serializeJson(reply, Serial);
  Serial.println();
}

void get_calibration(const unsigned long id, const JsonVariant &params)
{
  StaticJsonDocument<128> reply;
  reply["id"] = id;
  JsonObject result = reply.createNestedObject("result");
  result["timestamp"] = millis();
  result["calibration_factor"] = Scale.getCalibrationFactor();
  result["zero_offset"] = Scale.getZeroOffset();
  serializeJson(reply, Serial);
  Serial.println();
}

void get_average_reading(const unsigned long id, const JsonVariant &params)
{
  StaticJsonDocument<128> reply;
  long num_readings = params["average_size"] | -1L;

  if ((num_readings < 1) || (num_readings > 64))
  {
    jsonrpc_invalid_params(id, F("By-name parameter 'average_size' is missing or > 64."));
    return;
  }

  int32_t avg_reading;
  error_code_t err = Scale.getAverageReading(&avg_reading, num_readings);

  if (!err)
  {
    reply["id"] = id;
    JsonObject result = reply.createNestedObject("result");
    result["timestamp"] = millis();
    result["raw_avg"] = avg_reading;
    result["num_samples"] = num_readings;
    serializeJson(reply, Serial);
    Serial.println();
  }
  else
  {
    jsonrpc_scale_error(id, err);
  }
}

void get_average_weight(const unsigned long id, const JsonVariant &params)
{
  StaticJsonDocument<128> reply;
  long num_readings = params["average_size"] | -1L;
  bool allow_negative = params["allow_negative"] | true;

  if ((num_readings < 1) || (num_readings > 64))
  {
    jsonrpc_invalid_params(id, F("By-name parameter 'num_readings' is missing or > 64."));
    return;
  }

  float avg_weight;
  error_code_t err = Scale.getAverageWeight(&avg_weight, num_readings, allow_negative);

  if (!err)
  {
    reply["id"] = id;
    JsonObject result = reply.createNestedObject("result");
    result["timestamp"] = millis();
    result["weight_avg"] = avg_weight;
    result["num_samples"] = num_readings;
    serializeJson(reply, Serial);
    Serial.println();
  }
  else
  {
    jsonrpc_scale_error(id, err);
  }
}

void get_status(const unsigned long id, const JsonVariant &params)
{
  StaticJsonDocument<128> reply;
  reply["id"] = id;
  JsonObject result = reply.createNestedObject("result");
  result["timestamp"] = millis();
  result["is_scale_connected"] = Scale.isConnected();
  result["is_calibrated"] = Scale.isCalibrated;
  result["is_cal_detected"] = Scale.calibrationDetected;
  serializeJson(reply, Serial);
  Serial.println();
}

void get_sensors(uint32_t id, const JsonVariant &params)
{
  float avg_weight;
  StaticJsonDocument<128> reply;

  error_code_t err = Scale.getAverageWeight(&avg_weight, AVG_SIZE);

  if (!err)
  {
    reply["id"] = id;
    JsonObject result = reply.createNestedObject("result");
    result["timestamp"] = millis();
    result["weight_avg"] = avg_weight;
    result["num_samples"] = AVG_SIZE;
    serializeJson(reply, Serial);
    Serial.println();
  }
  else
  {
    jsonrpc_scale_error(SERVER_ID, err);
  }
}

// Continuous Streaming Mode
void stream_sensors(void)
{
  static bool streaming_error;
  float avg_weight;
  StaticJsonDocument<128> reply;

  error_code_t err = Scale.getAverageWeight(&avg_weight, AVG_SIZE);

  if (!err)
  {
    streaming_error = false;
    reply["id"] = SERVER_ID;
    JsonObject result = reply.createNestedObject("result");
    result["timestamp"] = millis();
    result["weight_avg"] = avg_weight;
    result["num_samples"] = AVG_SIZE;
    serializeJson(reply, Serial);
    Serial.println();
  }
  else
  {
    // Only send the first error encountered
    if (!streaming_error)
    {
      jsonrpc_scale_error(SERVER_ID, err);
      streaming_error = true;
    }
  }
}

// Acknowledgement Response
void jsonrpc_ack(const unsigned long id)
{
  StaticJsonDocument<128> reply;
  reply["id"] = id;
  JsonObject result = reply.createNestedObject("result");
  result["timestamp"] = millis();
  serializeJson(reply, Serial);
  Serial.println();
}

// Error Response Helper Functions
void jsonrpc_scale_error(uint32_t id, error_code_t scale_err)
{
  StaticJsonDocument<128> reply;
  JsonObject err = reply.createNestedObject("error");
  err["timestamp"] = millis();
  err["code"] = static_cast<int>(scale_err);
  err["message"] = Scale.strerror_f(scale_err);
  reply["id"] = id;
  serializeJson(reply, Serial);
  Serial.println();
}

void jsonrpc_parse_error(const DeserializationError &error)
{
  StaticJsonDocument<128> reply;
  JsonObject err = reply.createNestedObject("error");
  err["code"] = PARSE_ERROR;
  err["message"] = F("Error parsing received JSON.");
  err["data"] = error.f_str();
  serializeJson(reply, Serial);
  Serial.println();
}

void jsonrpc_invalid_request(void)
{
  StaticJsonDocument<128> reply;
  JsonObject err = reply.createNestedObject("error");
  err["code"] = INVALID_REQUEST;
  err["message"] = F("The JSON sent is not a valid Request object.");
  serializeJson(reply, Serial);
  Serial.println();
}

void jsonrpc_method_not_found(const unsigned long id)
{
  StaticJsonDocument<128> reply;
  JsonObject err = reply.createNestedObject("error");
  err["code"] = METHOD_NOT_FOUND;
  err["message"] = F("The method does not exist.");
  reply["id"] = id;
  serializeJson(reply, Serial);
  Serial.println();
}

void jsonrpc_invalid_params(const unsigned long id, const __FlashStringHelper *data)
{
  StaticJsonDocument<128> reply;
  JsonObject err = reply.createNestedObject("error");
  err["code"] = INVALID_PARAMS;
  err["message"] = F("Invalid method parameter(s).");
  err["data"] = data;
  reply["id"] = id;
  serializeJson(reply, Serial);
  Serial.println();
}

// INITIALIZATION (ONLY RUNS ONCE AT THE BEGINNING)
void setup()
{
  error_code_t err;

  Serial.begin(BAUDRATE, SERIAL_8N1);

  while (!Serial)
    continue; //Freeze
  Serial.flush();
  Serial.setTimeout(100);

  Wire.begin();

  StaticJsonDocument<256> reply;
  err = Scale.begin();
  if (err)
  {
    jsonrpc_scale_error(SERVER_ID, err);
    FREEZE
  }

  // Configure Scale
  err = Scale.setSampleRate(NAU7802_SPS_80);
  if (err)
  {
    jsonrpc_scale_error(SERVER_ID, err);
    FREEZE
  }

  // Internal calibration.
  // Recommended after power up, gain changes, sample rate changes, or channel change
  err = Scale.calibrateAFE();
  if (err)
  {
    jsonrpc_scale_error(SERVER_ID, err);
    FREEZE
  }

  // Load zeroOffset and calibrationFactor from EEPROM
  err = Scale.readCalibration();
  if (err)
  {
    jsonrpc_scale_error(SERVER_ID, err);
  }
}
// MAIN LOOP (CALLED INDEFINITELY)
void loop()
{

  StaticJsonDocument<128> request;

  if (Serial.available())
  {
    String request_line = Serial.readStringUntil('\n');
    DeserializationError err = deserializeJson(request, request_line);
    if (err)
    {
      jsonrpc_parse_error(err);
      // flush any remaining input
//      while (Serial.available())
//        Serial.read();
      return;
    }
    //flush any remaining input
//    while (Serial.available())
//      Serial.read();
    // Parse the request
    const char *method = request["method"] | "?";
    JsonVariant params = request["params"];
    unsigned long  id = request["id"] | 0uL;

    if ((strcmp(method, "?") != 0) && (id > 0))
    {
      dispatch(id, method, params);
    }
    else
    {
      jsonrpc_invalid_request();
    }
  }
  else if (sample_mode == CONTINUOUS)
  {
    stream_sensors();
  }
}

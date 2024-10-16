#include "CtrlTracker.h"

/**
 * Constructor
*/
CtrlTracker::CtrlTracker() {}

/**
 * Initialize the tracker sensor controller
 * 
*/
void CtrlTracker::init(uint8_t _pin_sensor, uint8_t _num_sensor) {
  #ifdef ACTIVE_TRACKER_SERIAL_SETUP
    Serial.printf("CtrlTracker init define pin sensor %i\n", _pin_sensor);
  #endif
  pin_sensor = _pin_sensor;
  num_sensor = _num_sensor;

  mutex_tracker_sensor = xSemaphoreCreateMutex();  // crete a mutex object

  createParallelFunctionReadTrackerSensor();
}

void CtrlTracker::createParallelFunctionReadTrackerSensor() {
  #ifdef ACTIVE_TRACKER_SERIAL_SETUP
    Serial.printf("Setup parallel task: readTrackerSensor\n");
  #endif
  xTaskCreatePinnedToCore (
    this->readTrackerSensor,   // Function to implement the task
    "readTrackerSensor" + num_sensor, // Name of the task
    2000,            // Stack size in words
    this,            // Task input parameter
    1,              // Priority of the task
    NULL,            // Task handle.
    0                // Core where the task should run
  );
}

void CtrlTracker::readTrackerSensor(void *pvParameters) {
  CtrlTracker *_tracker_sensor = (CtrlTracker *) pvParameters;  

  uint8_t _pin_sensor = _tracker_sensor->getPinSensor();
  uint8_t _num_sensor = _tracker_sensor->getNumSensor();
  String _num_sensor_formated = _tracker_sensor->print2Zeros(_num_sensor);
  bool _tracker_sensor_white_detected;

  uint8_t _robot_status = _tracker_sensor->getRobotStatus();
  uint8_t _delay_control = TRACKER_SENSOR_DELAY_RUN_MIN;
  uint16_t _sensor_value = TRACKER_ANALOG_UPPER_LIMIT;

  #ifdef ACTIVE_TRACKER_SERIAL
    long _prev_time = micros();
  #endif

  #ifdef ACTIVE_TRACKER_SERIAL_SETUP
    Serial.printf("Loop readTrackerSensor Sensor %s with pin %i\n", _num_sensor_formated, _pin_sensor); 
  #endif
  while(1) {
    _robot_status = _tracker_sensor->getRobotStatus();
    if (_robot_status == ROBOT_STATUS_RUN) {
      #ifdef ACTIVE_TRACKER_SERIAL
        long _ini_time = micros();
      #endif
      _sensor_value = analogRead(_pin_sensor);
      #ifdef ACTIVE_TRACKER_SERIAL
        Serial.printf("%i Tracker sensor %s value :%i\n", millis() - _prev_time, _num_sensor_formated, _sensor_value);
        _prev_time = millis();
      #endif
      if(_sensor_value < TRACKER_ANALOG_UPPER_LIMIT){
        #ifdef ACTIVE_TRACKER_SERIAL
          if (!_tracker_sensor_white_detected) {
            long _end_time = micros();
            Serial.printf("%lu tracker sensor %s WHITE DETECTED, time read: %ius\n", micros(), _num_sensor_formated, _end_time - _ini_time);
          }
        #endif
        _tracker_sensor_white_detected = true;
        _tracker_sensor->setTrackerSensorValues(_tracker_sensor_white_detected, true);
        // _last_white_line_detected = millis();
        // if (_delay_control != TRACKER_SENSOR_DELAY_RUN_MIN) {
        //   _delay_control = TRACKER_SENSOR_DELAY_RUN_MIN;
        // }
      }
      else {
        #ifdef ACTIVE_TRACKER_SERIAL
          if (_tracker_sensor_white_detected) {
            long _end_time = micros();
            Serial.printf("%lu tracker sensor %s WHITE NOT DETECTED, time read: %ius\n", micros(), _num_sensor_formated, _end_time - _ini_time);
          }
        #endif        
        _tracker_sensor_white_detected = false;
        _tracker_sensor->setTrackerSensorValues(_tracker_sensor_white_detected, true);

        // if (_delay_control != TRACKER_SENSOR_DELAY_RUN_MAX) {
        //   if (millis() - _last_white_line_detected > TRACKER_SENSOR_DELAY_RUN_TIME) {
        //     _delay_control = TRACKER_SENSOR_DELAY_RUN_MAX;
        //   }
        // }
      }

      delay(_delay_control);
    }
    else if (_robot_status == ROBOT_STATUS_COUNTDOWN){
      delay(TRACKER_SENSOR_DELAY_PRE_RUN);
    }
    else {
      delay(TRACKER_SENSOR_DELAY_WAITING);
    }
  }
}

uint8_t CtrlTracker::getPinSensor(){
  return pin_sensor;
}

uint8_t CtrlTracker::getNumSensor(){
  return num_sensor;
}

String CtrlTracker::print2Zeros(uint16_t value) {
  char buf[3];  // 2 digit integer plus null terminator.
  sprintf(buf, "%02d", value);
  return String(buf);
}

void CtrlTracker::setTrackerSensorValues(bool _tracker_sensor_white_detected, bool _tracker_sensor_changed){
  if (xSemaphoreTake (mutex_tracker_sensor, portMAX_DELAY)) {
    tracker_sensor_white_detected = _tracker_sensor_white_detected;
    tracker_sensor_changed = _tracker_sensor_changed;
    xSemaphoreGive (mutex_tracker_sensor);
  }
}

TrackerSensorValuesStruct CtrlTracker::getTrackerSensorValues(){
  TrackerSensorValuesStruct _tracker_sensor_values;
  bool _white_detected;
  bool _value_changed;
  if (xSemaphoreTake (mutex_tracker_sensor, portMAX_DELAY)) {
    _white_detected = tracker_sensor_white_detected;
    _value_changed = tracker_sensor_changed;
    tracker_sensor_changed = false;
    xSemaphoreGive (mutex_tracker_sensor);
  }
  _tracker_sensor_values = {_white_detected, _value_changed};
  return _tracker_sensor_values;
}

uint8_t CtrlTracker::getRobotStatus() {
  return robot_status;
}

void CtrlTracker::setRobotStatus(uint8_t _robot_status) {
  robot_status = _robot_status;
}

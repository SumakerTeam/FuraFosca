#include "CtrlLaser.h"

/**
 * Constructor
*/
CtrlLaser::CtrlLaser() {}

/**
 * Initialize the laser sensor controller
 * 
*/
void CtrlLaser::init(uint8_t _pin_xshut, uint8_t _num_sensor) {
  #ifdef ACTIVE_LASER_SERIAL_SETUP
    Serial.printf("CtrlLaser init define pin_xshut %i\n", pin_xshut);
  #endif
  pin_xshut = _pin_xshut;
  num_sensor = _num_sensor;
  mutex_laser_sensor = xSemaphoreCreateMutex();  // crete a mutex object
  initLaserSensor();
  createParallelFunctionReadLaserSensor();
}

void CtrlLaser::resetLaserSensor(uint8_t _pin_xshut, uint8_t _num_sensor) {
  #ifdef ACTIVE_LASER_SERIAL_SETUP
    String _num_sensor_formated = print2Zeros(_num_sensor);
    Serial.printf("Laser Reset sensor: %s\n", _num_sensor_formated);
  #endif

  // Stop driving this sensor's XSHUT low. This should allow the carrier
  // board to pull it high. (We do NOT want to drive XSHUT high since it is
  // not level shifted.) Then wait a bit for the sensor to start up.
  // all reset
  pinMode(_pin_xshut, OUTPUT);
  digitalWrite(_pin_xshut, LOW);    
  delay(100);
}

void CtrlLaser::initLaserSensor() {
  #ifdef ACTIVE_LASER_SERIAL_SETUP
    String _num_sensor_formated = print2Zeros(num_sensor);
    Serial.printf("Laser Init sensor: %s\n", _num_sensor_formated);
  #endif

  #ifdef ACTIVE_LASER_SERIAL_SETUP
    Serial.printf("initLaserSensor: %s\n", _num_sensor_formated);
  #endif

  pinMode(pin_xshut, INPUT);
  delay(10);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    #ifdef ACTIVE_LASER_SERIAL_SETUP
      Serial.printf("LASER ERROR. Failed to detect and initialize laser sensor: %s\n", _num_sensor_formated);
    #endif
    // while (1);
    sensor_status = false;
  }
  else {
    #ifdef ACTIVE_LASER_SERIAL_SETUP
      Serial.printf("Laser %s detected and initialized\n", _num_sensor_formated);
    #endif
    sensor_status = true;
  }

  // Each sensor must have its address changed to a unique value other than
  // the default of 0x29 (except for the last one, which could be left at
  // the default). To make it simple, we'll just count up from 0x2A.
  sensor.setAddress(0x29 + num_sensor);
  #ifdef LASER_SENSOR_CONTINOUS_READING
    // sensor.startContinuous(50);
    sensor.startContinuous();
  #endif

  #ifdef ACTIVE_LASER_SERIAL_SETUP
    Serial.printf("activeHighSpeedSensor: %s\n", _num_sensor_formated);
    // reduce timing budget to 20 ms (default is about 33 ms)
  #endif  
  #ifdef LASER_SENSOR_HIGH_SPEED
    sensor.setMeasurementTimingBudget(20000);
  #endif
}

void CtrlLaser::createParallelFunctionReadLaserSensor() {
  #ifdef ACTIVE_LASER_SERIAL_SETUP
    Serial.printf("Setup parallel task: readLaserSensor\n");
  #endif
  xTaskCreatePinnedToCore (
    this->readLaserSensor,   // Function to implement the task
    "readLaserSensor" + num_sensor, // Name of the task
    5000,            // Stack size in words
    this,            // Task input parameter
    4,               // Priority of the task
    NULL,            // Task handle.
    0                // Core where the task should run
  );
}

void CtrlLaser::readLaserSensor(void *pvParameters) {
  CtrlLaser *_laser_sensor = (CtrlLaser *) pvParameters;

  uint8_t _pin_xshut = _laser_sensor->getPinXshut();
  uint8_t _num_sensor = _laser_sensor->getNumSensor();
  String _num_sensor_formated = _laser_sensor->print2Zeros(_num_sensor);
  VL53L0X _sensor = * _laser_sensor->getSensor();
  bool _sensor_status = _laser_sensor->getSensorStatus();
  uint8_t _robot_status = _laser_sensor->getRobotStatus();

  uint16_t _laser_sensor_distance;

  #ifdef ACTIVE_LASER_SERIAL_SETUP
    Serial.printf("Loop readLaserSensor Sensor %s\n", _num_sensor_formated); 
  #endif

  #ifdef ACTIVE_LASER_SERIAL
    long _prev_time = micros();
  #endif

  long delay_reference = 5;
  while(1) {
    _robot_status = _laser_sensor->getRobotStatus();
    if (_sensor_status && _robot_status == ROBOT_STATUS_RUN) {
      long _ini_time = millis();
      #ifdef ACTIVE_LASER_SERIAL
        long _ini_time_serial = micros();
      #endif
      #ifdef ACTIVE_LASER_SERIAL_PLOTTER
        long _ini_time_plotter = micros();
      #endif
      // uint16_t _range;
      #ifdef LASER_SENSOR_CONTINOUS_READING
        _laser_sensor_distance = _sensor.readRangeContinuousMillimeters();
      #else
        _laser_sensor_distance = _sensor.readRangeSingleMillimeters();
      #endif
      if (_laser_sensor_distance > 1500) {
        _laser_sensor_distance = 1500;
      }
      _laser_sensor->setLaserSensorValues(_laser_sensor_distance, true);
      int time_delay = millis() - _ini_time;
      #ifdef ACTIVE_LASER_SERIAL
        long _end_time = micros();
        if (_sensor.timeoutOccurred()) { 
          Serial.printf("TIMEOUT laser sensor: %s\n", _num_sensor_formated); 
        }
        String _distance = _laser_sensor->print4Zeros(_laser_sensor_distance);
        String _time_read = _laser_sensor->print6Zeros(_end_time - _ini_time_serial);
        Serial.printf("%i Laser sensor %s-> distance: %smm, time read: %sus, time_delay: %i,delay_reference previous: %i\n", millis() - _prev_time, _num_sensor_formated, _distance, _time_read, time_delay, delay_reference);
        _prev_time = millis();
      #endif

      if (time_delay > 3 ) {
        if (delay_reference < 20) {
          delay_reference++;
        }
      }
      else if (time_delay <=1) {
        if (delay_reference > 1) {
          delay_reference--;
        }
      }

      #ifdef ACTIVE_LASER_SERIAL_PLOTTER
        Serial.printf("Laser%i:%i Time%i:%i delay%i:%i %i\n", _num_sensor, _laser_sensor_distance, _num_sensor, (micros() - _ini_time_plotter) / 1000, _num_sensor,delay_reference, time_delay);
      #endif

      delay(delay_reference);
    }
    else if (_robot_status == ROBOT_STATUS_PRE_RUN){
      delay(5);
    }
    else {
      delay(100);
    }
  }
}

void CtrlLaser::setLaserSensorValues(uint16_t _laser_sensor_distance, bool _laser_sensor_changed){
  if (xSemaphoreTake (mutex_laser_sensor, portMAX_DELAY)) {
    laser_sensor_distance = _laser_sensor_distance;
    laser_sensor_changed = _laser_sensor_changed;
    xSemaphoreGive (mutex_laser_sensor);
  }
}

LaserSensorValuesStruct CtrlLaser::getLaserSensorValues(){
  // LaserSensorValuesStruct _laser_sensor_values;
  uint16_t _distance;
  bool value_changed;
  if (xSemaphoreTake (mutex_laser_sensor, portMAX_DELAY)) {
    _distance = laser_sensor_distance;
    value_changed = laser_sensor_changed;
    laser_sensor_changed = false;
    xSemaphoreGive (mutex_laser_sensor);
  }
  LaserSensorValuesStruct _laser_sensor_values = {_distance, value_changed};
  return _laser_sensor_values;
}

uint8_t CtrlLaser::getPinXshut(){
  return pin_xshut;
}

uint8_t CtrlLaser::getNumSensor(){
  return num_sensor;
}

VL53L0X * CtrlLaser::getSensor() {
  return &sensor;
}

bool CtrlLaser::getSensorStatus() {
  return sensor_status;
}

uint8_t CtrlLaser::getRobotStatus() {
  return robot_status;
}

void CtrlLaser::setRobotStatus(uint8_t _robot_status) {
  robot_status = _robot_status;
}

String CtrlLaser::print2Zeros(uint16_t value) {
  char buf[3];  // 2 digit integer plus null terminator.
  sprintf(buf, "%02d", value);
  return String(buf);
}

String CtrlLaser::print4Zeros(uint16_t value) {
  char buf[5];  // 4 digit integer plus null terminator.
  sprintf(buf, "%04d", value);
  return String(buf);
}

String CtrlLaser::print6Zeros(uint16_t value) {
  char buf[7];  // 3 digit integer plus null terminator.
  sprintf(buf, "%06d", value);
  return String(buf);
}
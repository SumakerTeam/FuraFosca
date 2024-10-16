#include "CtrlButtonRun.h"

/**
 * Constructor
*/
CtrlButtonRun::CtrlButtonRun() {}

/**
 * Initialize the button controller
 * 
 * @param pin The pin number of the button
*/
void CtrlButtonRun::init(int _pin_button) {
  #ifdef ACTIVE_BUTTON_RUN_SERIAL_SETUP
    Serial.printf("CtrlButtonRun init define pin button %i\n", pin_button);
  #endif
  pin_button = _pin_button;
  pinMode(pin_button, INPUT_PULLUP);

  mutex_button_run = xSemaphoreCreateMutex();  // crete a mutex object

  createParallelFunctionReadButtonRun();
}

void CtrlButtonRun::createParallelFunctionReadButtonRun() {
  #ifdef ACTIVE_BUTTON_RUN_SERIAL_SETUP
    Serial.printf("Setup parallel task: readButtonRun\n");
  #endif
  xTaskCreatePinnedToCore (
    this->readButtonRun,   // Function to implement the task
    "readButtonRun", // Name of the task
    2000,            // Stack size in words
    this,            // Task input parameter
    1,              // Priority of the task
    NULL,            // Task handle.
    0                // Core where the task should run
  );
}

void CtrlButtonRun::readButtonRun(void *pvParameters) {
  CtrlButtonRun *_button_run = (CtrlButtonRun *) pvParameters;  

  uint8_t _pin_button = _button_run->getPinButton();

  uint8_t _robot_status;
  long _countdown_us;
  u_int16_t _delay = BUTTON_RUN_DELAY_WAITING;

  u_int16_t current_button_state_01 = 0;
  u_int16_t current_button_state_02 = 0;
  u_int16_t current_button_state_03 = 0;

  #ifdef ACTIVE_BUTTON_RUN_SERIAL_SETUP
    Serial.printf("Loop readButtonRun pin %i\n", _pin_button); 
  #endif
  while(1) {
    // read the state of the switch/button:
    delay(_delay);
    current_button_state_01 = digitalRead(_pin_button);
    delay(_delay);
    current_button_state_02 = digitalRead(_pin_button);
    delay(_delay);
    current_button_state_03 = digitalRead(_pin_button);
    // delay(10);
    
    if (   current_button_state_01 == LOW 
        && current_button_state_02 == LOW 
        && current_button_state_03 == LOW 
        && _robot_status != ROBOT_STATUS_PRE_RUN
        && _robot_status != ROBOT_STATUS_POST_RUN) {
      if (_robot_status != ROBOT_STATUS_RUN && _robot_status != ROBOT_STATUS_COUNTDOWN) {
        _robot_status = ROBOT_STATUS_PRE_RUN;
        #ifdef ACTIVE_BUTTON_RUN_SERIAL
          Serial.println("Pre run true");
        #endif
      }
      else {
        _robot_status = ROBOT_STATUS_POST_RUN;
        #ifdef ACTIVE_BUTTON_RUN_SERIAL
          Serial.println("Post run true");
        #endif
      }
      _delay = BUTTON_RUN_DELAY_WAITING;
      _button_run->setRobotStatus(_robot_status, true);
    }

    if (   current_button_state_01 == HIGH 
        && current_button_state_02 == HIGH 
        && current_button_state_03 == HIGH 
        && (_robot_status == ROBOT_STATUS_PRE_RUN || _robot_status == ROBOT_STATUS_POST_RUN) 
        && _robot_status != ROBOT_STATUS_COUNTDOWN) {
      if (_robot_status == ROBOT_STATUS_PRE_RUN) {
        _countdown_us = micros();
        _robot_status = ROBOT_STATUS_COUNTDOWN;
        _delay = BUTTON_RUN_DELAY_COUNTDOWN;
        #ifdef ACTIVE_BUTTON_RUN_SERIAL_SETUP
          Serial.printf("Countdown true, waiting %ius\n", BUTTON_RUN_COUNTDOWN_US);
          // Serial.println("Countdown true");
        #endif
      }
      else if (_robot_status == ROBOT_STATUS_POST_RUN) {
        _robot_status = ROBOT_STATUS_STOP;
        _delay = BUTTON_RUN_DELAY_WAITING;
        #ifdef ACTIVE_BUTTON_RUN_SERIAL
            Serial.println("Run false");
        #endif
      }
      _button_run->setRobotStatus(_robot_status, true);
    }

    if (_robot_status == ROBOT_STATUS_COUNTDOWN) {
      long _end_countdown_us = micros();
      if (_end_countdown_us - _countdown_us > BUTTON_RUN_COUNTDOWN_US) {
        #ifdef ACTIVE_BUTTON_RUN_SERIAL
          // Serial.printf("End countdown %i,%i,%i\n", _countdown_us, _end_countdown_us, _end_countdown_us - _countdown_us);
          Serial.printf(",,,,%i,%i,%i\n", _countdown_us, _end_countdown_us, _end_countdown_us - _countdown_us);
        #endif
        _robot_status = ROBOT_STATUS_RUN;
        _delay = BUTTON_RUN_DELAY_RUN;
        // _delay = BUTTON_RUN_DELAY_WAITING;
        _button_run->setRobotStatus(_robot_status, true);
        #ifdef ACTIVE_BUTTON_RUN_SERIAL
          Serial.println("Run true");
        #endif
      }
      else {
        #ifdef ACTIVE_BUTTON_RUN_SERIAL
          // Serial.printf("Init countdown %i,%i,%i\n", _countdown_us, _end_countdown_us, _end_countdown_us - _countdown_us);
          Serial.printf(",,,,%i,%i,%i\n", _countdown_us, _end_countdown_us, _end_countdown_us - _countdown_us);
        #endif
      }
    }
  }
}

void CtrlButtonRun::setRobotStatus(uint8_t _robot_status, bool _robot_status_changed){
  if (xSemaphoreTake (mutex_button_run, portMAX_DELAY)) {
    robot_status = _robot_status;
    robot_status_changed = _robot_status_changed;
    xSemaphoreGive (mutex_button_run);
  }
}

ButtonRunValuesStruct CtrlButtonRun::getRobotStatus(){
  ButtonRunValuesStruct _button_run_values;
  uint8_t _robot_status;
  bool _value_changed;
  if (xSemaphoreTake (mutex_button_run, portMAX_DELAY)) {
    _robot_status = robot_status;
    _value_changed = robot_status_changed;
    robot_status_changed = false;
    xSemaphoreGive (mutex_button_run);
  }
  _button_run_values = {_robot_status, _value_changed};
  return _button_run_values;
}

uint8_t CtrlButtonRun::getPinButton(){
  return pin_button;
}

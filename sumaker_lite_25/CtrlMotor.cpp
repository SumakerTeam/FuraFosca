#include "CtrlMotor.h"

/**
 * Constructor
*/
CtrlMotor::CtrlMotor() {}

/**
 * Initialize the tracker sensor controller
 * 
*/
void CtrlMotor::init(uint8_t _num_motor, uint8_t _pin_esc, uint8_t _channel_esc) {
  #ifdef ACTIVE_MOTOR_SERIAL_SETUP
    Serial.printf("CtrlMotor init motor %i define pin esc %i channel esc %i\n", _num_motor, _pin_esc, _channel_esc);
  #endif
  num_motor = _num_motor;
  pin_esc = _pin_esc;
  channel_esc = _channel_esc;
  mutex_motor = xSemaphoreCreateMutex();  // crete a mutex object
  pinMode(pin_esc, OUTPUT);            // Setting the signal output pin
  ledcSetup(channel_esc, 50, 16);         // _channel_esc config: freq: 50Hz (20mS) Resolution: 16 bits (0 a 65535)
  ledcAttachPin(pin_esc, _channel_esc); // Attach the signal output pin to a ledc channel, necessary for generating PWM with ledc

  delay(20);
  //some ESCs need a initial stop signal to connect
  PWM_width_us = 1500;
  duty_cycle = map(MOTOR_CENTER_PPM_THOTTLE, MOTOR_BOTTOM_VALUE, MOTOR_TOP_VALUE, MOTOR_DUTY_CICLE_BOTTOM_VALUE, MOTOR_DUTY_CICLE_TOP_VALUE); 
  ledcWrite(channel_esc, duty_cycle);
  createParallelFunctionMoveMotor();
}

void CtrlMotor::createParallelFunctionMoveMotor() {
  #ifdef ACTIVE_motor_SERIAL_SETUP
    Serial.printf("Setup parallel task: moveMotor\n");
  #endif
  xTaskCreatePinnedToCore (
    this->moveMotor,   // Function to implement the task
    "moveMotor" + num_motor, // Name of the task
    2000,            // Stack size in words
    this,            // Task input parameter
    1,              // Priority of the task
    NULL,            // Task handle.
    1                // Core where the task should run
  );
}

void CtrlMotor::moveMotor(void *pvParameters) {
  CtrlMotor *_motor = (CtrlMotor *) pvParameters;  

  uint8_t _num_motor = _motor->getNumMotor();
  uint8_t _channel_esc = _motor->getChannelEsc();
  u_int16_t _duty_cycle_progresive = MOTOR_DUTY_CICLE_MIDDLE_VALUE;
  u_int16_t _duty_cycle_progresive_prev = MOTOR_DUTY_CICLE_MIDDLE_VALUE;
  long _time_progresive = 0;
  int num_iterations_without_set = 0;

  bool _turbo;
  bool _change_speed;
  u_int16_t _duty_cycle;
  u_int16_t _duty_cycle_prev;


  #ifdef ACTIVE_MOTOR_SERIAL_SETUP
    Serial.printf("Loop moveMotor num_motor %i\n", _num_motor); 
  #endif
  while(true) {
    #ifdef ACTIVE_MOTOR_SERIAL
      long iniTimeLoop = micros(); 
    #endif

    _turbo = _motor->getTurbo();    
    _change_speed = _motor->getChangeSpeed();
    _duty_cycle = _motor->getDutyCycle();
    _duty_cycle_prev = _motor->getDutyCyclePrev();

    if (_turbo) {
      #ifdef ACTIVE_MOTOR_SERIAL_STEP
        Serial.printf("1 _duty_cycle: %i\n",_duty_cycle);
      #endif
      _motor->setTurboOff();
      if (_duty_cycle_progresive != _duty_cycle) {
        _duty_cycle_progresive = _duty_cycle;
      }
    }
    else if (_duty_cycle <= MOTOR_DUTY_CICLE_MIDDLE_VALUE) {
      #ifdef ACTIVE_MOTOR_SERIAL_STEP
        Serial.printf("2 _duty_cycle: %i\n",_duty_cycle);
      #endif
      if (_duty_cycle_progresive != _duty_cycle) {
        _duty_cycle_progresive = _duty_cycle;
      }
    }
    else {
      
      if (_duty_cycle_progresive == _duty_cycle) {
        #ifdef ACTIVE_MOTOR_SERIAL_STEP
          Serial.printf("31 _duty_cycle: %i\n",_duty_cycle);
        #endif
        // Nothing to do
      }
      else if (_duty_cycle_progresive > _duty_cycle) {
        #ifdef ACTIVE_MOTOR_SERIAL_STEP
          Serial.printf("32 _duty_cycle: %i\n",_duty_cycle);
        #endif
        _duty_cycle_progresive = _duty_cycle;
      }
      else if (_change_speed) {
        #ifdef ACTIVE_MOTOR_SERIAL_STEP
          Serial.printf("33 _duty_cycle: %i\n",_duty_cycle);
        #endif
        if (_duty_cycle_progresive < MOTOR_DUTY_CICLE_MIN_FORWARD_VALUE) {
          _duty_cycle_progresive = MOTOR_DUTY_CICLE_MIN_FORWARD_VALUE;
        }
        else {
          _duty_cycle_progresive + 5;
        }
        _time_progresive = micros();
      }
      else {
        Serial.printf("34 _duty_cycle: %i\n",_duty_cycle);
        long _time_current = micros() - _time_progresive;
        if (_time_current > 20000){ 
        // if( _time_current >= MOTOR_GRADUAL_US) {
          _duty_cycle_progresive =  _duty_cycle_progresive + ((int)_time_current / (int)MOTOR_GRADUAL_US);
          if (_duty_cycle_progresive >= _duty_cycle) {
            _duty_cycle_progresive = _duty_cycle;
          }
          _time_progresive = micros();
        }
      }
    }

    if (_change_speed) {
        _motor->setChangeSpeedOff();
    }

    if (_duty_cycle_progresive_prev != _duty_cycle_progresive || num_iterations_without_set > 20) {
      ledcWrite(_channel_esc, _duty_cycle_progresive);                      // generating a PWM with the proper width in us
      _duty_cycle_progresive_prev = _duty_cycle_progresive;
      num_iterations_without_set = 0;
    }
    else {
      num_iterations_without_set++;
    }

    #ifdef ACTIVE_MOTOR_SERIAL
      Serial.printf("MOTOR %i Channel: %i duty cycle: %i time: %ius\n", _num_motor, _channel_esc, _duty_cycle_progresive, micros() - iniTimeLoop);
    #endif

    // delay(21);//freq: 50Hz
    delay(1);
  }
}

void CtrlMotor::SendPWMSignal(u_int16_t _value_pwm_us, bool _turbo) {
  #ifdef ACTIVE_MOTOR_SERIAL
    Serial.printf("Motor %i SendPWMSignal _value_pwm_us: %i\n", num_motor, _value_pwm_us);
  #endif
  u_int16_t _value_pwm_us_ajusted;
  if (_value_pwm_us > MOTOR_MIDDLE_VALUE) {
    _value_pwm_us_ajusted = map(_value_pwm_us, MOTOR_MIDDLE_VALUE_SUP, MOTOR_TOP_VALUE, MOTOR_MIN_FORWARD, MOTOR_TOP_VALUE);
  }
  else if (_value_pwm_us < MOTOR_MIDDLE_VALUE) {
    _value_pwm_us_ajusted = map(_value_pwm_us, MOTOR_BOTTOM_VALUE, MOTOR_MIDDLE_VALUE_INF, MOTOR_BOTTOM_VALUE, MOTOR_MIN_BACKWARD);
  }
  else {
    _value_pwm_us_ajusted = MOTOR_CENTER_PPM_THOTTLE;
  }

  if (xSemaphoreTake (mutex_motor, portMAX_DELAY)) {
    turbo = _turbo;
    u_int16_t _duty_cycle_prev = duty_cycle;
    duty_cycle = map(_value_pwm_us_ajusted, MOTOR_BOTTOM_VALUE, MOTOR_TOP_VALUE, MOTOR_DUTY_CICLE_BOTTOM_VALUE, MOTOR_DUTY_CICLE_TOP_VALUE);  // Dutycycle to generate a pwm signal with that width in microseconds(us)
    if (duty_cycle != duty_cycle_prev) {
      change_speed = true;
      duty_cycle_prev = _duty_cycle_prev;
    }
    xSemaphoreGive (mutex_motor);
  }
  
  #ifdef ACTIVE_MOTOR_SERIAL
    Serial.printf("Motor %i Channel: %i PWM width: %i(%i)us Speed: %i%% - Direction: %s duty_cicle %i\n", num_motor, channel_esc, _value_pwm_us, _value_pwm_us_ajusted, map(_value_pwm_us,1000,2000,-100,100), (_value_pwm_us > MOTOR_STOP ? "Foward" : "Reverse"), duty_cycle);
  #endif

}

uint8_t CtrlMotor::getNumMotor(){
  return num_motor;
}

uint8_t CtrlMotor::getChannelEsc(){
  return channel_esc;
}

u_int16_t CtrlMotor::getDutyCycle(){
  u_int16_t _duty_cycle;
  if (xSemaphoreTake (mutex_motor, portMAX_DELAY)) {
    _duty_cycle = duty_cycle;
    xSemaphoreGive (mutex_motor);
  }
  return _duty_cycle;
}

u_int16_t CtrlMotor::getDutyCyclePrev(){
  u_int16_t _duty_cycle_prev;
  if (xSemaphoreTake (mutex_motor, portMAX_DELAY)) {
    _duty_cycle_prev = duty_cycle_prev;
    xSemaphoreGive (mutex_motor);
  }
  return _duty_cycle_prev;
}

bool CtrlMotor::getTurbo() {
  bool _turbo;
  if (xSemaphoreTake (mutex_motor, portMAX_DELAY)) {
    _turbo = turbo;
    xSemaphoreGive (mutex_motor);
  }
  return _turbo;
}

void CtrlMotor::setTurboOn() {
  if (xSemaphoreTake (mutex_motor, portMAX_DELAY)) {
    turbo = true;
    xSemaphoreGive (mutex_motor);
  }
}

void CtrlMotor::setTurboOff() {
  if (xSemaphoreTake (mutex_motor, portMAX_DELAY)) {
    turbo = false;
    xSemaphoreGive (mutex_motor);
  }
}

bool CtrlMotor::getChangeSpeed() {
  bool _change_speed;
  if (xSemaphoreTake (mutex_motor, portMAX_DELAY)) {
    _change_speed = change_speed;
    xSemaphoreGive (mutex_motor);
  }
  return _change_speed;
}

void CtrlMotor::setChangeSpeedOn() {
  if (xSemaphoreTake (mutex_motor, portMAX_DELAY)) {
    change_speed = true;
    xSemaphoreGive (mutex_motor);
  }
}

void CtrlMotor::setChangeSpeedOff() {
  if (xSemaphoreTake (mutex_motor, portMAX_DELAY)) {
    change_speed = false;
    xSemaphoreGive (mutex_motor);
  }
}
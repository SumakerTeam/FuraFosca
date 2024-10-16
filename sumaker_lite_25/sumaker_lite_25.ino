
#include <Wire.h>
#include <VL53L0X.h>


#include "Constants.h"
#include "CtrlButtonRun.h"
#include "CtrlLed.h"
#include "CtrlLaser.h"
#include "CtrlTracker.h"
#include "CtrlMotor.h"


void setup()
{
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  Serial.println("Configured sumaker_lite_06.ino");
}

void f01_initLeds(CtrlLed &led_internal, CtrlLed &led_red, CtrlLed &led_yellow, CtrlLed &led_green);

void f11_setRobotStatus(ButtonRunValuesStruct &_robot_status, 
                     CtrlLed &led_internal, 
                     CtrlLed &led_red, 
                     CtrlLed &led_yellow, 
                     CtrlLed &led_green, 
                     u_int16_t &_value_pwm_us_01, 
                     CtrlMotor &motor_01, 
                     u_int16_t &_value_pwm_us_02, 
                     CtrlMotor &motor_02);

void f12_setRobotStatusTrackerSensor(CtrlTracker &tracker_sensor_01, ButtonRunValuesStruct &_robot_status, CtrlTracker &tracker_sensor_02);

void f13_setRobotStatusLaserSensor(CtrlLaser &laser_sensor_01, ButtonRunValuesStruct &_robot_status, CtrlLaser &laser_sensor_02, CtrlLaser &laser_sensor_03, CtrlLaser &laser_sensor_04);

void f04_initLasers(CtrlLaser &laser_sensor_01, CtrlLaser &laser_sensor_02, CtrlLaser &laser_sensor_03, CtrlLaser &laser_sensor_04);

void f05_initButtonRun(CtrlButtonRun &button_run);

void f10_getRobotStatus(ButtonRunValuesStruct &_robot_status, CtrlButtonRun &button_run, bool _critical_stop);

void f02_initMotors(CtrlMotor &motor_01, CtrlMotor &motor_02);

void f03_initTrackers(CtrlTracker &tracker_sensor_01, CtrlTracker &tracker_sensor_02);

void loop()
{
  CtrlMotor motor_01;
  CtrlMotor motor_02;
  CtrlLed led_internal;
  CtrlLed led_red;
  CtrlLed led_yellow;
  CtrlLed led_green;
  CtrlButtonRun button_run;             // Button to start/stop the robot
  CtrlLaser laser_sensor_01;
  CtrlLaser laser_sensor_02;
  CtrlLaser laser_sensor_03;
  CtrlLaser laser_sensor_04;
  CtrlTracker tracker_sensor_01;
  CtrlTracker tracker_sensor_02;

  ButtonRunValuesStruct _robot_status;

  TrackerSensorValuesStruct _tracker_sensor_values_01 = {false, true};
  TrackerSensorValuesStruct _tracker_sensor_values_02 = {false, true};
  bool _tracker_sensor_on_01 = false;
  bool _tracker_sensor_on_02 = false;
  bool _black_line_detected = false;
  long _time_black_line = 0;


  LaserSensorValuesStruct _laser_sensor_values_01 = {1500, true};
  LaserSensorValuesStruct _laser_sensor_values_02 = {1500, true};
  LaserSensorValuesStruct _laser_sensor_values_03 = {1500, true};
  LaserSensorValuesStruct _laser_sensor_values_04 = {1500, true};

  long _ini_time_loop = 0;
  int void_loop = 0;
  
  long _ini_time_tracker_sensors = 0;
  long _ini_time_laser_sensors = 0;
  bool _critical_stop = false;

  u_int16_t _value_pwm_us_01 = MOTOR_STOP;
  u_int16_t _value_pwm_us_02 = MOTOR_STOP;

  bool _turbo = false;
  bool _stop_and_go = false;

  bool _start_fight = true;
  long _ini_time_fight = 0;

  bool _time_traker = 0;

  f01_initLeds(led_internal, led_red, led_yellow, led_green);
  led_red.activeLed();
  f02_initMotors(motor_01, motor_02);
  delay(3000);
  led_red.deactiveLed();
  led_yellow.activeLed();
  f03_initTrackers(tracker_sensor_01, tracker_sensor_02);
  f04_initLasers(laser_sensor_01, laser_sensor_02, laser_sensor_03, laser_sensor_04);
  f05_initButtonRun(button_run);
  led_yellow.deactiveLed();
  led_green.blickSlowLed();

  bool is_initial_direction_righ = false;
  #ifdef INITIAL_DIRECTION_RIGHT
    is_initial_direction_righ = true;
  #endif
  #ifdef INITIAL_DIRECTION_LEFT
    is_initial_direction_righ = false;
  #endif
  #ifdef INITIAL_DIRECTION_RANDOM
    is_initial_direction_righ = random(10) > 5;
  #endif
  #ifdef ACTIVE_MAIN_SERIAL_SETUP
    Serial.printf("initial_direction: %s\n", is_initial_direction_righ ? "right" : "left");
  #endif


  #ifdef ACTIVE_MAIN_SERIAL
    Serial.printf("Loop main\n");
  #endif
  while(true) {
    f10_getRobotStatus(_robot_status, button_run, _critical_stop);
    if (_robot_status.value_changed) {
      f11_setRobotStatus(_robot_status, led_internal, led_red, led_yellow, led_green, _value_pwm_us_01, motor_01, _value_pwm_us_02, motor_02);
      f12_setRobotStatusTrackerSensor(tracker_sensor_01, _robot_status, tracker_sensor_02);
      f13_setRobotStatusLaserSensor(laser_sensor_01, _robot_status, laser_sensor_02, laser_sensor_03, laser_sensor_04);
    }

    if (_robot_status.status == ROBOT_STATUS_RUN) {
      if (!_start_fight) {
        _ini_time_fight = millis();
        _start_fight = true;
      }

      ////////////////////////////////////////
      // TRAKER SENSORS
      ////////////////////////////////////////
      #ifdef ACTIVE_TRACKER_SENSOR_01
        _tracker_sensor_values_01 = tracker_sensor_01.getTrackerSensorValues();
        // if(_tracker_sensor_values_01.white_detected) {
        //   led_green.activeLed();
        // }
        // else {
        //   led_green.deactiveLed();
        // }
        #ifdef ACTIVE_MAIN_SERIAL
          Serial.printf("_tracker_sensor_values_01 white_detected: %d value_changed: %d\n", _tracker_sensor_values_01.white_detected, _tracker_sensor_values_01.value_changed);
        #endif
      #endif
      #ifdef ACTIVE_TRACKER_SENSOR_02
        _tracker_sensor_values_02 = tracker_sensor_02.getTrackerSensorValues();
        // if(_tracker_sensor_values_02.white_detected) {
        //   led_red.activeLed();
        // }
        // else {
        //   led_red.deactiveLed();
        // }
        #ifdef ACTIVE_MAIN_SERIAL
          Serial.printf("_tracker_sensor_values_02 white_detected: %d value_changed: %d\n", _tracker_sensor_values_02.white_detected, _tracker_sensor_values_02.value_changed);
        #endif
      #endif

      // if (   _tracker_sensor_values_01.value_changed 
      //     || _tracker_sensor_values_02.value_changed) {
      //   _ini_time_tracker_sensors = millis();
      // }
      // else {
      //   if (_ini_time_tracker_sensors > 0 && millis() - _ini_time_tracker_sensors > 20) {
      //     #ifdef ACTIVE_MOTOR_01
      //       motor_01.SendPWMSignal(MOTOR_STOP, true);
      //     #endif
      //     #ifdef ACTIVE_MOTOR_02
      //       motor_02.SendPWMSignal(MOTOR_STOP, true);
      //     #endif
      //     _critical_stop = true;
      //     led_red.blickFastLed();
      //     led_green.blickFastLed();
      //     delay(10000);
      //   }
      // }

      _turbo = false;
      if (   _tracker_sensor_values_01.white_detected
          || _tracker_sensor_values_02.white_detected
          || _tracker_sensor_on_01
          || _tracker_sensor_on_02) {
        
        if (_tracker_sensor_on_01) {
          if (_tracker_sensor_values_01.white_detected) {
            if (_black_line_detected) {
              _black_line_detected = false;  
            }
            if (_tracker_sensor_values_02.white_detected) {
              if (_value_pwm_us_01 != MOTOR_SPEED_TRACKER_WHEEL_TURBO_FAST || _value_pwm_us_02 != MOTOR_SPEED_TRACKER_WHEEL_TURBO_SLOW) {
                // _turbo = true;
                _stop_and_go = true;
                _value_pwm_us_01 = MOTOR_SPEED_TRACKER_WHEEL_TURBO_FAST;
                _value_pwm_us_02 = MOTOR_SPEED_TRACKER_WHEEL_TURBO_SLOW;
                _time_traker = TRACKER_BORDER_TIME_ROUND_LONG;
              }
            }
            else {
              if (_value_pwm_us_01 != MOTOR_SPEED_TRACKER_WHEEL_FAST || _value_pwm_us_02 != MOTOR_SPEED_TRACKER_WHEEL_SLOW){
                _turbo = true;
                _stop_and_go = true;
                // _value_pwm_us_01 = MOTOR_SPEED_TRACKER_WHEEL_FAST;
                _value_pwm_us_02 = MOTOR_SPEED_TRACKER_WHEEL_SLOW;
              }
            }
          }
          else {
            if (!_black_line_detected) {
              _time_black_line = millis();
              _black_line_detected = true;
            }
            else {
              if (millis() - _time_black_line > _time_traker) {
                _tracker_sensor_on_01 = false;
                _black_line_detected = false;
                // if (_value_pwm_us_01 != MOTOR_SPEED_SEARCH_WHEEL_FAST || _value_pwm_us_02 != MOTOR_SPEED_SEARCH_WHEEL_FAST){
                //   _stop_and_go = true;
                //   // _value_pwm_us_01 = MOTOR_SPEED_SEARCH_WHEEL_FAST;
                //   _value_pwm_us_02 = MOTOR_SPEED_SEARCH_WHEEL_FAST;
                // }
              }
            }
          }
        }
        else if (_tracker_sensor_on_02) {
          if (_tracker_sensor_values_02.white_detected) {
            if (_black_line_detected) {
              _black_line_detected = false;  
            }
            if (_tracker_sensor_values_01.white_detected) {
              if (_value_pwm_us_01 != MOTOR_SPEED_TRACKER_WHEEL_TURBO_SLOW || _value_pwm_us_02 != MOTOR_SPEED_TRACKER_WHEEL_TURBO_FAST) {
                // _turbo = true;
                _stop_and_go = true;
                _value_pwm_us_01 = MOTOR_SPEED_TRACKER_WHEEL_TURBO_SLOW;
                _value_pwm_us_02 = MOTOR_SPEED_TRACKER_WHEEL_TURBO_FAST;
                _time_traker = TRACKER_BORDER_TIME_ROUND_LONG;
              }
            }
            else {
              if (_value_pwm_us_01 != MOTOR_SPEED_TRACKER_WHEEL_SLOW || _value_pwm_us_02 != MOTOR_SPEED_TRACKER_WHEEL_FAST){
                _stop_and_go = true;
                _turbo = true;
                _value_pwm_us_01 = MOTOR_SPEED_TRACKER_WHEEL_SLOW;
                // _value_pwm_us_02 = MOTOR_SPEED_TRACKER_WHEEL_FAST;
              }
            }
          }
          else {
            if (!_black_line_detected) {
              _time_black_line = millis();
              _black_line_detected = true;
            }
            else {
              if (millis() - _time_black_line > _time_traker) {
                _tracker_sensor_on_02 = false;
                _black_line_detected = false;
                // if (_value_pwm_us_01 != MOTOR_SPEED_SEARCH_WHEEL_FAST || _value_pwm_us_02 != MOTOR_SPEED_SEARCH_WHEEL_FAST){
                //   _stop_and_go = true;
                //   // _value_pwm_us_01 = MOTOR_SPEED_SEARCH_WHEEL_FAST;
                //   // _value_pwm_us_02 = MOTOR_SPEED_SEARCH_WHEEL_FAST;
                // }
              }
            }
          }
        }
//////////////////////
        else {
          if (_tracker_sensor_values_01.white_detected) {
            _tracker_sensor_on_01 = true;
            _black_line_detected = false;
            _time_black_line = millis();
            // _value_pwm_us_01 = MOTOR_SPEED_TRACKER_WHEEL_FAST;
            if (_value_pwm_us_01 - MOTOR_SPEED_TRACKER_WHEEL_FAST_SLOWDOWN > MOTOR_MIN_FORWARD) {
              _value_pwm_us_01 = _value_pwm_us_01 - MOTOR_SPEED_TRACKER_WHEEL_FAST_SLOWDOWN;
            }
            _value_pwm_us_02 = MOTOR_SPEED_TRACKER_WHEEL_SLOW;
            _time_traker = TRACKER_BORDER_TIME_ROUND;
            #ifdef ACTIVE_MAIN_SERIAL_LOGIC
              Serial.println("Tracker sensors 01 active, 02 not active");
            #endif
          }
          else if (_tracker_sensor_values_02.white_detected) {
            _tracker_sensor_on_02 = true;
            _black_line_detected = false;
            _time_black_line = millis();
            _value_pwm_us_01 = MOTOR_SPEED_TRACKER_WHEEL_SLOW;
            // _value_pwm_us_02 = MOTOR_SPEED_TRACKER_WHEEL_FAST;
            if (_value_pwm_us_02 - MOTOR_SPEED_TRACKER_WHEEL_FAST_SLOWDOWN > MOTOR_MIN_FORWARD) {
              _value_pwm_us_02 = _value_pwm_us_02 - MOTOR_SPEED_TRACKER_WHEEL_FAST_SLOWDOWN;
            }
            _time_traker = TRACKER_BORDER_TIME_ROUND;
            #ifdef ACTIVE_MAIN_SERIAL_LOGIC
              Serial.println("Tracker sensors 02 active, 01 not active");
            #endif
          }
        }

      }

      if (false && !_tracker_sensor_on_01 && !_tracker_sensor_on_02 ) {
      ////////////////////////////////////////
      // LASER SENSORS
      ////////////////////////////////////////
        #ifdef ACTIVE_LASER_SENSOR_01
          _laser_sensor_values_01 = laser_sensor_01.getLaserSensorValues();
          #ifdef ACTIVE_MAIN_SERIAL
            Serial.printf("_laser_sensor_values_01 distance: %i value_changed: %d\n", _laser_sensor_values_01.distance, _laser_sensor_values_01.value_changed);
          #endif
        #endif
        #ifdef ACTIVE_LASER_SENSOR_02
          _laser_sensor_values_02 = laser_sensor_02.getLaserSensorValues();
          #ifdef ACTIVE_MAIN_SERIAL
            Serial.printf("_laser_sensor_values_02 distance: %i value_changed: %d\n", _laser_sensor_values_02.distance, _laser_sensor_values_02.value_changed);
          #endif
        #endif
        #ifdef ACTIVE_LASER_SENSOR_03
          _laser_sensor_values_03 = laser_sensor_03.getLaserSensorValues();
          #ifdef ACTIVE_MAIN_SERIAL
            Serial.printf("_laser_sensor_values_03 distance: %i value_changed: %d\n", _laser_sensor_values_03.distance, _laser_sensor_values_03.value_changed);
          #endif
        #endif
        #ifdef ACTIVE_LASER_SENSOR_04
          _laser_sensor_values_04 = laser_sensor_04.getLaserSensorValues();
          #ifdef ACTIVE_MAIN_SERIAL
            Serial.printf("_laser_sensor_values_04 distance: %i value_changed: %d\n", _laser_sensor_values_04.distance, _laser_sensor_values_04.value_changed);
          #endif
        #endif

        bool _laser_sensor_in_distance_01 = _laser_sensor_values_01.distance < LASER_SENSOR_MAX_DISTANCE;
        bool _laser_sensor_in_distance_02 = _laser_sensor_values_02.distance < LASER_SENSOR_MAX_DISTANCE;
        bool _laser_sensor_in_distance_03 = _laser_sensor_values_03.distance < LASER_SENSOR_MAX_DISTANCE;
        bool _laser_sensor_in_distance_04 = _laser_sensor_values_04.distance < LASER_SENSOR_MAX_DISTANCE;

        if(_laser_sensor_in_distance_02) {
          led_green.activeLed();
        }
        else {
          led_green.deactiveLed();
        }
        if(_laser_sensor_in_distance_03) {
          led_red.activeLed();
        }
        else {
          led_red.deactiveLed();
        }

        if (
               _laser_sensor_values_01.value_changed 
            || _laser_sensor_values_02.value_changed 
            || _laser_sensor_values_03.value_changed 
            || _laser_sensor_values_04.value_changed) {
          _ini_time_laser_sensors = millis();
        }
        else {
          if (_ini_time_laser_sensors > 0 && millis() - _ini_time_laser_sensors > 90) {
            #ifdef ACTIVE_MOTOR_01
              motor_01.SendPWMSignal(MOTOR_STOP, true);
            #endif
            #ifdef ACTIVE_MOTOR_02
              motor_02.SendPWMSignal(MOTOR_STOP, true);
            #endif
            _critical_stop = true;
            led_red.blickFastLed();
            led_yellow.blickFastLed();
            led_green.blickFastLed();
            delay(10000);
          }
        }

        if (_laser_sensor_in_distance_02 && _laser_sensor_in_distance_03) {
          _value_pwm_us_01 = MOTOR_SPEED_ATTACK;
          _value_pwm_us_02 = MOTOR_SPEED_ATTACK;
          #ifdef ACTIVE_MAIN_SERIAL_LOGIC
            Serial.println("Laser sensors 02 03 detect object");
          #endif
        } 
        else if (_laser_sensor_in_distance_02 && !_laser_sensor_in_distance_03) {
          _value_pwm_us_01 = MOTOR_SPEED_LASER_CENTER_WHEEL_SLOW;
          _value_pwm_us_02 = MOTOR_SPEED_LASER_CENTER_WHEEL_FAST;
          #ifdef ACTIVE_MAIN_SERIAL_LOGIC
            Serial.println("Laser sensors 02 detect object");
          #endif
        } 
        else if (!_laser_sensor_in_distance_02 && _laser_sensor_in_distance_03) {
          _value_pwm_us_01 = MOTOR_SPEED_LASER_CENTER_WHEEL_FAST;
          _value_pwm_us_02 = MOTOR_SPEED_LASER_CENTER_WHEEL_SLOW;
          #ifdef ACTIVE_MAIN_SERIAL_LOGIC
            Serial.println("Laser sensors 03 detect object");
          #endif
        }
        else if (!_laser_sensor_in_distance_02 && !_laser_sensor_in_distance_03 && _laser_sensor_in_distance_01 && !_laser_sensor_in_distance_04) {
          _value_pwm_us_01 = MOTOR_SPEED_LASER_LATERAL_WHEEL_SLOW;
          _value_pwm_us_02 = MOTOR_SPEED_LASER_LATERAL_WHEEL_FAST;
          #ifdef ACTIVE_MAIN_SERIAL_LOGIC
            Serial.println("Laser sensors 01 detect object");
          #endif
        } 
        else if (!_laser_sensor_in_distance_02 && !_laser_sensor_in_distance_03 && !_laser_sensor_in_distance_01 && _laser_sensor_in_distance_04) {
          _value_pwm_us_01 = MOTOR_SPEED_LASER_LATERAL_WHEEL_FAST;
          _value_pwm_us_02 = MOTOR_SPEED_LASER_LATERAL_WHEEL_SLOW;
          #ifdef ACTIVE_MAIN_SERIAL_LOGIC
            Serial.println("Laser sensors 04 detect object");
          #endif
        }
        else {
          _value_pwm_us_01 = MOTOR_SPEED_SEARCH_WHEEL_FAST;
          _value_pwm_us_02 = MOTOR_SPEED_SEARCH_WHEEL_SLOW;
          #ifdef ACTIVE_MAIN_SERIAL_LOGIC
            Serial.println("No laser sensors detected");
          #endif
        }
        led_yellow.activeLed();
      }
      else {
        led_yellow.deactiveLed();
        if(_tracker_sensor_values_01.white_detected) {
          led_green.activeLed();
        }
        else {
          led_green.deactiveLed();
        }
        if(_tracker_sensor_values_02.white_detected) {
          led_red.activeLed();
        }
        else {
          led_red.deactiveLed();
        }
        _ini_time_laser_sensors = millis();
      }

      if (!_tracker_sensor_on_01 && !_tracker_sensor_on_02 ) {
        long _time_fight = millis() - _ini_time_fight;

        if (is_initial_direction_righ) {
          if (_time_fight < TIME_STEP_1) {
            _value_pwm_us_01 = SPEED_STEP_1_SLOW;
            _value_pwm_us_02 = SPEED_STEP_1_FAST;
          }
          else if (_time_fight < TIME_STEP_2) {
            _value_pwm_us_01 = SPEED_STEP_2_SLOW;
            _value_pwm_us_02 = SPEED_STEP_2_FAST;
          }
          else if (_time_fight < TIME_STEP_3) {
            _value_pwm_us_01 = SPEED_STEP_3_SLOW;
            _value_pwm_us_02 = SPEED_STEP_3_FAST;
          }
          else if (_time_fight < TIME_STEP_TOP) {
            _value_pwm_us_01 = SPEED_STEP_4_SLOW;
            _value_pwm_us_02 = SPEED_STEP_4_FAST;
          }
          else if (_time_fight > TIME_STEP_TOP) {
            _value_pwm_us_01 = MOTOR_STOP;
            _value_pwm_us_02 = MOTOR_STOP;
          }
        }
        else {
          if (_time_fight < TIME_STEP_1) {
            _value_pwm_us_01 = SPEED_STEP_1_FAST;
            _value_pwm_us_02 = SPEED_STEP_1_SLOW;
          }
          else if (_time_fight < TIME_STEP_2) {
            _value_pwm_us_01 = SPEED_STEP_2_FAST;
            _value_pwm_us_02 = SPEED_STEP_2_SLOW;
          }
          else if (_time_fight < TIME_STEP_3) {
            _value_pwm_us_01 = SPEED_STEP_3_FAST;
            _value_pwm_us_02 = SPEED_STEP_3_SLOW;
          }
          else if (_time_fight < TIME_STEP_TOP) {
            _value_pwm_us_01 = SPEED_STEP_4_FAST;
            _value_pwm_us_02 = SPEED_STEP_4_SLOW;
          }
          else if (_time_fight > TIME_STEP_TOP) {
            _value_pwm_us_01 = MOTOR_STOP;
            _value_pwm_us_02 = MOTOR_STOP;
          }
        }
      }

        #ifdef ACTIVE_MAIN_SERIAL_LOGIC
          Serial.printf("Motor 01 _value_pwm_us_01: %i\n", _value_pwm_us_01);
        #endif
      #ifdef ACTIVE_MOTOR_01
        motor_01.SendPWMSignal(_value_pwm_us_01, _turbo);
      #endif
        #ifdef ACTIVE_MAIN_SERIAL_LOGIC
          Serial.printf("Motor 02 _value_pwm_us_02: %i\n", _value_pwm_us_02);
        #endif
      #ifdef ACTIVE_MOTOR_02
        motor_02.SendPWMSignal(_value_pwm_us_02, _turbo);
      #endif

      #ifdef ACTIVE_MAIN_PLOTTER
        Serial.printf("Desfase loop: %d\n", micros() - _ini_time_loop);
        _ini_time_loop = micros();
            // Serial.printf("%i,,\n", micros() - _ini_time_loop);
            // Serial.printf(",%i,%i\n", 200, 0);
        if (   _laser_sensor_values_01.value_changed 
            || _laser_sensor_values_02.value_changed 
            || _laser_sensor_values_03.value_changed 
            || _laser_sensor_values_04.value_changed 
            || _tracker_sensor_values_01.value_changed
            || _tracker_sensor_values_02.value_changed) {
          Serial.printf("%i,%i,%i,%i,%s%d,%s%d\n"
                        , _laser_sensor_values_01.distance
                        , _laser_sensor_values_02.distance
                        , _laser_sensor_values_03.distance
                        , _laser_sensor_values_04.distance
                        , _tracker_sensor_values_01.value_changed ? "*" : ""
                        , _tracker_sensor_values_01.white_detected
                        , _tracker_sensor_values_02.value_changed ? "*" : ""
                        , _tracker_sensor_values_02.white_detected);
        //   Serial.printf("%i\n", _laser_sensor_distance_03);
          // Serial.printf("%i,,\n", micros() - _ini_time_loop);
          // _ini_time_loop = micros();
        }
        else {
          // void_loop++;
          // Serial.printf("%i\n", void_loop);
        }
        
      #endif
      delay(1);
    }
    else {
      _start_fight = false;
      #ifdef ACTIVE_MOTOR_01
        #ifdef ACTIVE_MAIN_SERIAL_LOGIC
          Serial.printf("Motor 01 _value_pwm_us_01: %i\n", MOTOR_STOP);
        #endif
        motor_01.SendPWMSignal(MOTOR_STOP, true);
      #endif
      #ifdef ACTIVE_MOTOR_02
        #ifdef ACTIVE_MAIN_SERIAL_LOGIC
          Serial.printf("Motor 02 _value_pwm_us_02: %i\n", MOTOR_STOP);
        #endif
        motor_02.SendPWMSignal(MOTOR_STOP, true);
      #endif
      delay(10);
    }
  }
}

void f01_initLeds(CtrlLed &led_internal, CtrlLed &led_red, CtrlLed &led_yellow, CtrlLed &led_green)
{
  led_internal.init(LED_PIN_INTERNAL);
  led_red.init(LED_PIN_RED);
  led_yellow.init(LED_PIN_YELLOW);
  led_green.init(LED_PIN_GREEN);
}

void f02_initMotors(CtrlMotor &motor_01, CtrlMotor &motor_02)
{
#ifdef ACTIVE_MOTOR_01
  motor_01.init(1, MOTOR_OUTPUT_ESC_01, MOTOR_CHANNEL_ESC_01);
#endif
#ifdef ACTIVE_MOTOR_02
  motor_02.init(2, MOTOR_OUTPUT_ESC_02, MOTOR_CHANNEL_ESC_02);
#endif
}

void f03_initTrackers(CtrlTracker &tracker_sensor_01, CtrlTracker &tracker_sensor_02)
{
#ifdef ACTIVE_TRACKER_SENSOR_01
  tracker_sensor_01.init(TRACKER_PIN_SENSOR_01, 1);
#endif
#ifdef ACTIVE_TRACKER_SENSOR_02
  tracker_sensor_02.init(TRACKER_PIN_SENSOR_02, 2);
#endif
}

void f04_initLasers(CtrlLaser &laser_sensor_01, CtrlLaser &laser_sensor_02, CtrlLaser &laser_sensor_03, CtrlLaser &laser_sensor_04)
{
  laser_sensor_01.resetLaserSensor(LASER_SENSOR_PIN_XSHUT_01, 1);
  laser_sensor_02.resetLaserSensor(LASER_SENSOR_PIN_XSHUT_02, 2);
  laser_sensor_03.resetLaserSensor(LASER_SENSOR_PIN_XSHUT_03, 3);
  laser_sensor_04.resetLaserSensor(LASER_SENSOR_PIN_XSHUT_04, 4);

#ifdef ACTIVE_LASER_SENSOR_01
  laser_sensor_01.init(LASER_SENSOR_PIN_XSHUT_01, 1);
#endif
#ifdef ACTIVE_LASER_SENSOR_02
  laser_sensor_02.init(LASER_SENSOR_PIN_XSHUT_02, 2);
#endif
#ifdef ACTIVE_LASER_SENSOR_03
  laser_sensor_03.init(LASER_SENSOR_PIN_XSHUT_03, 3);
#endif
#ifdef ACTIVE_LASER_SENSOR_04
  laser_sensor_04.init(LASER_SENSOR_PIN_XSHUT_04, 4);
#endif
}

void f05_initButtonRun(CtrlButtonRun &button_run)
{
  button_run.init(BUTTON_RUN_PIN);
}

void f10_getRobotStatus(ButtonRunValuesStruct &_robot_status, CtrlButtonRun &button_run, bool _critical_stop)
{
  if (_critical_stop) {
    _robot_status.status = ROBOT_STATUS_STOP;
    _robot_status.value_changed = true;
  }
  else {
    _robot_status = button_run.getRobotStatus();
  }
}

void f11_setRobotStatus(ButtonRunValuesStruct &_robot_status, 
                     CtrlLed &led_internal, 
                     CtrlLed &led_red, 
                     CtrlLed &led_yellow, 
                     CtrlLed &led_green, 
                     u_int16_t &_value_pwm_us_01, 
                     CtrlMotor &motor_01, 
                     u_int16_t &_value_pwm_us_02, 
                     CtrlMotor &motor_02)
{
  #ifdef ACTIVE_MAIN_SERIAL
    Serial.printf("_robot_status: %i\n", _robot_status.status);
  #endif
  _value_pwm_us_02 = MOTOR_STOP;
  if (_robot_status.status == ROBOT_STATUS_STOP)
  {
    led_internal.deactiveLed();
    led_red.deactiveLed();
    led_yellow.deactiveLed();
    led_green.deactiveLed();
    _value_pwm_us_01 = MOTOR_STOP;
    #ifdef ACTIVE_MOTOR_01
      motor_01.SendPWMSignal(_value_pwm_us_01, true);
    #endif
    _value_pwm_us_02 = MOTOR_STOP;
    #ifdef ACTIVE_MOTOR_02
      motor_02.SendPWMSignal(_value_pwm_us_02, true);
    #endif
  }
  else if (_robot_status.status == ROBOT_STATUS_RUN)
  {
    led_internal.activeLed();
    led_green.deactiveLed();
  }
  else if (_robot_status.status == ROBOT_STATUS_PRE_RUN)
  {
    led_internal.blickSlowLed();
  }
  else if (_robot_status.status == ROBOT_STATUS_POST_RUN)
  {
    led_internal.deactiveLed();
    led_red.deactiveLed();
    led_yellow.deactiveLed();
    led_green.deactiveLed();
    _value_pwm_us_01 = MOTOR_STOP;
    #ifdef ACTIVE_MOTOR_01
      motor_01.SendPWMSignal(_value_pwm_us_01, true);
    #endif
    _value_pwm_us_02 = MOTOR_STOP;
    #ifdef ACTIVE_MOTOR_02
      motor_02.SendPWMSignal(_value_pwm_us_02, true);
    #endif
  }
  else if (_robot_status.status == ROBOT_STATUS_COUNTDOWN)
  {
    led_internal.blickFastLed();
  }
  else
  {
    led_internal.deactiveLed();
  }
  #ifdef ACTIVE_MAIN_SERIAL
    if (_robot_status.status == ROBOT_STATUS_RUN) {
      Serial.printf("Main loop robot run\n");
    }
    else {
      Serial.printf("Main loop robot stop: %i\n", _robot_status.status);
    }
  #endif
}

void f12_setRobotStatusTrackerSensor(CtrlTracker &tracker_sensor_01, ButtonRunValuesStruct &_robot_status, CtrlTracker &tracker_sensor_02)
{
#ifdef ACTIVE_TRACKER_SENSOR_01
  tracker_sensor_01.setRobotStatus(_robot_status.status);
#endif
#ifdef ACTIVE_TRACKER_SENSOR_02
  tracker_sensor_02.setRobotStatus(_robot_status.status);
#endif
}

void f13_setRobotStatusLaserSensor(CtrlLaser &laser_sensor_01, ButtonRunValuesStruct &_robot_status, CtrlLaser &laser_sensor_02, CtrlLaser &laser_sensor_03, CtrlLaser &laser_sensor_04)
{
#ifdef ACTIVE_LASER_SENSOR_01
  laser_sensor_01.setRobotStatus(_robot_status.status);
#endif
#ifdef ACTIVE_LASER_SENSOR_02
  laser_sensor_02.setRobotStatus(_robot_status.status);
#endif
#ifdef ACTIVE_LASER_SENSOR_03
  laser_sensor_03.setRobotStatus(_robot_status.status);
#endif
#ifdef ACTIVE_LASER_SENSOR_04
  laser_sensor_04.setRobotStatus(_robot_status.status);
#endif
}


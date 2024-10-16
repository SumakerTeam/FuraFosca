#ifndef CtrlTracker_h
#define CtrlTracker_h

#include <Arduino.h>
#include "Constants.h"

typedef struct {
  bool white_detected;
  bool value_changed;
} TrackerSensorValuesStruct;

class CtrlTracker {
  /**
   * Class to control button
   * 
   * The switch can be checked if it is pressed, down or up
  */
  protected:
   
    uint8_t pin_sensor;                         // Bunton pin to read the input
    uint8_t num_sensor;
    SemaphoreHandle_t mutex_tracker_sensor = NULL;  // Create a mutex object
    uint8_t robot_status = ROBOT_STATUS_STOP;
    bool tracker_sensor_white_detected = false;
    bool tracker_sensor_changed = false;

    void virtual createParallelFunctionReadTrackerSensor();
    static void readTrackerSensor(void *pvParameters);
    uint8_t virtual getPinSensor();
    uint8_t virtual getNumSensor();
    static String print2Zeros(uint16_t value);
    void virtual setTrackerSensorValues(bool _tracker_sensor_white_detected, bool _tracker_sensor_changed);

  public:
    CtrlTracker();
    void virtual init(uint8_t _pin_sensor, uint8_t _num_sensor);

    TrackerSensorValuesStruct virtual getTrackerSensorValues();
    uint8_t virtual getRobotStatus();
    void virtual setRobotStatus(uint8_t _robot_status);
  private:

};



#endif
#ifndef CtrlLaser_h
#define CtrlLaser_h

#include <Arduino.h>
#include <VL53L0X.h>
#include "Constants.h"

typedef struct {
  uint16_t distance;
  bool value_changed;
} LaserSensorValuesStruct;

class CtrlLaser {
  /**
   * Class to control button
   * 
   * The switch can be checked if it is pressed, down or up
  */
  protected:
    uint8_t pin_xshut;                         // Bunton pin to read the input
    uint8_t num_sensor;
    VL53L0X sensor;
    bool sensor_status;
    SemaphoreHandle_t mutex_laser_sensor = NULL;  // Create a mutex object
    uint16_t laser_sensor_distance = LASER_SENSOR_MAX_DISTANCE;
    bool laser_sensor_changed = false;
    uint8_t robot_status = ROBOT_STATUS_STOP;

    void virtual createParallelFunctionReadLaserSensor();
    static void readLaserSensor(void *pvParameters);

    uint8_t virtual getPinXshut();
    uint8_t virtual getNumSensor();
    static String print2Zeros(uint16_t value);
    static String print4Zeros(uint16_t value);
    static String print6Zeros(uint16_t value);
    VL53L0X virtual * getSensor();
    bool virtual getSensorStatus();
    void virtual initLaserSensor();
    void virtual setLaserSensorValues(uint16_t _laser_sensor_distance, bool _laser_sensor_changed);

  public:
    CtrlLaser();
    void virtual resetLaserSensor(uint8_t _pin_xshut, uint8_t _num_sensor);
    void virtual init(uint8_t _pin_xshut, uint8_t _num_sensor);


    LaserSensorValuesStruct virtual getLaserSensorValues();
    uint8_t virtual getRobotStatus();
    void virtual setRobotStatus(uint8_t robot_status);

  private:

};



#endif
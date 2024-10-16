#ifndef CtrlButtonRun_h
#define CtrlButtonRun_h

#include <Arduino.h>
#include "Constants.h"

typedef struct {
  uint8_t status;
  bool value_changed;
} ButtonRunValuesStruct;

class CtrlButtonRun {
  /**
   * Class to control button
   * 
   * The switch can be checked if it is pressed, down or up
  */
  protected:   
    uint8_t pin_button;                         // Bunton pin to read the input
    uint8_t robot_status;
    bool robot_status_changed = false;
    SemaphoreHandle_t mutex_button_run = NULL;  // Create a mutex object

    uint8_t virtual getPinButton();
    void virtual createParallelFunctionReadButtonRun();
    static void readButtonRun(void *pvParameters);
    void virtual setRobotStatus(uint8_t _robot_status, bool _robot_status_changed);
  public:
    CtrlButtonRun();
    void virtual init(int _pin_button);
    ButtonRunValuesStruct virtual getRobotStatus();
  private:

};



#endif
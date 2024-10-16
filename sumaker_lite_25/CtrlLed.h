#ifndef CtrlLed_h
#define CtrlLed_h

#include <Arduino.h>
#include "Constants.h"


class CtrlLed {
  /**
   * Class to control button
   * 
   * The switch can be checked if it is pressed, down or up
  */
  protected:   
    uint8_t pin_led;                         // Bunton pin to read the input
    bool led_active = false;
    bool blick_active = false;
    uint8_t blick_speed = LED_BLINK_SLOW;

    bool virtual getBlickActive();
    uint8_t virtual getPinLed();
    void virtual blickLed();
    static void blicking(void *pvParameters);
    uint8_t virtual getBlickSpeed();


  public:
    CtrlLed();
    void virtual init(uint8_t _pin_led);
    void virtual activeLed();
    void virtual deactiveLed();
    bool virtual getStatusLed();
    void virtual blickFastLed();
    void virtual blickSlowLed();
  private:

};



#endif
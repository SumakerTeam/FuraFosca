#ifndef CtrlMotor_h
#define CtrlMotor_h

#include <Arduino.h>
#include "Constants.h"


class CtrlMotor {
  /**
   * Class to control button
   * 
   * The switch can be checked if it is pressed, down or up
  */
  protected:   
    uint8_t num_motor;
    uint8_t pin_esc;
    uint8_t channel_esc;
    u_int16_t PWM_width_us = 0; //PWM high width in Microseconds (us) (amount of time the signal is high)
    u_int16_t duty_cycle = 0;    //Duty Cycle of the pwm signal generated
    u_int16_t duty_cycle_prev = 0;    //Duty Cycle of the pwm signal generated previous
    bool turbo = false;
    bool change_speed = false;
    SemaphoreHandle_t mutex_motor = NULL;  // Create a mutex object

    void virtual createParallelFunctionMoveMotor();
    static void moveMotor(void *pvParameters);
    uint8_t virtual getNumMotor();
    uint8_t virtual getChannelEsc();
    u_int16_t virtual getDutyCycle();
    u_int16_t virtual getDutyCyclePrev();
    bool virtual getTurbo();
    void virtual setTurboOn();
    void virtual setTurboOff();
    bool virtual getChangeSpeed();
    void virtual setChangeSpeedOn();
    void virtual setChangeSpeedOff();

  public:
    CtrlMotor();
    void virtual init(uint8_t _num_motor, uint8_t _pin_esc, uint8_t _channel_esc);
    void virtual SendPWMSignal(u_int16_t _value_pwm_us, bool _turbo);

  private:

};



#endif
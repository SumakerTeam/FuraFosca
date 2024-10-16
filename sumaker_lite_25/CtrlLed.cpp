#include "CtrlLed.h"

/**
 * Constructor
*/
CtrlLed::CtrlLed() {}

/**
 * Initialize the tracker sensor controller
 * 
*/
void CtrlLed::init(uint8_t _pin_led) {
  #ifdef ACTIVE_LED_SERIAL_SETUP
    Serial.printf("CtrlLed init define pin led %i\n", _pin_led);
  #endif
  pin_led = _pin_led;
  led_active = false;
  blick_active = false;
  pinMode(_pin_led, OUTPUT);
}

uint8_t CtrlLed::getPinLed(){
  return pin_led;
}

bool CtrlLed::getBlickActive(){
  return blick_active;
}

uint8_t CtrlLed::getBlickSpeed(){
  return blick_speed;
}

void CtrlLed::activeLed(){
  if (!led_active || blick_active) {
    led_active = true;
    blick_active = false;
    digitalWrite(pin_led, HIGH);
  }
}
void CtrlLed::deactiveLed(){
  if (led_active || blick_active) {
    led_active = false;
    blick_active = false;
    digitalWrite(pin_led, LOW);
  }
}

bool CtrlLed::getStatusLed(){
  return led_active;
}

void CtrlLed::blickFastLed() {
  #ifdef ACTIVE_LED_SERIAL
    Serial.println("Blick fast led");
  #endif
  blick_speed = LED_BLINK_FAST;
  blickLed();
}

void CtrlLed::blickSlowLed() {
  #ifdef ACTIVE_LED_SERIAL
    Serial.println("Blick slow led");
  #endif
  blick_speed = LED_BLINK_SLOW;
  blickLed();
}

void CtrlLed::blickLed() {
  #ifdef ACTIVE_LED_SERIAL
    Serial.println("Blick led");
  #endif
  if (!blick_active) {
    led_active = true;
    blick_active = true;

    xTaskCreatePinnedToCore (
      this->blicking,   // Function to implement the task
      "blicking", // Name of the task
      1000,            // Stack size in words
      this,            // Task input parameter
      2,              // Priority of the task
      NULL,            // Task handle.
      0                // Core where the task should run
    );
  }
}

void CtrlLed::blicking(void *pvParameters) {
  CtrlLed *_led = (CtrlLed *) pvParameters; 
  
  uint8_t _pin_led = _led->getPinLed();
  bool _tmp_led_active = true;

  while (_led->getBlickActive()) {
    if (_tmp_led_active) {
      _tmp_led_active = false;
      digitalWrite(_pin_led, LOW);
    }
    else {
      _tmp_led_active = true;
      digitalWrite(_pin_led, HIGH);
    }
    delay(_led->getBlickSpeed());
  }
  vTaskDelete( NULL );
} 
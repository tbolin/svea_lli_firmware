#ifndef SVEA_BUTTONS

#include <Arduino.h>
#include "Adafruit_MCP23008.h"

namespace buttons {

constexpr uint8_t NUM_BUTTONS = 4;
// The buttons are labeled 0-3 with button 0 to the front and 3 to the back.
constexpr uint8_t BTN_0_PIN = 7;
constexpr uint8_t BTN_1_PIN = 6;
constexpr uint8_t BTN_2_PIN = 5;
constexpr uint8_t BTN_3_PIN = 4;
constexpr uint8_t BTN_PINS[NUM_BUTTONS] = {
    BTN_0_PIN,
    BTN_1_PIN,
    BTN_2_PIN,
    BTN_3_PIN};

constexpr unsigned long LONG_PRESS_LIMIT = 1000; // ms

enum ButtonState {
    UP,
    DOWN,
    LONG_DOWN,
    INVALID_STATE, 
};

enum ButtonEvent {
    NONE,
    PRESSED,
    RELEASED,
    LONG_PRESSED,
    LONG_RELEASED,
    INVALID_EVENT,
};


extern ButtonState current_states[];
extern ButtonEvent current_events[];
extern unsigned long button_press_duration[NUM_BUTTONS];
extern unsigned long last_update_time;

void updateButtons();

ButtonState readButton(uint8_t button_num);


ButtonEvent readEvent(uint8_t button_num);

void setup(Adafruit_MCP23008 &_gpio_extender);

} // namespace buttons
#endif //SVEA_BUTTONS
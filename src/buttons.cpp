#include "buttons.h"
#include <Arduino.h>
#include "Adafruit_MCP23008.h"

namespace buttons {

ButtonState current_states[NUM_BUTTONS] = {UP, UP, UP, UP};
ButtonEvent current_events[NUM_BUTTONS] = {NONE, NONE, NONE, NONE};
unsigned long button_press_duration[NUM_BUTTONS] = {0, 0, 0, 0};
unsigned long last_update_time = millis();
static Adafruit_MCP23008* gpio_extender;

void updateButtons(){
    unsigned long duration_last_update = millis() - last_update_time;
    last_update_time = millis();
    for (int i=0; i<NUM_BUTTONS; i++){
        uint8_t pressed = gpio_extender->digitalRead(BTN_PINS[i]);
        current_events[i] = NONE;
        if (pressed != 0) {
            if (current_states[i] == DOWN){
                current_events[i] = RELEASED;
            } else if (current_states[i] == LONG_DOWN){
                current_events[i] = LONG_RELEASED;
            }
            current_states[i] = UP;
            button_press_duration[i] = 0;
        } else {
            button_press_duration[i] += duration_last_update;
            if (button_press_duration[i] > LONG_PRESS_LIMIT){
                if (current_states[i] == DOWN){
                    current_events[i] = LONG_PRESSED;
                }
                current_states[i] = LONG_DOWN;
            } else {
                if (current_states[i] == UP){
                    current_events[i] = PRESSED;
                }
                current_states[i] = DOWN;
            }
        }
    }
}

ButtonState readButton(uint8_t button_num){
    if (button_num < NUM_BUTTONS){
        return current_states[button_num];
    } else {
        return INVALID_STATE;
    }
}

ButtonEvent readEvent(uint8_t button_num){
    if (button_num < NUM_BUTTONS){
        return current_events[button_num];
    } else {
        return INVALID_EVENT;
    }
}

void setup(Adafruit_MCP23008 &_gpio_extender) {
    gpio_extender = &_gpio_extender;
    gpio_extender->pinMode(BTN_0_PIN, INPUT);
    gpio_extender->pinMode(BTN_1_PIN, INPUT);
    gpio_extender->pinMode(BTN_2_PIN, INPUT);
    gpio_extender->pinMode(BTN_3_PIN, INPUT);

    gpio_extender->pullUp(BTN_0_PIN, HIGH);
    gpio_extender->pullUp(BTN_1_PIN, HIGH);
    gpio_extender->pullUp(BTN_2_PIN, HIGH);
    gpio_extender->pullUp(BTN_3_PIN, HIGH);
}

} // namespace buttons
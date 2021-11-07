#include <Arduino.h>
#include <EEPROM.h>

#include "actuate.h"
#include "utility.h"
#include "buttons.h"

namespace actuate {

//! The 16-bit value corresponding to a neutral duty cycle
unsigned int PWM_OUT_NEUTRAL_TICK[5];

//! The scaling factor between actuation values and the 12-bit values sent to the pwm board.
float OUTPUT_SCALE[5];

//! Storage for the last actuated pwm register values
uint16_t ACTUATED_TICKS[5] = {0, 0, 0, 0, 0};

float STEER_PWM_OUT_MIN_PW = DEFAULT_PWM_OUT_MIN_PW[0];
float STEER_PWM_OUT_MAX_PW = DEFAULT_PWM_OUT_MAX_PW[0];


void setup(){
  // Setup pin modes for pwm output pins 
  for (int i = 0; i < 5; i++){
    pinMode(PWM_OUT_PINS[i], OUTPUT);
    analogWriteFrequency(PWM_OUT_PINS[i], PWM_OUT_FREQUENCY); 
  }
  analogWriteResolution(PWM_OUT_BITS);
  // Calculate scaling values
  for (int i=0; i<5; i++){
    unsigned int min_pwm_tick = DEFAULT_PWM_OUT_MIN_PW[i]*PWM_OUT_RES*PWM_OUT_FREQUENCY;
    unsigned int max_pwm_tick = DEFAULT_PWM_OUT_MAX_PW[i]*PWM_OUT_RES*PWM_OUT_FREQUENCY;
    PWM_OUT_NEUTRAL_TICK[i] = int((min_pwm_tick + (max_pwm_tick - min_pwm_tick)*0.5)/1000.0 + 0.5);
    OUTPUT_SCALE[i] = ((PWM_OUT_FREQUENCY*PWM_OUT_RES*DEFAULT_PWM_OUT_MIN_PW[i])/1000.0 - PWM_OUT_NEUTRAL_TICK[i])/(float)ACTUATION_MIN;
  }
  float max_pwm;
  float min_pwm;
  if (loadSteeringValues(min_pwm, max_pwm)){
    setSteeringPwm(min_pwm, max_pwm);
  }
}

static Actuation current_actuated_values;

void getActuatedValues(Actuation& values){
  values = current_actuated_values;  
}


inline void setPwmDriver(act_t channel, act_t value){
  if (abs_difference(value, ACTUATION_NEUTRAL) < DEAD_ZONE) {
    value = ACTUATION_NEUTRAL;
  }
  uint16_t off_tick = PWM_OUT_NEUTRAL_TICK[channel] + OUTPUT_SCALE[channel]*value;
  ACTUATED_TICKS[channel] = off_tick;
  analogWrite(PWM_OUT_PINS[channel], off_tick);
}

uint8_t actuate(const Actuation& new_values){
  /* Set steering and velocity */
  
  uint8_t has_changed = 0;
  for (size_t i=0; i<new_values.NUMEL; i++){
    if (new_values.at(i) != current_actuated_values.at(i)
        && new_values.at(i) != ACTUATION_PREVIOUS) {
      setPwmDriver(i, new_values.at(i));
      current_actuated_values.at(i) = new_values.at(i);
      has_changed++;
    }
  }
  return has_changed;
}

/* 
 *  Steering calibration functions
 */

CalibState callibrateSteering(){
  const uint8_t calib_button = 0;
  const uint8_t abort_button = 1;
  static CalibState state = NOT_CALIBRATING;
  static float max_pwm = DEFAULT_PWM_OUT_MAX_PW[0];
  static float min_pwm = DEFAULT_PWM_OUT_MIN_PW[0];
  static unsigned long done_time;
  const unsigned long done_duration = 1500; //ms
  if (buttons::readEvent(abort_button) == buttons::PRESSED){
    state = NOT_CALIBRATING;
    if(loadSteeringValues(min_pwm, max_pwm)){
      setSteeringPwm(min_pwm, max_pwm);
    }
  }
  switch (state)
  {
  case NOT_CALIBRATING:
    if (buttons::readEvent(calib_button) == buttons::LONG_PRESSED){
      state = TURN_LEFT;
      int steer_ix = 0;
      max_pwm = DEFAULT_PWM_OUT_MAX_PW[steer_ix];
      min_pwm = DEFAULT_PWM_OUT_MIN_PW[steer_ix];
      setSteeringPwm(min_pwm, max_pwm);
    }
    break;
  case TURN_LEFT:
    if (buttons::readEvent(calib_button) == buttons::PRESSED){
      min_pwm = 1000.0 * ACTUATED_TICKS[0] / (PWM_OUT_RES*PWM_OUT_FREQUENCY);
      state = TURN_RIGHT;
    }
    break;
  case TURN_RIGHT:
    if (buttons::readEvent(calib_button) == buttons::PRESSED){
      max_pwm = 1000.0 * ACTUATED_TICKS[0] / (PWM_OUT_RES*PWM_OUT_FREQUENCY);
      setSteeringPwm(min_pwm, max_pwm);
      saveSteeringValues(min_pwm, max_pwm);
      done_time = millis();
      state = DONE;
    }
    break;
  case DONE:
    if(millis() - done_time < done_duration){
    } else {
      state = NOT_CALIBRATING;
    }
    break;
  }
  return state;
}

void setSteeringPwm(float desired_min_pwm, float desired_max_pwm){
  const uint8_t steer_ix = 0;
  STEER_PWM_OUT_MIN_PW = desired_min_pwm;
  STEER_PWM_OUT_MAX_PW = desired_max_pwm;
  unsigned int min_pwm_tick = desired_min_pwm*PWM_OUT_RES*PWM_OUT_FREQUENCY;
  unsigned int max_pwm_tick = desired_max_pwm*PWM_OUT_RES*PWM_OUT_FREQUENCY;
  PWM_OUT_NEUTRAL_TICK[steer_ix] = int((min_pwm_tick + (max_pwm_tick - min_pwm_tick)*0.5)/1000.0 + 0.5);
  OUTPUT_SCALE[steer_ix] = ((PWM_OUT_FREQUENCY*PWM_OUT_RES*desired_min_pwm)/1000.0 - PWM_OUT_NEUTRAL_TICK[steer_ix])/(float)ACTUATION_MIN;
}
 

bool loadSteeringValues(float &min_pwm, float &max_pwm){
  int eeAddress = EEP_STEERING_ADDRESS;
  uint8_t data = EEPROM.read(eeAddress);
  if (data == 255) {
      return false;
  }
  EEPROM.get(eeAddress, min_pwm);
  eeAddress += sizeof(float);
  data = EEPROM.read(eeAddress);
  if (data == 255) {
      return false;
  }
  EEPROM.get(eeAddress, max_pwm);
  return true;
}


void saveSteeringValues(float min_pwm, float max_pwm){
  int eeAddress = EEP_STEERING_ADDRESS;
  EEPROM.put(eeAddress, min_pwm);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, max_pwm);
}


void resetSteeringValues(){
  int start_addr = EEP_STEERING_ADDRESS;
  int end_addr = EEP_STEERING_ADDRESS + 2*sizeof(float);
  for (int addr=start_addr; addr < end_addr; addr++){
      EEPROM.write(addr, 255);
  }
  const int steer_ix = 0;
  setSteeringPwm(DEFAULT_PWM_OUT_MIN_PW[steer_ix], DEFAULT_PWM_OUT_MAX_PW[steer_ix]);
}

} //namespace actuation

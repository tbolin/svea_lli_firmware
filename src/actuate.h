#ifndef SVEA_ACTUATE
#define SVEA_ACTUATE

#include "actuation.h"

namespace actuate{

static const int PWM_OUT_BITS = 16; //!< Output pwm resolution in bits
static const int PWM_OUT_RES = 1 << (PWM_OUT_BITS); //!< Output pwm resolution in number of states

static const float PWM_OUT_FREQUENCY = 100.0;     //!< Pwm frequency (Hz)
//! Default actuation minium pulse widths (corresponds to -127)
static const float DEFAULT_PWM_OUT_MIN_PW[] = {0.800, 1.000, 1.000, 1.000, 2.000};
//! Default actuation maximum pulse widths (correspoinds to 127)
static const float DEFAULT_PWM_OUT_MAX_PW[] = {2.200, 2.000, 2.000, 2.000, 1.000};

static constexpr Actuation IDLE_ACTUATION = {
  ACTUATION_NEUTRAL,
  ACTUATION_NEUTRAL,
  ACTUATION_MIN,
  ACTUATION_MIN,
  ACTUATION_MIN
};

static const act_t DEAD_ZONE = 2; //!< Deadzone for actuation signals

/*!  
* @defgroup PwmOutputChannels PWM output pins on the Teensy
*/
/*@{*/
static const act_t PWM_OUT_STEER_PIN = 15; //!< Pwm pin for steering
static const act_t PWM_OUT_VELOC_PIN = 5; //!< Pwm pin for velocity
static const act_t PWM_OUT_GEAR_PIN = 14;  //!< Pwm pin for transmission
static const act_t PWM_OUT_FDIFF_PIN = 9; //!< Pwm pin for front differential lock
static const act_t PWM_OUT_RDIFF_PIN = 6; //!< Pwm pin for rear differential lock
//! Array with mapping for the PWM channels
static const act_t PWM_OUT_PINS[5] = {
    PWM_OUT_STEER_PIN,
    PWM_OUT_VELOC_PIN,
    PWM_OUT_GEAR_PIN,
    PWM_OUT_FDIFF_PIN,
    PWM_OUT_RDIFF_PIN
};
/*@}*/

/**
 * @brief Setup actuation 
 */
void setup();

/*
 * ACTUATION FUNCTIONS
 */

/*! 
 * @brief Set actuation PWM
 * convert a 8 bit actuation value to a pwm signal and send it to the pwm board. 
 * The value gets scaled to a duration that suits the servos (approximately 
 * 1 to 2 milli seconds).
 * @see INPUT_SCALE
 * @see PWM_NEUTRAL_TICK
 * To avoid servo jitter at neutral a small dead zone exists around 0. 
 * @see DEAD_ZONE
 * @param channel The channel (pin) of the pwm board to send to. 
 * @param in_value Value, between -127 and 127. to send.
 */ 
inline void setPwmDriver(act_t channel, act_t actuation_value);

/*! @brief Send settings to the pwm board through setPwmDriver()
 * 
 * If any setting or actuation code have changed, the current 
 * actuation values and flags will be published on /lli/ctrl_actuated.
 * If nothing have been changed, nothing will be sent to the
 * pwm register or /lli/ctrl_actuated.
 * @see setPwmDriver
 * @param actuation_values array containg 5 values. 
 */
uint8_t actuate(const Actuation& new_values);

/*!
 * @brief return the currently active actuation values
 */
void getActuatedValues(Actuation& values);


/* Steering calibration */

//! enum representing states of the calibrate function
enum CalibState {
    NOT_CALIBRATING,
    TURN_LEFT,
    TURN_RIGHT,
    DONE,
};

//! EEPROM address where the steering calibration values are stored.
const int EEP_STEERING_ADDRESS = 0;

/*!
 * @brief Steering callibration functionality. Should be called in every loop update.
 *
 * Initiate callibration by holding down button 0 for 1 second.
 * The LEDs should turn yellow. Now turn the tires as far to the left
 * as they can go without pushing against the chassis. 
 * Push button 0 again. The LEDs should turn blue. 
 * Turn the tire as far to the right as they can go without
 * pushing against the chassis. 
 * Push button 0 again and the LEDs should blink for a short while.
 * The callibration is complet and the values have been saved to flash.
 * 
 * The calibration process can be aborted by pushing button 1.
 * 
 * @return true if a calibration is ongoing, false otherwise
 */
CalibState callibrateSteering();

/** @brief set the pulse widths that will correspond to the min and max actuation values
 * 
 * An actuation value of -127 will correspond to `desired_min_pwm`, and
 * an actuation value of 127 will correspond to `desired_max_pwm`.
 */ 
void setSteeringPwm(float desired_min_pwm, float desired_max_pwm);

/**
 * @brief Load saved steering claibration values from EEPROM. 
 * return true if the values are found
 */
bool loadSteeringValues(float &min_pwm, float &max_pwm);

/**
 * @brief Save steering calibration pulse widths to EEPROM.
 */
void saveSteeringValues(float min_pwm, float max_pwm);

/**
 * @brief reset steering calibration to default and ovverwrite saved values
 */
void resetSteeringValues();

} // namespace actuation
#endif //SVEA_ACTUATION
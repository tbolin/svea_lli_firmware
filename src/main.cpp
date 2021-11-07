#include <Arduino.h>

#include <ros.h>
#include <limits>
#include <i2c_driver.h>
#include <svea_msgs/lli_ctrl.h>
#include <svea_msgs/lli_encoder.h>
#include <svea_msgs/lli_emergency.h>
#include "encoders.h"
// #include "settings.h"
#include "svea_teensy.h"
#include "pwm_reader.h"
#include "utility.h"
#include "Adafruit_MCP23008.h"
#include "led_control.h"
#include "buttons.h"
#include "actuate.h"
/*! @file main.cpp*/ 


const uint8_t STEERING_CLOCKWISE = 1;
const uint8_t STEERING_COUNTERCLOCKWISE = -1;
//! Sets the steering direction that is sent and recieved from ROS
const uint8_t STEERING_DIRECTION = STEERING_COUNTERCLOCKWISE;

/*!
 * @brief set the control code in messages sent to ROS
 */
inline uint8_t getStatusFlags() {
  return SW_IDLE 
         | pwm_reader::REM_IDLE << 1 
         | pwm_reader::REM_OVERRIDE << 2 
         | SW_EMERGENCY << 3;
}

void publishActuationValues(const actuate::Actuation& values, lli_ctrl_out_t& msg, ros::Publisher& pub){
  msg.steering = STEERING_DIRECTION*values.steering();
  msg.velocity = values.velocity();
  msg.trans_diff = bit(ENABLE_GEARCHANGE_BIT)
                 | bit(ENABLE_FDIFCHANGE_BIT)
                 | bit(ENABLE_RDIFCHANGE_BIT);
  for (int i=0; i<3; i++){
    msg.trans_diff += values.at(i+2) > actuate::ACTUATION_NEUTRAL ? bit(i) : 0;
  }
  msg.ctrl = getStatusFlags();
  pub.publish(&msg);
}

void publishRemoteReading(const actuate::Actuation& values){
  publishActuationValues(
    values,
    MSG_REMOTE,
    remote_pub
  );
}

void actuateAndPublish(const actuate::Actuation& values){
  static uint8_t last_status = 0; // Code that was last sent to ROS
  uint8_t changed = actuate::actuate(values);
  uint8_t new_status = getStatusFlags();
  if (changed != 0 || last_status != new_status){
    actuate::Actuation new_values;
    actuate::getActuatedValues(new_values);
    publishActuationValues(
      new_values,
      MSG_ACTUATED,
      ctrl_actuated_pub
    );
    last_status = new_status;
  }
}

/*
 * SETUP ROS
 */

/*!
 * @brief Callback function for control requests from ROS
 * Interprets the message and sends the values to the pwm board
 * through actuate().
 * 
 * @param data Message to be evaluated
 */
void callbackCtrlRequest(const lli_ctrl_in_t& data){  
  SW_ACTUATION.steering(STEERING_DIRECTION * data.steering);
  SW_ACTUATION.velocity(data.velocity);
  
  // Set the on/off values
  for (int i=0; i<3; i++){
    // Only change gear/diff settings if the corresponding enable change bit is set
    if(data.trans_diff & bit(ENABLE_ACT_CHANGE_BITS[i])){
      const actuate::act_t on = actuate::ACTUATION_MAX;
      const actuate::act_t off = actuate::ACTUATION_MIN;
      int8_t is_on = data.trans_diff & bit(ACT_BITS[i]);
      SW_ACTUATION.at(i+2) = is_on ? on : off; 
    }
    else { // Otherwise use the previous value
      SW_ACTUATION.at(i+2) = actuate::ACTUATION_PREVIOUS;
    }
  }
  SW_IDLE = false; 
  SW_T_RECIEVED = millis();
  if (!pwm_reader::REM_OVERRIDE && !SW_EMERGENCY){
    actuateAndPublish(SW_ACTUATION);
  }
}

/*!
 * @brief Callback function for emergency requests from ROS
 * Set/clear the emergency flag depending on message content.
 * The ID field functionality is not yet implemented.
 * 
 * @param data Message to be evaluated
 */
void callbackEmergency(const svea_msgs::lli_emergency& data){
    SW_EMERGENCY = data.emergency;
    SW_IDLE = false;
    SW_T_RECIEVED = millis();
}
/*@}*/

// END OF ROS SETUP

/*!
 * @brief Check if the emergency brake should be engaged.
 * Should be called every update loop.
 * 
 * The emergency brake will be activated if the SW_EMERGENCY
 * flag is true. A braking sequence is then initiated.
 * The sequence first make sures that the ESC is not in
 * a reverse state, and then applies full brakes. 
 * For proper functionality all other actuation sources
 * must respect the SW_EMERGENCY flag and not send actuation
 * signals until it is cleared.
 * 
 * @return true if the emergency brake is engaged, false otherwise.
 */
bool checkEmergencyBrake(){
  enum States {
    NO_EMERGENCY,
    EMERGENCY_SET,
    WAIT_FOR_UNSET_REVERSE,
    BRAKING,
    DONE_BRAKING,
  };
  static States state = NO_EMERGENCY;
  static unsigned long last_time = millis();
  const unsigned long reverse_wait_time = 50; // (ms)
  const actuate::Actuation init_brake_actuation = {-128,15,-128,-128,-128};
  const actuate::Actuation brake_actuation = {-128,-127,-128,-128,-128};
  // const unsigned long minimum_emergency_duration = 500;
  unsigned long wait_duration = (millis() - last_time);
  if (SW_EMERGENCY == false) {
    state = NO_EMERGENCY;
  }
  switch (state)
  {
  case NO_EMERGENCY:
    if (SW_EMERGENCY){
      state = EMERGENCY_SET;
    } else {
      break;
    }
  case EMERGENCY_SET:
    actuateAndPublish(init_brake_actuation);
    last_time = millis();
    state = WAIT_FOR_UNSET_REVERSE;
    break;
  case WAIT_FOR_UNSET_REVERSE:
    if (wait_duration > reverse_wait_time) {
      state = BRAKING;
    } else {
      break;
    }
  case BRAKING:
    actuateAndPublish(brake_actuation);
    state = DONE_BRAKING;
    break;
  case DONE_BRAKING:
    break;
  default:
    break;
  }
  return state != NO_EMERGENCY;
}

void EncoderReadingToMsg(const encoders::encoder_reading_t& reading, lli_encoder_t& msg){
  msg.right_ticks = reading.right_ticks;
  msg.left_ticks = reading.left_ticks;
  msg.right_time_delta = reading.right_time_delta;
  msg.left_time_delta = reading.left_time_delta;
}

//! Setup ROS
void rosSetup() {
  nh.getHardware()->setBaud(SERIAL_BAUD_RATE);
  nh.initNode();
  // NOTE: Putting advertise before subscribe destroys 
  //       EVERYTHING :DDDD~~~~~
  nh.subscribe(ctrl_request);
  nh.subscribe(emergency_request);
  nh.advertise(remote_pub);
  nh.advertise(ctrl_actuated_pub);
  nh.advertise(encoder_pub);
  nh.advertise(debug_pub);
}

/* GPIO extender variables */
constexpr uint8_t GPIO_ADDRESS = 0;
constexpr uint8_t SERVO_PWR_ENABLE_PIN = 3;
Adafruit_MCP23008 gpio_extender(Master1);

//! Arduino setup function
void setup() {
  actuate::setup();
  /* ROS setup */
  rosSetup();
  pinMode(LED_BUILTIN, OUTPUT);
  gpio_extender.begin(GPIO_ADDRESS);
  gpio_extender.pinMode(SERVO_PWR_ENABLE_PIN, OUTPUT);
  buttons::setup(gpio_extender);
  led::setup(gpio_extender);
  pwm_reader::setup();
  encoders::setup();
}

//! Main loop
void loop() {
  static bool all_idle = false;
  int sw_status = nh.spinOnce();
  unsigned long d_since_last_msg = millis() - SW_T_RECIEVED;
  if (sw_status != ros::SPIN_OK || d_since_last_msg > SW_TIMEOUT) {
    SW_IDLE = true;
  }
  checkEmergencyBrake();
  actuate::Actuation remote_actuations;
  if (pwm_reader::processPwm(remote_actuations)){
    if (!pwm_reader::REM_IDLE){
      publishRemoteReading(remote_actuations);
      if ((SW_IDLE && !SW_EMERGENCY) || pwm_reader::REM_OVERRIDE){
        actuateAndPublish(remote_actuations);
      }
      if (d_since_last_msg > EMERGENCY_T_CLEAR_LIMIT
          && pwm_reader::REM_OVERRIDE 
          && SW_EMERGENCY) {
        SW_EMERGENCY = false;
      }
    }
  }
  if (pwm_reader::REM_IDLE && SW_IDLE && !SW_EMERGENCY) {
    if (!all_idle){
      actuateAndPublish(actuate::IDLE_ACTUATION);
      gpio_extender.digitalWrite(SERVO_PWR_ENABLE_PIN, LOW);
    }
    all_idle = true;
  } else {
    gpio_extender.digitalWrite(SERVO_PWR_ENABLE_PIN, HIGH);
    all_idle = false;
  }
  encoders::encoder_reading_t reading;
  if (encoders::processEncoderTicks(reading)){
    EncoderReadingToMsg(reading, MSG_ENCODER);
    encoder_pub.publish(&MSG_ENCODER);
  }
  if (gpio_extender.update() == DONE){
    ;
  }
  buttons::updateButtons();
  actuate::CalibState calibration_status = actuate::callibrateSteering();
  // LED logic
  switch (calibration_status)
  {
  case actuate::CalibState::NOT_CALIBRATING:
    if (all_idle && !SW_EMERGENCY) {
      led::blinkLEDs();
    }
    else {
      if(SW_IDLE){
        led::setLED(0, led::color_red);
      } else {
        led::setLED(0, led::color_green);
      }
      if(pwm_reader::REM_IDLE){
        led::setLED(1, led::color_red);
      } else {
        led::setLED(1, led::color_green);
      }
      if(!pwm_reader::REM_OVERRIDE){
        led::setLED(2, led::color_red);
      } else {
        led::setLED(2, led::color_green);
      }
      if(!SW_EMERGENCY){
        led::setLED(3, led::color_red);
      } else {
        led::setLED(3, led::color_green);
      }
    }
    break;
  case actuate::CalibState::TURN_LEFT:
    led::setLEDs(led::color_blue);
    break;
  case actuate::CalibState::TURN_RIGHT:
    led::setLEDs(led::color_blue);
    break;
  case actuate::CalibState::DONE:
    led::blinkLEDs();
    break;
  }

  led::updateLEDs();
}

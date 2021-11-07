#ifndef SVEA_LLI
#define SVEA_LLI
#include <Arduino.h>
#include <ros.h>
#include <svea_msgs/lli_ctrl.h>
#include <svea_msgs/lli_encoder.h>
#include <svea_msgs/lli_emergency.h>
#include <std_msgs/UInt16.h>
#include "actuation.h"

/*! @file svea_teensy.h*/ 

#define DURATION_DEBUG 0

// //! Sampling interval for the wheel encoders in micro seconds
// const unsigned long ENCODER_SAMPLE_INTERVAL = 25000;


//! Baud rate for serial transmisions
#if DURATION_DEBUG
const uint32_t SERIAL_BAUD_RATE = 115200;
#else
const uint32_t SERIAL_BAUD_RATE = 250000;
#endif

/*
 * Software and remote state constants
 */
const unsigned long SW_TIMEOUT = 200;  //!< Duration (ms) from last recieved computer 
                                       //!< message when the computer will count as idle
//! Maximum number of ROS subscribers
const uint8_t MAX_ROS_SUBSCRIBERS = 2;
//! Maximum number of ROS publishers
const uint8_t MAX_ROS_PUBLISHERS = 5;
//! Maximum number of ROS subscribers
const uint16_t ROS_IN_BUFFER_SIZE = 300;
const uint16_t ROS_OUT_BUFFER_SIZE = 300;

/*  
 * Message type definitions and related constants
 */ 
typedef svea_msgs::lli_ctrl lli_ctrl_in_t; //!< Message type for incomming messages
typedef svea_msgs::lli_ctrl lli_ctrl_out_t; //!< Message type for outgoing messages'
typedef svea_msgs::lli_encoder lli_encoder_t; //!< Message type for encoder messages

/*!  
 * @defgroup MsgBitPositions Bit positions used for the trans_diff_ctrl field in messages
 */
/*@{*/
const uint8_t GEAR_BIT = 0;   //!< Bit used for gear value (0 unlocked, 1 locked)
const uint8_t FDIFF_BIT = 1;  //!< Bit used for front differential value (0 unlocked, 1 locked)
const uint8_t RDIFF_BIT = 2;  //!< Bit used for rear differential value (0 unlocked, 1 locked)
//! Vector with the bit postitions in msg.gear_diff in order: gear, front diff, rear diff
const uint8_t ACT_BITS[3] = {GEAR_BIT, FDIFF_BIT, RDIFF_BIT}; 
//! Bit indicating if the GEAR_BIT value should be read from incoming messages
const uint8_t ENABLE_GEARCHANGE_BIT = 3;
//! Bit indicating if the front differential values should be read from incoming messages
const uint8_t ENABLE_FDIFCHANGE_BIT = 4;
//! Bit indicating if the rear differential values should be read from incoming messages
const uint8_t ENABLE_RDIFCHANGE_BIT = 5;
//! Vector with the enable change bits in order: gear, front diff, rear diff
const uint8_t ENABLE_ACT_CHANGE_BITS[3] = {ENABLE_GEARCHANGE_BIT, ENABLE_FDIFCHANGE_BIT, ENABLE_RDIFCHANGE_BIT};
/*@}*/ 

/*
 * Storage variables
 */

/*!  
 * @defgroup ActuationValueStorage Actuation value storage
 * The order is Steering, velocity, gear, front differential, rear differential
 */
/*@{*/
//! Actuation values sent from the computer
actuate::Actuation SW_ACTUATION;

/*!  
 * @defgroup StatusVariables Status variables
 */
/*@{*/
unsigned long SW_T_RECIEVED=millis(); //!< Time when last message was recieved from the computer
bool SW_IDLE = true; //!< True if the computer is considered idle

/*!
 *  True if the computer has set an emergency.
 *  Should bloc other actuation signals from computer until cleared.
 */
bool SW_EMERGENCY = false;
//! Emergency cleared if override active and 
//  no message has been recieved for this many ms.
unsigned long EMERGENCY_T_CLEAR_LIMIT = 5000;
/*@}*/

/* Function definitions */
inline uint8_t getActuatedCode();
void callbackCtrlRequest(const lli_ctrl_in_t& data);
void callbackEmergency(const svea_msgs::lli_emergency& data);


/*!
 * @defgroup ROSSetup Variables used by ROS
 */
/*@{*/
//! NodeHandle class definition
ros::NodeHandle_<ArduinoHardware, 
                 MAX_ROS_SUBSCRIBERS,
                 MAX_ROS_PUBLISHERS,
                 ROS_IN_BUFFER_SIZE, 
                 ROS_OUT_BUFFER_SIZE> nh;
lli_ctrl_out_t MSG_REMOTE; //!< Message used for sending the remote signals
lli_ctrl_out_t MSG_ACTUATED; //!< Message sending actuated messages
lli_encoder_t MSG_ENCODER; //!< Message used for outgoing wheel encoder messages
lli_encoder_t MSG_DEBUG; //!< Message used for misc debugging
ros::Publisher remote_pub("lli/remote", &MSG_REMOTE); //!< Remote message publisher
ros::Publisher ctrl_actuated_pub("lli/ctrl_actuated", &MSG_ACTUATED); //!< Actuated control message publisher
ros::Publisher encoder_pub("lli/encoder", &MSG_ENCODER); //!< Encoder reading publisher
ros::Publisher debug_pub("lli/debug", &MSG_DEBUG); //!< Encoder reading publisher
ros::Subscriber<lli_ctrl_in_t> ctrl_request("lli/ctrl_request", &callbackCtrlRequest ); //!< Controll request subscriber
ros::Subscriber<svea_msgs::lli_emergency> emergency_request("lli/emergency", &callbackEmergency); //!< Controll request subscriber

/*@}*/
#endif /* SVEA_LLI */

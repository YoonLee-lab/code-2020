/**
 * MC_motor_position_information.cpp - CAN message parser: RMS Motor Controller motor position information message
 * Created by Nathan Cheek, November 22, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: A5
 * DESCR: MC Motor Position Information
 * MACRO: MC_MOTOR_POS_INFO, previously: ID_MC_MOTOR_POSITION_INFORMATION
 * STRUCT: MCMotorPosInfo_t, previously: CAN_message_mc_motor_position_information_t 
 * CLASS: MC_motor_position_information
 * DATA:
 *      motor_angle                 [0:1]
 *      motor_speed                 [2:3]
 *      electrical_output_frequency [4:5]
 *      delta_resolver_filtered     [6:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_motor_position_information
 */
MC_motor_position_information::MC_motor_position_information() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_motor_position_information::MC_motor_position_information(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_motor_position_information::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.motor_angle), &buf[0], sizeof(int16_t));
    memcpy(&(message.motor_speed), &buf[2], sizeof(int16_t));
    memcpy(&(message.electrical_output_frequency), &buf[4], sizeof(int16_t));
    memcpy(&(message.delta_resolver_filtered), &buf[6], sizeof(int16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_motor_position_information::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.motor_angle), sizeof(int16_t));
    memcpy(&buf[2], &(message.motor_speed), sizeof(int16_t));
    memcpy(&buf[4], &(message.electrical_output_frequency), sizeof(int16_t));
    memcpy(&buf[6], &(message.delta_resolver_filtered), sizeof(int16_t));
}

/**
 * To get the motor_angle
 * @return motor angle
 */
int16_t MC_motor_position_information::get_motor_angle() {
    return message.motor_angle;
}

/**
 * To get the motor_speed
 * @return motor speed
 */
int16_t MC_motor_position_information::get_motor_speed() {
    return message.motor_speed;
}

/**
 * To get the electrical_output_frequency
 * @return frequency
 */
int16_t MC_motor_position_information::get_electrical_output_frequency() {
    return message.electrical_output_frequency;
}

/**
 * To get the delta_resolver_filtered
 * @return delta
 */
int16_t MC_motor_position_information::get_delta_resolver_filtered() {
    return message.delta_resolver_filtered;
}

/**
 * MC_temperatures_3.cpp - CAN message parser: RMS Motor Controller temperatures 3 message
 * Created by Nathan Cheek, November 22, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: A2
 * DESCR: Motor Controller Temperatures #3
 * MACRO: ID_MC_TEMPERATURES_3
 * STRUCT: CAN_message_mc_temperatures_3_t 
 * CLASS: MC_temperatures_3
 * DATA:
 *      rtd_4_temperature [0:1]
 *      rtd_5_temperature [2:3]
 *      motor_temperature [4:5]
 *      torque_shudder    [6:7]
 * 
 * 
 * RTD = resistance temperature detector
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_temperatures_3
 */
MC_temperatures_3::MC_temperatures_3() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_temperatures_3::MC_temperatures_3(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_temperatures_3::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.rtd_4_temperature), &buf[0], sizeof(int16_t));
    memcpy(&(message.rtd_5_temperature), &buf[2], sizeof(int16_t));
    memcpy(&(message.motor_temperature), &buf[4], sizeof(int16_t));
    memcpy(&(message.torque_shudder), &buf[6], sizeof(int16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_temperatures_3::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.rtd_4_temperature), sizeof(int16_t));
    memcpy(&buf[2], &(message.rtd_5_temperature), sizeof(int16_t));
    memcpy(&buf[4], &(message.motor_temperature), sizeof(int16_t));
    memcpy(&buf[6], &(message.torque_shudder), sizeof(int16_t));
}

/**
 * To get the rtd_4_temperature
 * @return rtd 4
 */
int16_t MC_temperatures_3::get_rtd_4_temperature() {
    return message.rtd_4_temperature;
}

/**
 * To get the rtd_5_temperature
 * @return rtd 5
 */
int16_t MC_temperatures_3::get_rtd_5_temperature() {
    return message.rtd_5_temperature;
}

/**
 * To get the motor_temperature
 * @return motor temperature
 */
int16_t MC_temperatures_3::get_motor_temperature() {
    return message.motor_temperature;
}

/**
 * To get the torque_shudder
 * @return torque shudder
 */
int16_t MC_temperatures_3::get_torque_shudder() {
    return message.torque_shudder;
}

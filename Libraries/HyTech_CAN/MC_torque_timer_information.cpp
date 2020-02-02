/**
 * MC_torque_timer_information.cpp - CAN message parser: RMS Motor Controller torque timer information message
 * Created by Nathan Cheek, November 22, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: AC
 * DESCR: MC Torque Timer Information
 * MACRO: ID_MC_TORQUE_TIMER_INFORMATION
 * STRUCT: CAN_message_mc_torque_timer_information_t 
 * CLASS: MC_torque_timer_information
 * DATA:
 *      commanded_torque [0:1]
 *      torque_feedback  [2:3]
 *      power_on_timer   [4:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_torque_timer_information
 */
MC_torque_timer_information::MC_torque_timer_information() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_torque_timer_information::MC_torque_timer_information(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_torque_timer_information::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.commanded_torque), &buf[0], sizeof(int16_t));
    memcpy(&(message.torque_feedback), &buf[2], sizeof(int16_t));
    memcpy(&(message.power_on_timer), &buf[4], sizeof(uint32_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_torque_timer_information::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.commanded_torque), sizeof(int16_t));
    memcpy(&buf[2], &(message.torque_feedback), sizeof(int16_t));
    memcpy(&buf[4], &(message.power_on_timer), sizeof(uint32_t));
}

/**
 * To get the commanded_torque
 * @return commanded torque
 */
int16_t MC_torque_timer_information::get_commanded_torque() {
    return message.commanded_torque;
}

/**
 * To get the torque_feedback
 * @return torque feedback
 */
int16_t MC_torque_timer_information::get_torque_feedback() {
    return message.torque_feedback;
}

/**
 * To get the power_on_timer
 * @return power
 */
uint32_t MC_torque_timer_information::get_power_on_timer() {
    return message.power_on_timer;
}

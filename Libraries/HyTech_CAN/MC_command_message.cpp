/**
 * MC_command_message.cpp - CAN message parser: RMS Motor Controller command message
 * Created by Nathan Cheek, November 20, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: C0
 * DESCR: MC Command Message
 * MACRO: ID_MC_COMMAND_MESSAGE
 * STRUCT: CAN_message_mc_command_message_t 
 * CLASS: MC_command_message
 * DATA:
 *      torque_command         [0:1]
 *      angular_velocity       [2:3]
 *      direction              [4]
 *      inverter_enable        [5]
 *      commanded_torque_limit [6:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_fault_codes
 */
MC_command_message::MC_command_message() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_command_message::MC_command_message(uint8_t buf[8]) {
    load(buf);
}

/**
 * Constructor, loading in the data from variables
 * @param torque_command: torque command
 * @param angular_velocity: angular velocity
 * @param direction: direction
 * @param inverter_enable: whether inverter is enabled
 * @param discharge_enable: whether discharge is enabled
 * @param commanded_torque_limit: commanded torque limit
 */
MC_command_message::MC_command_message(int16_t torque_command,
    int16_t angular_velocity, bool direction, bool inverter_enable,
    bool discharge_enable, int16_t commanded_torque_limit) {
    message = {};
    set_torque_command(torque_command);
    set_angular_velocity(angular_velocity);
    set_direction(direction);
    set_inverter_enable(inverter_enable);
    set_discharge_enable(discharge_enable);
    set_commanded_torque_limit(commanded_torque_limit);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_command_message::load(uint8_t buf[]) {
    message = {};
    memcpy(&(message.torque_command), &buf[0], sizeof(uint16_t));
    memcpy(&(message.angular_velocity), &buf[2], sizeof(uint16_t));
    memcpy(&(message.direction), &buf[4], sizeof(bool));
    memcpy(&(message.inverter_enable_discharge_enable), &buf[5], sizeof(uint8_t));
    memcpy(&(message.commanded_torque_limit), &buf[6], sizeof(uint16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_command_message::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.torque_command), sizeof(uint16_t));
    memcpy(&buf[2], &(message.angular_velocity), sizeof(uint16_t));
    memcpy(&buf[4], &(message.direction), sizeof(bool));
    memcpy(&buf[5], &(message.inverter_enable_discharge_enable), sizeof(uint8_t));
    memcpy(&buf[6], &(message.commanded_torque_limit), sizeof(uint16_t));
}

/**
 * Getter functions
 */
int16_t MC_command_message::get_torque_command() {
    return message.torque_command;
}

int16_t MC_command_message::get_angular_velocity() {
    return message.angular_velocity;
}

bool MC_command_message::get_direction() {
    return message.direction;
}

bool MC_command_message::get_inverter_enable() {
    return message.inverter_enable_discharge_enable & 0x1;
}

bool MC_command_message::get_discharge_enable() {
    return (message.inverter_enable_discharge_enable & 0x2) >> 1;
}

int16_t MC_command_message::get_commanded_torque_limit() {
    return message.commanded_torque_limit;
}

/**
 * Setter functions
 */
void MC_command_message::set_torque_command(int16_t torque_command) {
    message.torque_command = torque_command;
}

void MC_command_message::set_angular_velocity(int16_t angular_velocity) {
    message.angular_velocity = angular_velocity;
}

void MC_command_message::set_direction(bool direction) {
    message.direction = direction & 0x1;
}

void MC_command_message::set_inverter_enable(bool inverter_enable) {
    message.inverter_enable_discharge_enable = 
        (message.inverter_enable_discharge_enable & 0xFE) | (inverter_enable & 0x1);
}

void MC_command_message::set_discharge_enable(bool discharge_enable) {
    message.inverter_enable_discharge_enable = 
        (message.inverter_enable_discharge_enable & 0xFD) | ((discharge_enable & 0x1) << 1);
}

void MC_command_message::set_commanded_torque_limit(int16_t commanded_torque_limit) {
    message.commanded_torque_limit = commanded_torque_limit;
}

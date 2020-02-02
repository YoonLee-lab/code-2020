/**
 * MC_internal_states.cpp - CAN message parser: RMS Motor Controller internal states message
 * Created by Nathan Cheek, November 20, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020. WIP
 * 
 * HEXID: AA
 * DESCR: MC Internal States
 * MACRO: ID_MC_INTERNAL_STATES
 * STRUCT: CAN_message_mc_internal_states_t 
 * CLASS: MC_internal_states
 * DATA:
 *      vsm_state                         [0:1]
 *      inverter_state                    [2]
 *      relay_state                       [3]
 *      inverter_run_mode_discharge_state [4]
 *      inverter_command_mode             [5]
 *      inverter_enable                   [6]
 *      direction_command                 [7]
 *
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_internal_states
 */
MC_internal_states::MC_internal_states() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_internal_states::MC_internal_states(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_internal_states::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.vsm_state), &buf[0], sizeof(uint16_t));
    memcpy(&(message.inverter_state), &buf[2], sizeof(uint8_t));
    memcpy(&(message.relay_state), &buf[3], sizeof(uint8_t));
    memcpy(&(message.inverter_run_mode_discharge_state), &buf[4], sizeof(uint8_t));
    memcpy(&(message.inverter_command_mode), &buf[5], sizeof(uint8_t));
    memcpy(&(message.inverter_enable), &buf[6], sizeof(uint8_t));
    memcpy(&(message.direction_command), &buf[7], sizeof(uint8_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_internal_states::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.vsm_state), sizeof(uint16_t));
    memcpy(&buf[2], &(message.inverter_state), sizeof(uint8_t));
    memcpy(&buf[3], &(message.relay_state), sizeof(uint8_t));
    memcpy(&buf[4], &(message.inverter_run_mode_discharge_state), sizeof(uint8_t));
    memcpy(&buf[5], &(message.inverter_command_mode), sizeof(uint8_t));
    memcpy(&buf[6], &(message.inverter_enable), sizeof(uint8_t));
    memcpy(&buf[7], &(message.direction_command), sizeof(uint8_t));
}

/**
 * To get the vsm_state
 * @return vsm state
 */
uint8_t MC_internal_states::get_vsm_state() {
    return message.vsm_state;
}

/**
 * To get the inverter_state
 * @return inverter state
 */
uint8_t MC_internal_states::get_inverter_state() {
    return message.inverter_state;
}

/**
 * To get the rtd_4_temperature
 * @return rtd 4
 */
bool MC_internal_states::get_relay_active_1() {
    return message.relay_state & 0x01;
}

/**
 * To get the rtd_4_temperature
 * @return rtd 4
 */
bool MC_internal_states::get_relay_active_2() {
    return (message.relay_state & 0x02) >> 1;
}

/**
 * To get the rtd_4_temperature
 * @return rtd 4
 */
bool MC_internal_states::get_relay_active_3() {
    return (message.relay_state & 0x04) >> 2;
}

/**
 * To get the rtd_4_temperature
 * @return rtd 4
 */
bool MC_internal_states::get_relay_active_4() {
    return (message.relay_state & 0x08) >> 3;
}

/**
 * To get the rtd_4_temperature
 * @return rtd 4
 */
bool MC_internal_states::get_relay_active_5() {
    return (message.relay_state & 0x10) >> 4;
}

/**
 * To get the rtd_4_temperature
 * @return rtd 4
 */
bool MC_internal_states::get_relay_active_6() {
    return (message.relay_state & 0x20) >> 5;
}

/**
 * To get the rtd_4_temperature
 * @return rtd 4
 */
bool MC_internal_states::get_inverter_run_mode() {
    return message.inverter_run_mode_discharge_state & 0x01;
}

/**
 * To get the get_inverter_active_discharge_state
 * @return state
 */
uint8_t MC_internal_states::get_inverter_active_discharge_state() {
    return (message.inverter_run_mode_discharge_state & 0xE0) >> 5;
}

/**
 * To get the inverter_command_mode
 * @return whether inverter command mode is enabled
 */
bool MC_internal_states::get_inverter_command_mode() {
    return message.inverter_command_mode;
}

/**
 * To get the inverter_enable
 * @return whether inverter is enabled
 */
bool MC_internal_states::get_inverter_enable_state() {
    return message.inverter_enable & 0x01;
}

/**
 * To get the 
 * @return rtd 4
 */
bool MC_internal_states::get_inverter_enable_lockout() {
    return (message.inverter_enable & 0x80) >> 7;
}

/**
 * To get the direction_command
 * @return direction command
 */
bool MC_internal_states::get_direction_command() {
    return message.direction_command;
}

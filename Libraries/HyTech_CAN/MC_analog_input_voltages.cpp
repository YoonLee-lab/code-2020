/**
 * MC_analog_input_voltages.cpp - CAN message parser: RMS Motor Controller analog input voltages message
 * Created by Ryan Gallaway, December 1, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: A3
 * DESCR: MC Analog Inputs Voltages
 * MACRO: ID_MC_ANALOG_INPUTS_VOLTAGES
 * STRUCT: CAN_message_mc_analog_input_voltages_t  
 * CLASS: MC_analog_input_voltages
 * DATA:
 *      analog_input_1 [0:1]
 *      analog_input_2 [2:3]
 *      analog_input_3 [4:5]
 *      analog_input_4 [6:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_analog_input_voltages
 */
MC_analog_input_voltages::MC_analog_input_voltages() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_analog_input_voltages::MC_analog_input_voltages(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_analog_input_voltages::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.analog_input_1), &buf[0], sizeof(int16_t));
    memcpy(&(message.analog_input_2), &buf[2], sizeof(int16_t));
    memcpy(&(message.analog_input_3), &buf[4], sizeof(int16_t));
    memcpy(&(message.analog_input_4), &buf[6], sizeof(int16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_analog_input_voltages::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.analog_input_1), sizeof(int16_t));
    memcpy(&buf[2], &(message.analog_input_2), sizeof(int16_t));
    memcpy(&buf[4], &(message.analog_input_3), sizeof(int16_t));
    memcpy(&buf[6], &(message.analog_input_4), sizeof(int16_t));
}

/**
 * To get the analog_input_1
 * @return input 1
 */
int16_t MC_analog_input_voltages::get_analog_input_1() {
    return message.analog_input_1;
}

/**
 * To get the analog_input_2
 * @return input 2
 */
int16_t MC_analog_input_voltages::get_analog_input_2() {
    return message.analog_input_2;
}

/**
 * To get the analog_input_3
 * @return input 3
 */
int16_t MC_analog_input_voltages::get_analog_input_3() {
    return message.analog_input_3;
}

/**
 * To get the analog_input_4
 * @return input 4
 */
int16_t MC_analog_input_voltages::get_analog_input_4() {
    return message.analog_input_4;
}

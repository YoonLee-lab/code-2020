/*
 * MC_digital_input_status.cpp - CAN message parser: RMS Motor Controller digital input status message
 * Created by Ryan Gallaway, December 1, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEX ID: A4
 * DESCR: MC Digital Input Status
 * MACRO: ID_MC_DIGITAL_INPUT_STATUS
 * STRUCT: CAN_message_mc_digital_input_status_t 
 * CLASS: MC_digital_input_status
 * DATA:
 *      digital_input_1 [0]
 *      digital_input_2 [1]
 *      digital_input_3 [2]
 *      digital_input_4 [3]
 *      digital_input_5 [4]
 *      digital_input_6 [5]
 *      digital_input_7 [6]
 *      digital_input_8 [7]
 * 
 *  all of the above are boolean
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_temperatures_2
 */
MC_digital_input_status::MC_digital_input_status() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_digital_input_status::MC_digital_input_status(uint8_t buf[]) {
    message = {};
    memcpy(&(message.digital_input_1), &buf[0], sizeof(bool));
    memcpy(&(message.digital_input_2), &buf[1], sizeof(bool));
    memcpy(&(message.digital_input_3), &buf[2], sizeof(bool));
    memcpy(&(message.digital_input_4), &buf[3], sizeof(bool));
    memcpy(&(message.digital_input_5), &buf[4], sizeof(bool));
    memcpy(&(message.digital_input_6), &buf[5], sizeof(bool));
    memcpy(&(message.digital_input_7), &buf[6], sizeof(bool));
    memcpy(&(message.digital_input_8), &buf[7], sizeof(bool));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_digital_input_status::write(uint8_t buf[]) {
    memcpy(&buf[0], &(message.digital_input_1), sizeof(bool));
    memcpy(&buf[1], &(message.digital_input_2), sizeof(bool));
    memcpy(&buf[2], &(message.digital_input_3), sizeof(bool));
    memcpy(&buf[3], &(message.digital_input_4), sizeof(bool));
    memcpy(&buf[4], &(message.digital_input_5), sizeof(bool));
    memcpy(&buf[5], &(message.digital_input_6), sizeof(bool));
    memcpy(&buf[6], &(message.digital_input_7), sizeof(bool));
    memcpy(&buf[7], &(message.digital_input_8), sizeof(bool));
}

/**
 * To get the digital_input_1
 * @return input 1
 */
bool MC_digital_input_status::get_digital_input_1() {
    return message.digital_input_1;
}

/**
 * To get the digital_input_2
 * @return input 2
 */
bool MC_digital_input_status::get_digital_input_2() {
    return message.digital_input_2;
}

/**
 * To get the digital_input_3
 * @return input 3
 */
bool MC_digital_input_status::get_digital_input_3() {
    return message.digital_input_3;
}

/**
 * To get the digital_input_4
 * @return input 4
 */
bool MC_digital_input_status::get_digital_input_4() {
    return message.digital_input_4;
}

/**
 * To get the digital_input_5
 * @return input 5
 */
bool MC_digital_input_status::get_digital_input_5() {
    return message.digital_input_5;
}

/**
 * To get the digital_input_6
 * @return input 6
 */
bool MC_digital_input_status::get_digital_input_6() {
    return message.digital_input_6;
}

/**
 * To get the digital_input_7
 * @return input 7
 */
bool MC_digital_input_status::get_digital_input_7() {
    return message.digital_input_7;
}

/**
 * To get the digital_input_8
 * @return input 8
 */
bool MC_digital_input_status::get_digital_input_8() {
    return message.digital_input_8;
}
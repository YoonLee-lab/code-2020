/**
 * MC_temperatures_2.cpp - CAN message parser: RMS Motor Controller temperatures 2 message
 * Created by Nathan Cheek, November 22, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: A1
 * DESCR: Motor Controller Temperatures #2
 * MACRO: MC_TEMP_2, previously: ID_MC_TEMPERATURES_2
 * STRUCT: MCTemp2_t, previously: CAN_message_mc_temperatures_2_t 
 * CLASS: MC_temperatures_2
 * DATA:
 *      control_board_temperature [0:1]
 *      rtd_1_temperature         [2:3]
 *      rtd_2_temperature         [4:5]
 *      rtd_3_temperature         [6:7]
 * 
 * 
 * RTD = resistance temperature detector
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_temperatures_2
 */
MC_temperatures_2::MC_temperatures_2() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_temperatures_2::MC_temperatures_2(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_temperatures_2::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.control_board_temperature), &buf[0], sizeof(int16_t));
    memcpy(&(message.rtd_1_temperature), &buf[2], sizeof(int16_t));
    memcpy(&(message.rtd_2_temperature), &buf[4], sizeof(int16_t));
    memcpy(&(message.rtd_3_temperature), &buf[6], sizeof(int16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_temperatures_2::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.control_board_temperature), sizeof(int16_t));
    memcpy(&buf[2], &(message.rtd_1_temperature), sizeof(int16_t));
    memcpy(&buf[4], &(message.rtd_2_temperature), sizeof(int16_t));
    memcpy(&buf[6], &(message.rtd_3_temperature), sizeof(int16_t));
}

/**
 * To get the control_board_temperature
 * @return control board temperature
 */
int16_t MC_temperatures_2::get_control_board_temperature() {
    return message.control_board_temperature;
}

/**
 * To get the rtd_1_temperature
 * @return rtd 1
 */
int16_t MC_temperatures_2::get_rtd_1_temperature() {
    return message.rtd_1_temperature;
}

/**
 * To get the rtd_2_temperature
 * @return rtd 2
 */
int16_t MC_temperatures_2::get_rtd_2_temperature() {
    return message.rtd_2_temperature;
}

/**
 * To get the rtd_3_temperature
 * @return rtd 3
 */
int16_t MC_temperatures_2::get_rtd_3_temperature() {
    return message.rtd_3_temperature;
}

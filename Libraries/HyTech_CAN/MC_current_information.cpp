/*
 * MC_current_information.cpp - CAN message parser: RMS Motor Controller voltage information message
 * Created by Nathan Cheek, November 20, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEX ID: A6
 * DESCR: MC Current Information
 * MACRO: ID_MC_CURRENT_INFORMATION
 * STRUCT: CAN_message_mc_current_information_t 
 * CLASS: MC_current_information
 * DATA:
 *      phase_a_current [0:1]
 *      phase_b_current [2:3]
 *      phase_c_current [4:5]
 *      dc_bus_current  [6:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_current_information
 */
MC_current_information::MC_current_information() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_current_information::MC_current_information(uint8_t buf[8]) {
    memcpy(&(message.phase_a_current), &buf[0], sizeof(int16_t));
    memcpy(&(message.phase_b_current), &buf[2], sizeof(int16_t));
    memcpy(&(message.phase_c_current), &buf[4], sizeof(int16_t));
    memcpy(&(message.dc_bus_current), &buf[6], sizeof(int16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_current_information::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.phase_a_current), sizeof(int16_t));
    memcpy(&buf[2], &(message.phase_b_current), sizeof(int16_t));
    memcpy(&buf[4], &(message.phase_c_current), sizeof(int16_t));
    memcpy(&buf[6], &(message.dc_bus_current), sizeof(int16_t));
}

/**
 * To get the phase_a_current
 * @return phase a current
 */
int16_t MC_current_information::get_phase_a_current() {
    return message.phase_a_current;
}

/**
 * To get the phase_b_current
 * @return phase b current
 */
int16_t MC_current_information::get_phase_b_current() {
    return message.phase_b_current;
}

/**
 * To get the phase_c_current
 * @return phase c current
 */
int16_t MC_current_information::get_phase_c_current() {
    return message.phase_c_current;
}

/**
 * To get the dc_bus_current
 * @return dc bus current
 */
int16_t MC_current_information::get_dc_bus_current() {
    return message.dc_bus_current;
}

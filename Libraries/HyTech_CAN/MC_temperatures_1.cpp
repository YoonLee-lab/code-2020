/*
 * MC_temperatures_1.cpp - CAN message parser: RMS Motor Controller temperatures 1 message
 * Created by Nathan Cheek, November 22, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEX ID: A0
 * DESCR: Motor Controller Temperatures #1
 * MACRO: ID_MC_TEMPERATURES_1
 * STRUCT: CAN_message_mc_temperatures_1_t 
 * CLASS: MC_temperatures_1
 * DATA:
 *      module_a_temperature [0:1] int16
 *      module_b_temperature [2:3] int16
 *      module_c_temperature [4:5] int16
 *      gate_driver_board_temperature [6:7] int16
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_temperatures_1
 */
MC_temperatures_1::MC_temperatures_1() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_temperatures_1::MC_temperatures_1(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.module_a_temperature), &buf[0], sizeof(int16_t));
    memcpy(&(message.module_b_temperature), &buf[2], sizeof(int16_t));
    memcpy(&(message.module_c_temperature), &buf[4], sizeof(int16_t));
    memcpy(&(message.gate_driver_board_temperature), &buf[6], sizeof(int16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_temperatures_1::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.module_a_temperature), sizeof(int16_t));
    memcpy(&buf[2], &(message.module_b_temperature), sizeof(int16_t));
    memcpy(&buf[4], &(message.module_c_temperature), sizeof(int16_t));
    memcpy(&buf[6], &(message.gate_driver_board_temperature), sizeof(int16_t));
}

/**
 * To get the module_a_temperature
 * @return temperature a
 */
int16_t MC_temperatures_1::get_module_a_temperature() {
    return message.module_a_temperature;
}

/**
 * To get the module_b_temperature
 * @return temperature b
 */
int16_t MC_temperatures_1::get_module_b_temperature() {
    return message.module_b_temperature;
}

/**
 * To get the module_c_temperature
 * @return temperature c
 */
int16_t MC_temperatures_1::get_module_c_temperature() {
    return message.module_c_temperature;
}

/**
 * To get the gate_driver_board_temperature
 * @return gate driver board temperature
 */
int16_t MC_temperatures_1::get_gate_driver_board_temperature() {
    return message.gate_driver_board_temperature;
}

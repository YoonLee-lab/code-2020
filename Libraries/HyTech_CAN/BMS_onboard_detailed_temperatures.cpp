/**
 * BMS_onboard_detailed_temperatures.cpp - CAN message parser: Battery Management System PCB onboard detailed temperatures message
 * Created by Yvonne Yeh, February 8, 2018.
 * Documentation by Meghavarnika Budati, February 2, 2020.
 * 
 * HEXID: D6
 * DESCR: BMS Onboard Detailed Temps
 * MACRO: ID_BMS_ONBOARD_DETAILED_TEMPERATURES
 * STRUCT: CAN_message_bms_onboard_detailed_temperatures_t 
 * CLASS: BMS_onboard_detailed_temperatures
 * DATA:
 *      ic_id         [0:1]
 *      temperature_0 [2:3]
 *      temperature_1 [4:5]
 *
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_temperatures_3
 */
BMS_onboard_detailed_temperatures::BMS_onboard_detailed_temperatures() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
BMS_onboard_detailed_temperatures::BMS_onboard_detailed_temperatures(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from a buffer
 * @param buf: buffer to load data from
 */
void BMS_onboard_detailed_temperatures::load(uint8_t buf[8]) {
    memcpy(&(message.ic_id), &buf[0], sizeof(uint8_t));
    memcpy(&(message.temperature_0), &buf[1], sizeof(int16_t));
    memcpy(&(message.temperature_1), &buf[3], sizeof(int16_t));
}

/**
 * Constructor, loading in the data from variables
 * @param ic_id: integrated circuit?
 * @param temperature_0: temperature 0
 * @param temperature_1: temperature 1
 */
BMS_onboard_detailed_temperatures::BMS_onboard_detailed_temperatures(uint8_t ic_id, int16_t temperature_0, int16_t temperature_1) {
    message = {};
    set_ic_id(ic_id);
    set_temperature_0(temperature_0);
    set_temperature_1(temperature_1);
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void BMS_onboard_detailed_temperatures::write(uint8_t buf[]) {
    memcpy(&buf[0], &(message.ic_id), sizeof(uint8_t));
    memcpy(&buf[1], &(message.temperature_0), sizeof(int16_t));
    memcpy(&buf[3], &(message.temperature_1), sizeof(int16_t));
}

/**
 * Getter functions
 */
uint8_t BMS_onboard_detailed_temperatures::get_ic_id() {
    return message.ic_id;
}

int16_t BMS_onboard_detailed_temperatures::get_temperature_0() {
    return message.temperature_0;
}

int16_t BMS_onboard_detailed_temperatures::get_temperature_1() {
    return message.temperature_1;
}

int16_t BMS_onboard_detailed_temperatures::get_temperature(uint8_t temperature_id) {
    if (temperature_id == 0) {
        return message.temperature_0;
    } else if (temperature_id == 1) {
        return message.temperature_1;
    } 
    return 0;
}

/**
 * Setter functions
 */
void BMS_onboard_detailed_temperatures::set_ic_id(uint8_t ic_id) {
    message.ic_id = ic_id;
}

void BMS_onboard_detailed_temperatures::set_temperature_0(int16_t temperature_0) {
    message.temperature_0 = temperature_0;
}

void BMS_onboard_detailed_temperatures::set_temperature_1(int16_t temperature_1) {
    message.temperature_1 = temperature_1;
}

void BMS_onboard_detailed_temperatures::set_temperature(uint8_t temperature_id, int16_t temperature) {
    if (temperature_id == 0) {
        message.temperature_0 = temperature;
    } else if (temperature_id == 1) {
        message.temperature_1 = temperature;
    }
}
/**
 * BMS_voltages.cpp - CAN message parser: Battery Management System voltages message
 * Created by Shrivathsav Seshan, January 10, 2017.
 * Documentation by Meghavarnika Budati, February 2, 2020.
 * 
 * HEXID: D7
 * DESCR: BMS Voltages
 * MACRO: BMS_VOLT, previously: ID_BMS_VOLTAGES
 * STRUCT: BMSVolt_t, previously: CAN_message_bms_voltages_t 
 * CLASS: BMS_voltages
 * DATA:
 *      average_voltage [0:1]
 *      low_voltage     [2:3]
 *      high_voltage    [4:5]
 *      total_voltage   [6:7]
 *
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for BMS_voltages
 */
BMS_voltages::BMS_voltages() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
BMS_voltages::BMS_voltages(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void BMS_voltages::load(uint8_t buf[8]) {
    memcpy(&(message.average_voltage), &buf[0], sizeof(uint16_t));
    memcpy(&(message.low_voltage), &buf[2], sizeof(uint16_t));
    memcpy(&(message.high_voltage), &buf[4], sizeof(uint16_t));
    memcpy(&(message.total_voltage), &buf[6], sizeof(uint16_t));
}

/**
 * Constructor, loading in the data from variables
 * @param average_voltage: average voltage
 * @param low_voltage: low voltage
 * @param high_voltage: high voltage
 * @param total_voltage: total voltage
 */
BMS_voltages::BMS_voltages(uint16_t average_voltage, uint16_t low_voltage, uint16_t high_voltage, uint16_t total_voltage) {
    message = {};
    message.average_voltage = average_voltage;
    message.low_voltage = low_voltage;
    message.high_voltage = high_voltage;
    message.total_voltage = total_voltage;
}

/**
 * Populates the specified byte array using the data stored in this object.
 * @param buf: buffer to load data from
 */
void BMS_voltages::write(uint8_t buf[]) {
    memcpy(&buf[0], &(message.average_voltage), sizeof(uint16_t));
    memcpy(&buf[2], &(message.low_voltage), sizeof(uint16_t));
    memcpy(&buf[4], &(message.high_voltage), sizeof(uint16_t));
    memcpy(&buf[6], &(message.total_voltage), sizeof(uint16_t));
}

/**
 * Getter functions for the variables
 */

uint16_t BMS_voltages::get_average() {
    return message.average_voltage;
}

uint16_t BMS_voltages::get_low() {
    return message.low_voltage;
}

uint16_t BMS_voltages::get_high() {
    return message.high_voltage;
}

uint16_t BMS_voltages::get_total() {
    return message.total_voltage;
}

/**
 * Setter functions for the variables
 */

void BMS_voltages::set_average(uint16_t average_voltage) {
    message.average_voltage = average_voltage;
}

void BMS_voltages::set_low(uint16_t low_voltage) {
    message.low_voltage = low_voltage;
}

void BMS_voltages::set_high(uint16_t high_voltage) {
    message.high_voltage = high_voltage;
}

void BMS_voltages::set_total(uint16_t total_voltage) {
    message.total_voltage = total_voltage;
}

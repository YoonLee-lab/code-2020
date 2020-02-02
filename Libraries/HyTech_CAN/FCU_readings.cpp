/**
 * FCU_readings.cpp - CAN message parser: Front Control Unit readings message
 * Created by Charith (Karvin) Dassanayake, March 10, 2017.
 * Documentation by Meghavarnika Budati, February 2, 2020.
 * 
 * HEXID: D3
 * MACRO: ID_FCU_READINGS
 * STRUCT: CAN_message_fcu_readings_t 
 * CLASS: FCU_readings
 * DATA:
 *      accelerator_pedal_raw_1 [0:1]
 *      accelerator_pedal_raw_2 [2:3]
 *      brake_pedal_raw         [4:5]
 *      temperature             [5:6]
 *
 */
#include "HyTech_CAN.h"

/*  Blank Constructor for FCU_readings
 *
 * Used to initialize instance of FCU_readings with no data
 */

FCU_readings::FCU_readings() {
    message = {};
}

/**
 * Constructor for FCU_readings using a buffer
 *
 * Used to initialize instance of FCU_readings with data
 * that's in an 8xbyte array (typically msg.buf)
 *
 * @param buf: Pass in buffer you are trying to initialize data from
 */

FCU_readings::FCU_readings(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from a buffer
 * @param buf: the buffer to load data from
 */
void FCU_readings::load(uint8_t buf[8]) {
    message = {};

    memcpy(&(message.accelerator_pedal_raw_1), &buf[0], sizeof(uint16_t));
    memcpy(&(message.accelerator_pedal_raw_2), &buf[2], sizeof(uint16_t));
    memcpy(&(message.brake_pedal_raw), &buf[4], sizeof(uint16_t));
    memcpy(&(message.temperature), &buf[6], sizeof(int16_t));
}

/**
 * Constructor for FCU_readings
 *
 * Used to copy data from msg variable in
 * microcontroller code to instance variable
 *
 * @param accelerator_pedal_raw_1: (uint16_t)
 *     - Accelerator Pedal Raw Value 1
 *     - Raw voltage readings from accelerator pedal sensor 1
 * @param accelerator_pedal_raw_2: (uint16_t)
 *     - Accelerator Pedal Raw Value 2
 *     - Raw voltage readings from accelerator pedal sensor 2
 * @param brake_pedal_raw: (uint16_t) 
 *     - Brake Pedal Raw Value
 *     - Raw voltage readings from brake pedal sensor
 * @param temperature: (int16_t) 
 *     - Temperature reading from temperature sensor
 *     - Temperature calculated in Celsius * 0.1
 */

FCU_readings::FCU_readings(uint16_t accelerator_pedal_raw_1, uint16_t accelerator_pedal_raw_2, uint16_t brake_pedal_raw, int16_t temperature) {
    set_accelerator_pedal_raw_1(accelerator_pedal_raw_1);
    set_accelerator_pedal_raw_2(accelerator_pedal_raw_2);
    set_brake_pedal_raw(brake_pedal_raw);
    set_temperature(temperature);
}

/**
 * Write to buffer
 *
 * Used to copy data from instance of this class
 * to msg variable in microcontroller code
 *
 * @param buf: Pass in buffer you are trying to modify
 * Example: curFCU_readings.write(msg.buf);
 */

void FCU_readings::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.accelerator_pedal_raw_1), sizeof(uint16_t));
    memcpy(&buf[2], &(message.accelerator_pedal_raw_2), sizeof(uint16_t));
    memcpy(&buf[4], &(message.brake_pedal_raw), sizeof(uint16_t));
    memcpy(&buf[6], &(message.temperature), sizeof(int16_t));
}

/* Get functions
 *
 * Used to retrieve values stored in this CAN_message_t
 */

uint16_t FCU_readings::get_accelerator_pedal_raw_1() {
    return message.accelerator_pedal_raw_1;
}

uint16_t FCU_readings::get_accelerator_pedal_raw_2() {
    return message.accelerator_pedal_raw_2;
}

uint16_t FCU_readings::get_brake_pedal_raw() {
    return message.brake_pedal_raw;
}

int16_t FCU_readings::get_temperature() {
    return message.temperature;
}

/**
 * Set functions
 *
 * Used to replace values in this CAN_message_t
 *
 * @param (uint16_t) - Variable to replace old data
 */

void FCU_readings::set_accelerator_pedal_raw_1(uint16_t accelerator_pedal_raw_1) {
    message.accelerator_pedal_raw_1 = accelerator_pedal_raw_1;
}

void FCU_readings::set_accelerator_pedal_raw_2(uint16_t accelerator_pedal_raw_2) {
    message.accelerator_pedal_raw_2 = accelerator_pedal_raw_2;
}

void FCU_readings::set_brake_pedal_raw(uint16_t brake_pedal_raw) {
    message.brake_pedal_raw = brake_pedal_raw;
}

void FCU_readings::set_temperature(int16_t temperature) {
    message.temperature = temperature;
}

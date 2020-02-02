#include "HyTech_CAN.h"

/**
 * Documentation by Meghavarnika Budati, February 2, 2020.
 * 
 * HEXID: CC
 * DESCR: GLV Current Readings
 * MACRO: ID_GLV_CURRENT_READINGS
 * STRUCT: CAN_message_glv_current_readings_t 
 * CLASS: GLV_current_readings
 * DATA:
 *      ecu_current_value     [0:2]
 *      cooling_current_value [3:4]
 *
 */

//Constructors

/**
 * Constructor, defining an empty message for MC_temperatures_3
 */
GLV_current_readings::GLV_current_readings(){
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
GLV_current_readings::GLV_current_readings(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void GLV_current_readings::load(uint8_t buf[8]) {
    message = {};

    memcpy(&(message.ecu_current_value), &buf[0], sizeof(uint16_t));
    memcpy(&(message.cooling_current_value), &buf[2], sizeof(uint16_t));
}

/**
 * Constructor, loading in the data from variables
 * @param ecu_current_value: ecu current value 
 * @param cooling_current_value: cooling current value 
 */
GLV_current_readings::GLV_current_readings(uint16_t ecu_current_value, uint16_t cooling_current_value) {
    set_ecu_current_value(ecu_current_value);
    set_cooling_current_value(cooling_current_value);
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void GLV_current_readings::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.ecu_current_value), sizeof(uint16_t));
    memcpy(&buf[2], &(message.cooling_current_value), sizeof(uint16_t));
}

//Get Functions

uint16_t GLV_current_readings::get_ecu_current_value() {
    return message.ecu_current_value;
}

uint16_t GLV_current_readings::get_cooling_current_value() {
    return message.cooling_current_value;
}

// Set functions

void GLV_current_readings::set_ecu_current_value(uint16_t ecu_current_value) {
    message.ecu_current_value = ecu_current_value;
}

void GLV_current_readings::set_cooling_current_value(uint16_t cooling_current_value) {
    message.cooling_current_value = cooling_current_value;
}

/**
 * FCU_accelerometer_values.cpp - retrieving and packaging accelerometer values via CAN
 * Created by Soohyun Kim, July 12, 2018.
 * Documentation by Meghavarnika Budati, February 2, 2020.
 * 
 * HEXID: DF
 * MACRO: FCU_ACCEL, previously: ID_FCU_ACCELEROMETER
 * STRUCT: FCUAccel_t, previously: CAN_message_fcu_accelerometer_values_t 
 * CLASS: FCU_accelerometer_values
 * DATA:
 *      XValue_x100  [0:1]
 *      YValue_x100  [2:3]
 *      ZValue_x100  [4:5]
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for FCU_accelerometer_values
 */
FCU_accelerometer_values::FCU_accelerometer_values() {
   message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
FCU_accelerometer_values::FCU_accelerometer_values(uint8_t buf[8]) {
   load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void FCU_accelerometer_values::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.XValue_x100), &buf[0], sizeof(uint8_t));
    memcpy(&(message.YValue_x100), &buf[2], sizeof(uint8_t));
    memcpy(&(message.ZValue_x100), &buf[4], sizeof(uint8_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void FCU_accelerometer_values::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.XValue_x100), sizeof(uint8_t));
    memcpy(&buf[2], &(message.YValue_x100), sizeof(uint8_t));
    memcpy(&buf[4], &(message.ZValue_x100), sizeof(uint8_t));
}

/**
 * Getter functions
 */
uint8_t FCU_accelerometer_values::get_x() {
   return message.XValue_x100;
}

uint8_t FCU_accelerometer_values::get_y() {
   return message.YValue_x100;
}

uint8_t FCU_accelerometer_values::get_z() {
   return message.ZValue_x100;
}

/**
 * Set the values of x, y and z values
 * @param x: XValue_x100
 * @param y: YValue_x100
 * @param z: ZValue_x100
 */
void FCU_accelerometer_values::set_values(uint8_t x, uint8_t y, uint8_t z) {
    message = {};
    message.XValue_x100 = x;
    message.YValue_x100 = y;
    message.ZValue_x100 = z;
}
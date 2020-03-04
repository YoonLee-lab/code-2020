/**
 * MCU_GPS_readings_alpha.cpp - CAN message parser: MCU_GPS Alpha message
 * Created by Nathan Cheek, May 19, 2019.
 * Documentation by Meghavarnika Budati, February 2, 2020.
 * 
 * HEXID: E7
 * MACRO: MCU_GPS_READ_ALPHA, previously: ID_ECU_GPS_READINGS_ALPHA
 * STRUCT: MCUGpsAlpha_t, previously: CAN_message_mcu_gps_readings_alpha_t 
 * CLASS: MCU_GPS_readings_alpha
 * DATA:
 *      latitude  [0:3]
 *      longitude [4:7]
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MCU_GPS_readings_alpha
 */
MCU_GPS_readings_alpha::MCU_GPS_readings_alpha() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MCU_GPS_readings_alpha::MCU_GPS_readings_alpha(uint8_t buf[]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MCU_GPS_readings_alpha::load(uint8_t buf[]) {
    message = {};
    memcpy(&(message.latitude), &buf[0], sizeof(int32_t));
    memcpy(&(message.longitude), &buf[4], sizeof(int32_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MCU_GPS_readings_alpha::write(uint8_t buf[]) {
    memcpy(&buf[0], &(message.latitude), sizeof(int32_t));
    memcpy(&buf[4], &(message.longitude), sizeof(int32_t));
}

/**
 * Getter functions
 */
int32_t MCU_GPS_readings_alpha::get_latitude() {
    return message.latitude;
}

int32_t MCU_GPS_readings_alpha::get_longitude() {
    return message.longitude;
}

/**
 * Setter functions
 */
void MCU_GPS_readings_alpha::set_latitude(int32_t latitude) {
    message.latitude = latitude;
}

void MCU_GPS_readings_alpha::set_longitude(int32_t longitude) {
    message.longitude = longitude;
}

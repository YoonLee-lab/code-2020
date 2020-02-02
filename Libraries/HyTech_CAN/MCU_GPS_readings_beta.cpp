/**
 * MCU_GPS_readings_beta.cpp - CAN message parser: MCU_GPS Beta message
 * Created by Nathan Cheek, May 19, 2019.
 * Documentation by Meghavarnika Budati, February 2, 2020.
 * 
 * HEXID: E8
 * DESCR: Motor Controller Temperatures #1
 * MACRO: ID_ECU_GPS_READINGS_BETA
 * STRUCT: CAN_message_mcu_gps_readings_beta_t 
 * CLASS: MCU_GPS_readings_beta
 * DATA:
 *      altitude [0:3] int16
 *      speed    [4:7] int16
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_temperatures_1
 */
MCU_GPS_readings_beta::MCU_GPS_readings_beta() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MCU_GPS_readings_beta::MCU_GPS_readings_beta(uint8_t buf[]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MCU_GPS_readings_beta::load(uint8_t buf[]) {
    message = {};
    memcpy(&(message.altitude), &buf[0], sizeof(int32_t));
    memcpy(&(message.speed), &buf[4], sizeof(int32_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MCU_GPS_readings_beta::write(uint8_t buf[]) {
    memcpy(&buf[0], &(message.altitude), sizeof(int32_t));
    memcpy(&buf[4], &(message.speed), sizeof(int32_t));
}

/**
 * Getter functions
 */
int32_t MCU_GPS_readings_beta::get_altitude() {
    return message.altitude;
}

int32_t MCU_GPS_readings_beta::get_speed() {
    return message.speed;
}

/**
 * Setter functions
 */
void MCU_GPS_readings_beta::set_altitude(int32_t altitude) {
    message.altitude = altitude;
}

void MCU_GPS_readings_beta::set_speed(int32_t speed) {
    message.speed = speed;
}

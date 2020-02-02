/*
 * MC_firmware_information.cpp - CAN message parser: RMS Motor Controller firmware information message
 * Created by Nathan Cheek, November 23, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020. WIP
 * 
 * HEX ID: AE
 * DESCR: MC Firmware Information
 * MACRO: ID_MC_FIRMWARE_INFORMATION
 * STRUCT: CAN_message_mc_firmware_information_t 
 * CLASS: MC_firmware_information
 * DATA:
 *      eeprom_version_project_code [0:1]
 *      software_version            [2:3]
 *      date_code_mmdd              [4:5]
 *      date_code_yyyy              [6:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_fault_codes
 */
MC_firmware_information::MC_firmware_information() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_firmware_information::MC_firmware_information(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_firmware_information::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.eeprom_version_project_code), &buf[0], sizeof(uint16_t));
    memcpy(&(message.software_version), &buf[2], sizeof(uint16_t));
    memcpy(&(message.date_code_mmdd), &buf[4], sizeof(uint16_t));
    memcpy(&(message.date_code_yyyy), &buf[6], sizeof(uint16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_firmware_information::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.eeprom_version_project_code), sizeof(uint16_t));
    memcpy(&buf[2], &(message.software_version), sizeof(uint16_t));
    memcpy(&buf[4], &(message.date_code_mmdd), sizeof(uint16_t));
    memcpy(&buf[6], &(message.date_code_yyyy), sizeof(uint16_t));
}

/**
 * To get the eeprom_version_project_code
 * @return version
 */
uint16_t MC_firmware_information::get_eeprom_version_project_code() {
    return message.eeprom_version_project_code;
}

/**
 * To get the software_version
 * @return version of software 
 */
uint16_t MC_firmware_information::get_software_version() {
    return message.software_version;
}

/**
 * To get the date_code_mmdd
 * @return month and date
 */
uint16_t MC_firmware_information::get_date_code_mmdd() {
    return message.date_code_mmdd;
}

/**
 * To get the date_code_yyyy
 * @return year
 */
uint16_t MC_firmware_information::get_date_code_yyyy() {
    return message.date_code_yyyy;
}

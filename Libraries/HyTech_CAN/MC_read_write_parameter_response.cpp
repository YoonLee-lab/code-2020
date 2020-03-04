/**
 * MC_read_write_parameter_response.cpp - CAN message parser: RMS Motor Controller read / write parameter response message - response from PM
 * Created by Nathan Cheek, November 22, 2016.
  * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: C2
 * DESCR: MC Read/Write Parameter Response
 * MACRO: MC_RW_PARAM_RESP, previously: ID_MC_READ_WRITE_PARAMETER_RESPONSE
 * STRUCT: MCRWParamResp_t, previously: CAN_message_mc_read_write_parameter_response_t 
 * CLASS: MC_read_write_parameter_response
 * DATA:
 *      parameter_address [0:1]
 *      write_success     [2:3]
 *      reserved1         [4:5]
 *      data              [6:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_read_write_parameter_response
 */
MC_read_write_parameter_response::MC_read_write_parameter_response() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_read_write_parameter_response::MC_read_write_parameter_response(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_read_write_parameter_response::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.parameter_address), &buf[0], sizeof(uint16_t));
    memcpy(&(message.write_success), &buf[2], sizeof(bool));
    memcpy(&(message.reserved1), &buf[3], sizeof(uint8_t));
    memcpy(&(message.data), &buf[4], sizeof(int32_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_read_write_parameter_response::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.parameter_address), sizeof(uint16_t));
    memcpy(&buf[2], &(message.write_success), sizeof(bool));
    memcpy(&buf[3], &(message.reserved1), sizeof(uint8_t));
    memcpy(&buf[4], &(message.data), sizeof(int32_t));
}

/**
 * To get the parameter_address
 * @return parameter address
 */
uint16_t MC_read_write_parameter_response::get_parameter_address() {
    return message.parameter_address;
}

/**
 * To get the write_success
 * @return whether it's a success
 */
bool MC_read_write_parameter_response::get_write_success() {
    return message.write_success;
}

/**
 * To get the data
 * @return data
 */
uint32_t MC_read_write_parameter_response::get_data() {
    return message.data;
}

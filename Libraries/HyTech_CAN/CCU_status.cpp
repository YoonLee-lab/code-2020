/**
 * CCU_status.cpp - CAN message parser: Charger Control Unit status message
 * Created by Nathan Cheek, December 17, 2017.
 * Documentation by Meghavarnika Budati, February 2, 2020.
 * 
 * HEXID: DD
 * DESCR: Charge Control Unit Status
 * MACRO: CCU_STAT, previously: ID_CCU_STATUS
 * STRUCT: CCUStat_t, previously: CAN_message_ccu_status_t 
 * CLASS: CCU_status
 * DATA:
 *      charger_enabled [0]
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for CCU_status
 */
CCU_status::CCU_status() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
CCU_status::CCU_status(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void CCU_status::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.charger_enabled), &buf[0], sizeof(uint8_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void CCU_status::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.charger_enabled), sizeof(uint8_t));
}

/**
 * To get the charger_enabled
 * @return whether charger is enabled
 */
bool CCU_status::get_charger_enabled() {
    return message.charger_enabled & 0x1;
}

/**
 * To set charger_enabled
 */
void CCU_status::set_charger_enabled(bool charger_enabled) {
    message.charger_enabled = charger_enabled & 0x1;
}

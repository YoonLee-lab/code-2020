/**
 *BMS_coulomb_counts - CAN message parser: Battery Management System coulomb count message
 * Created by Shaan Dhawan, March 26, 2019.
 * Documentation by Meghavarnika Budati, February 2, 2020.
 * 
 * HEXID: E2
 * DESCR: BMS Coulomb Counts
 * MACRO: ID_BMS_COULOMB_COUNTS
 * STRUCT: CAN_message_bms_coulomb_counts_t 
 * CLASS: BMS_coulomb_counts
 * DATA:
 *      total_charge     [0:3]
 *      total_discharge  [4:7]
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_temperatures_1
 */
BMS_coulomb_counts::BMS_coulomb_counts() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
BMS_coulomb_counts::BMS_coulomb_counts(uint8_t buf[]) {
    load(buf);
}

/**
 * Constructor, loading in the data from variables
 * @param total_charge: total charge 
 * @param total_discharge: total discharge
 */
BMS_coulomb_counts::BMS_coulomb_counts(uint32_t total_charge, uint32_t total_discharge) {
    set_total_charge(total_charge);
    set_total_discharge(total_discharge);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void BMS_coulomb_counts::load(uint8_t buf[]) {
    message = {};
    memcpy(&(message.total_charge), &buf[0], sizeof(uint32_t));
    memcpy(&(message.total_discharge), &buf[4], sizeof(uint32_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void BMS_coulomb_counts::write(uint8_t buf[]) {
    memcpy(&buf[0], &(message.total_charge), sizeof(uint32_t));
    memcpy(&buf[4], &(message.total_discharge), sizeof(uint32_t));
}

/**
 * Getter functions
 */
uint32_t BMS_coulomb_counts::get_total_charge() {
    return message.total_charge;
}

uint32_t BMS_coulomb_counts::get_total_discharge() {
    return message.total_discharge;
}

/**
 * Setter functions
 */
void BMS_coulomb_counts::set_total_charge(uint32_t total_charge) {
    message.total_charge = total_charge;
}

void BMS_coulomb_counts::set_total_discharge(uint32_t total_discharge) {
    message.total_discharge = total_discharge;
}


/**
 * MC_voltage_information.cpp - CAN message parser: RMS Motor Controller voltage information message
 * Created by Nathan Cheek, November 20, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: A7
 * DESCR: MC Voltage Information
 * MACRO: MC_VOLT_INFO, previously: ID_MC_VOLTAGE_INFORMATION
 * STRUCT: MCVoltInfo_t, previously: CAN_message_mc_voltage_information_t 
 * CLASS: MC_voltage_information
 * DATA:
 *      dc_bus_voltage   [0:1]
 *      output_voltage   [2:3]
 *      phase_ab_voltage [4:5]
 *      phase_bc_voltage [6:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_voltage_information
 */
MC_voltage_information::MC_voltage_information() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_voltage_information::MC_voltage_information(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_voltage_information::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.dc_bus_voltage), &buf[0], sizeof(int16_t));
    memcpy(&(message.output_voltage), &buf[2], sizeof(int16_t));
    memcpy(&(message.phase_ab_voltage), &buf[4], sizeof(int16_t));
    memcpy(&(message.phase_bc_voltage), &buf[6], sizeof(int16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_voltage_information::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.dc_bus_voltage), sizeof(int16_t));
    memcpy(&buf[2], &(message.output_voltage), sizeof(int16_t));
    memcpy(&buf[4], &(message.phase_ab_voltage), sizeof(int16_t));
    memcpy(&buf[6], &(message.phase_bc_voltage), sizeof(int16_t));
}

/**
 * To get the dc_bus_voltage
 * @return dc bus voltage
 */
int16_t MC_voltage_information::get_dc_bus_voltage() {
    return message.dc_bus_voltage;
}

/**
 * To get the output_voltage
 * @return output voltage
 */
int16_t MC_voltage_information::get_output_voltage() {
    return message.output_voltage;
}

/**
 * To get the phase_ab_voltage
 * @return phase ab voltage
 */
int16_t MC_voltage_information::get_phase_ab_voltage() {
    return message.phase_ab_voltage;
}

/**
 * To get the phase_bc_voltage
 * @return phase bc voltage
 */
int16_t MC_voltage_information::get_phase_bc_voltage() {
    return message.phase_bc_voltage;
}

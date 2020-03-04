/**
 * MC_modulation_index_flux_weakening_output_information.cpp
 * CAN message parser: RMS Motor Controller modulation index & flux weakening output information message
 * Created by Ryan Gallaway, December 1, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: AD
 * DESCR: MC Modulation Index Flux Weakening Output Information
 * MACRO: MC_MOD_INDEX_FLUX_WEAKENING_OUT_INFO,
 *        previously: ID_MC_MODULATION_INDEX_FLUX_WEAKENING_OUTPUT_INFORMATION
 * STRUCT: MCModIndexFluxWeakeningOutInfo_t,
 *        previously: CAN_message_mc_modulation_index_flux_weakening_output_information_t 
 * CLASS: MC_modulation_index_flux_weakening_output_information
 * DATA:
 *      modulation_index      [0:1]
 *      flux_weakening_output [2:3]
 *      id_command            [4:5]
 *      iq_command            [6:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_modulation_index_flux_weakening_output_information
 */
MC_modulation_index_flux_weakening_output_information::
    MC_modulation_index_flux_weakening_output_information() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_modulation_index_flux_weakening_output_information::
    MC_modulation_index_flux_weakening_output_information(uint8_t buf[8]) {
    load(buf);
}

/**
 * Load in the data from buffer
 * @param buf: buffer to load data from
 */
void MC_modulation_index_flux_weakening_output_information::load(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.modulation_index), &buf[0], sizeof(uint16_t));
    memcpy(&(message.flux_weakening_output), &buf[2], sizeof(int16_t));
    memcpy(&(message.id_command), &buf[4], sizeof(int16_t));
    memcpy(&(message.iq_command), &buf[6], sizeof(int16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_modulation_index_flux_weakening_output_information::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.modulation_index), sizeof(uint16_t));
    memcpy(&buf[2], &(message.flux_weakening_output), sizeof(int16_t));
    memcpy(&buf[4], &(message.id_command), sizeof(int16_t));
    memcpy(&buf[6], &(message.iq_command), sizeof(int16_t));
}

/**
 * To get the modulation_index
 * @return vsm state
 */
uint16_t MC_modulation_index_flux_weakening_output_information::get_modulation_index() {
    return message.modulation_index;
}

/**
 * To get the flux_weakening_output
 * @return flux weakening output
 */
int16_t MC_modulation_index_flux_weakening_output_information::get_flux_weakening_output() {
    return message.flux_weakening_output;
}

/**
 * To get the id_command
 * @return ID command
 */
int16_t MC_modulation_index_flux_weakening_output_information::get_id_command() {
    return message.id_command;
}

/**
 * To get the iq_command
 * @return IQ command
 */
int16_t MC_modulation_index_flux_weakening_output_information::get_iq_command() {
    return message.iq_command;
}

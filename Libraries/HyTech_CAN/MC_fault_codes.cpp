/**
 * MC_fault_codes.cpp - CAN message parser: RMS Motor Controller fault codes message
 * Created by Nathan Cheek, November 22, 2016.
 * Documentation by Meghavarnika Budati, January 31, 2020.
 * 
 * HEXID: AB
 * DESCR: MC Fault Codes
 * MACRO: MC_FAULT_CODES, previously: ID_MC_FAULT_CODES
 * STRUCT: MCFaultCodes_t, preivously: CAN_message_mc_fault_codes_t 
 * CLASS: MC_fault_codes 
 * DATA:
 *      post_fault_lo [0:1]
 *      post_fault_hi [2:3]
 *      run_fault_lo  [4:5]
 *      run_fault_hi  [6:7]
 * 
 */

#include "HyTech_CAN.h"

/**
 * Constructor, defining an empty message for MC_fault_codes
 */
MC_fault_codes::MC_fault_codes() {
    message = {};
}

/**
 * Constructor, loading in the data from buffer
 * @param buf: buffer to load data from
 */
MC_fault_codes::MC_fault_codes(uint8_t buf[8]) {
    message = {};
    memcpy(&(message.post_fault_lo), &buf[0], sizeof(int16_t));
    memcpy(&(message.post_fault_hi), &buf[2], sizeof(int16_t));
    memcpy(&(message.run_fault_lo), &buf[4], sizeof(int16_t));
    memcpy(&(message.run_fault_hi), &buf[6], sizeof(int16_t));
}

/**
 * Writing data to the buffer
 * @param buf: buffer to load data into
 */
void MC_fault_codes::write(uint8_t buf[8]) {
    memcpy(&buf[0], &(message.post_fault_lo), sizeof(int16_t));
    memcpy(&buf[2], &(message.post_fault_hi), sizeof(int16_t));
    memcpy(&buf[4], &(message.run_fault_lo), sizeof(int16_t));
    memcpy(&buf[6], &(message.run_fault_hi), sizeof(int16_t));
}

uint16_t MC_fault_codes::get_post_fault_lo() {
    return message.post_fault_lo;
}

uint16_t MC_fault_codes::get_post_fault_hi() {
    return message.post_fault_hi;
}

uint16_t MC_fault_codes::get_run_fault_lo() {
    return message.run_fault_lo;
}

uint16_t MC_fault_codes::get_run_fault_hi() {
    return message.run_fault_hi;
}

bool MC_fault_codes::get_post_lo_hw_gate_desaturation_fault() {
    return message.post_fault_lo & 0x0001;
}

bool MC_fault_codes::get_post_lo_hw_overcurrent_fault() {
    return (message.post_fault_lo & 0x0002) >> 1;
}

bool MC_fault_codes::get_post_lo_accelerator_shorted() {
    return (message.post_fault_lo & 0x0004) >> 2;
}

bool MC_fault_codes::get_post_lo_accelerator_open() {
    return (message.post_fault_lo & 0x0008) >> 3;
}

bool MC_fault_codes::get_post_lo_current_sensor_low() {
    return (message.post_fault_lo & 0x0010) >> 4;
}

bool MC_fault_codes::get_post_lo_current_sensor_high() {
    return (message.post_fault_lo & 0x0020) >> 5;
}

bool MC_fault_codes::get_post_lo_module_temperature_low() {
    return (message.post_fault_lo & 0x0040) >> 6;
}

bool MC_fault_codes::get_post_lo_module_temperature_high() {
    return (message.post_fault_lo & 0x0080) >> 7;
}

bool MC_fault_codes::get_post_lo_ctrl_pcb_temperature_low() {
    return (message.post_fault_lo & 0x0100) >> 8;
}

bool MC_fault_codes::get_post_lo_ctrl_pcb_temperature_high() {
    return (message.post_fault_lo & 0x0200) >> 9;
}

bool MC_fault_codes::get_post_lo_gate_drive_pcb_temperature_low() {
    return (message.post_fault_lo & 0x0400) >> 10;
}

bool MC_fault_codes::get_post_lo_gate_drive_pcb_temperature_high() {
    return (message.post_fault_lo & 0x0800) >> 11;
}

bool MC_fault_codes::get_post_lo_5v_sense_voltage_low() {
    return (message.post_fault_lo & 0x1000) >> 12;
}

bool MC_fault_codes::get_post_lo_5v_sense_voltage_high() {
    return (message.post_fault_lo & 0x2000) >> 13;
}

bool MC_fault_codes::get_post_lo_12v_sense_voltage_low() {
    return (message.post_fault_lo & 0x4000) >> 14;
}

bool MC_fault_codes::get_post_lo_12v_sense_voltage_high() {
    return (message.post_fault_lo & 0x8000) >> 15;
}

bool MC_fault_codes::get_post_hi_25v_sense_voltage_low() {
    return message.post_fault_hi & 0x0001;
}

bool MC_fault_codes::get_post_hi_25v_sense_voltage_high() {
    return (message.post_fault_hi & 0x0002) >> 1;
}

bool MC_fault_codes::get_post_hi_15v_sense_voltage_low() {
    return (message.post_fault_hi & 0x0004) >> 2;
}

bool MC_fault_codes::get_post_hi_15v_sense_voltage_high() {
    return (message.post_fault_hi & 0x0008) >> 3;
}

bool MC_fault_codes::get_post_hi_dc_bus_voltage_high() {
    return (message.post_fault_hi & 0x0010) >> 4;
}

bool MC_fault_codes::get_post_hi_dc_bus_voltage_low() {
    return (message.post_fault_hi & 0x0020) >> 5;
}

bool MC_fault_codes::get_post_hi_precharge_timeout() {
    return (message.post_fault_hi & 0x0040) >> 6;
}

bool MC_fault_codes::get_post_hi_precharge_voltage_failure() {
    return (message.post_fault_hi & 0x0080) >> 7;
}

bool MC_fault_codes::get_post_hi_eeprom_checksum_invalid() {
    return (message.post_fault_hi & 0x0100) >> 8;
}

bool MC_fault_codes::get_post_hi_eeprom_data_out_of_range() {
    return (message.post_fault_hi & 0x0200) >> 9;
}

bool MC_fault_codes::get_post_hi_eeprom_update_required() {
    return (message.post_fault_hi & 0x0400) >> 10;
}

bool MC_fault_codes::get_post_hi_reserved1() {
    return (message.post_fault_hi & 0x0800) >> 11;
}

bool MC_fault_codes::get_post_hi_reserved2() {
    return (message.post_fault_hi & 0x1000) >> 12;
}

bool MC_fault_codes::get_post_hi_reserved3() {
    return (message.post_fault_hi & 0x2000) >> 13;
}

bool MC_fault_codes::get_post_hi_brake_shorted() {
    return (message.post_fault_hi & 0x4000) >> 14;
}

bool MC_fault_codes::get_post_hi_brake_open() {
    return (message.post_fault_hi & 0x8000) >> 15;
}

bool MC_fault_codes::get_run_lo_motor_overspeed_fault() {
    return message.run_fault_lo & 0x0001;
}

bool MC_fault_codes::get_run_lo_overcurrent_fault() {
    return (message.run_fault_lo & 0x0002) >> 1;
}

bool MC_fault_codes::get_run_lo_overvoltage_fault() {
    return (message.run_fault_lo & 0x0004) >> 2;
}

bool MC_fault_codes::get_run_lo_inverter_overtemperature_fault() {
    return (message.run_fault_lo & 0x0008) >> 3;
}

bool MC_fault_codes::get_run_lo_accelerator_input_shorted_fault() {
    return (message.run_fault_lo & 0x0010) >> 4;
}

bool MC_fault_codes::get_run_lo_accelerator_input_open_fault() {
    return (message.run_fault_lo & 0x0020) >> 5;
}

bool MC_fault_codes::get_run_lo_direction_command_fault() {
    return (message.run_fault_lo & 0x0040) >> 6;
}

bool MC_fault_codes::get_run_lo_inverter_response_timeout_fault() {
    return (message.run_fault_lo & 0x0080) >> 7;
}

bool MC_fault_codes::get_run_lo_hardware_gatedesaturation_fault() {
    return (message.run_fault_lo & 0x0100) >> 8;
}

bool MC_fault_codes::get_run_lo_hardware_overcurrent_fault() {
    return (message.run_fault_lo & 0x0200) >> 9;
}

bool MC_fault_codes::get_run_lo_undervoltage_fault() {
    return (message.run_fault_lo & 0x0400) >> 10;
}

bool MC_fault_codes::get_run_lo_can_command_message_lost_fault() {
    return (message.run_fault_lo & 0x0800) >> 11;
}

bool MC_fault_codes::get_run_lo_motor_overtemperature_fault() {
    return (message.run_fault_lo & 0x1000) >> 12;
}

bool MC_fault_codes::get_run_lo_reserved1() {
    return (message.run_fault_lo & 0x2000) >> 13;
}

bool MC_fault_codes::get_run_lo_reserved2() {
    return (message.run_fault_lo & 0x4000) >> 14;
}

bool MC_fault_codes::get_run_lo_reserved3() {
    return (message.run_fault_lo & 0x8000) >> 15;
}

bool MC_fault_codes::get_run_hi_brake_input_shorted_fault() {
    return message.run_fault_hi & 0x0001;
}

bool MC_fault_codes::get_run_hi_brake_input_open_fault() {
    return (message.run_fault_hi & 0x0002) >> 1;
}

bool MC_fault_codes::get_run_hi_module_a_overtemperature_fault() {
    return (message.run_fault_hi & 0x0004) >> 2;
}

bool MC_fault_codes::get_run_hi_module_b_overtemperature_fault() {
    return (message.run_fault_hi & 0x0008) >> 3;
}

bool MC_fault_codes::get_run_hi_module_c_overtemperature_fault() {
    return (message.run_fault_hi & 0x0010) >> 4;
}

bool MC_fault_codes::get_run_hi_pcb_overtemperature_fault() {
    return (message.run_fault_hi & 0x0020) >> 5;
}

bool MC_fault_codes::get_run_hi_gate_drive_board_1_overtemperature_fault() {
    return (message.run_fault_hi & 0x0040) >> 6;
}

bool MC_fault_codes::get_run_hi_gate_drive_board_2_overtemperature_fault() {
    return (message.run_fault_hi & 0x0080) >> 7;
}

bool MC_fault_codes::get_run_hi_gate_drive_board_3_overtemperature_fault() {
    return (message.run_fault_hi & 0x0100) >> 8;
}

bool MC_fault_codes::get_run_hi_current_sensor_fault() {
    return (message.run_fault_hi & 0x0200) >> 9;
}

bool MC_fault_codes::get_run_hi_reserved1() {
    return (message.run_fault_hi & 0x0400) >> 10;
}

bool MC_fault_codes::get_run_hi_reserved2() {
    return (message.run_fault_hi & 0x0800) >> 11;
}

bool MC_fault_codes::get_run_hi_reserved3() {
    return (message.run_fault_hi & 0x1000) >> 12;
}

bool MC_fault_codes::get_run_hi_reserved4() {
    return (message.run_fault_hi & 0x2000) >> 13;
}

bool MC_fault_codes::get_run_hi_resolver_not_connected() {
    return (message.run_fault_hi & 0x4000) >> 14;
}

bool MC_fault_codes::get_run_hi_inverter_discharge_active() {
    return (message.run_fault_hi & 0x8000) >> 15;
}

/*

 * A GENERAL_NOTE: the load functions in these classes take a byte array containing raw CAN buffer data.
 * The data contained in this byte array is used to populate the struct.
 *
 * A GENERAL_NOTE: The write functions in these classes take a byte array that is meant to be populated
 * with the data contained in the object. The byte array can then be used as the raw CAN buffer.
 *
 * TODO: If you make changes here, make sure to update https://hytechracing.me.gatech.edu/wiki/CAN_Data_Formats
 */

/**
 * IMPORTANT:
 * If you're adding more structs/functions to this, please follow the HEX order. I have rearranged the file to be
 * easier to read and add stuff to. Please don't change the order without a good reason. Add comments wherever you
 * believe is necessary. Better more comments than none.
 * 
 * CONVENTION:
 * Add a one-line comment containing the hex ID, name you used in the doc and the macro ID.
 * Do this for both the struct and the class.
 * Add the class/struct in the correct place.
 * 
 * TODO:
 * There is no information on: A8, A9, AF, DC, E1
 * 
 * OTHER:
 * D0 and D2 are obsolete. Check documentation for more info.
 * 
 * --Varnika/Vivacia/Meghavarnika Budati
 */

#ifndef __HYTECH_CAN_H__
#define __HYTECH_CAN_H__

#include <string.h>
#include <stdint.h>

/*
 * ECU state definitions // TODO make these enums?
 */
#define MCU_STATE_TRACTIVE_SYSTEM_NOT_ACTIVE 1
#define MCU_STATE_TRACTIVE_SYSTEM_ACTIVE 2
#define MCU_STATE_ENABLING_INVERTER 3
#define MCU_STATE_WAITING_READY_TO_DRIVE_SOUND 4
#define MCU_STATE_READY_TO_DRIVE 5

#define FCU_STATE_TRACTIVE_SYSTEM_NOT_ACTIVE 1
#define FCU_STATE_TRACTIVE_SYSTEM_ACTIVE 2
#define FCU_STATE_ENABLING_INVERTER 3
#define FCU_STATE_WAITING_READY_TO_DRIVE_SOUND 4
#define FCU_STATE_READY_TO_DRIVE 5

#define BMS_STATE_DISCHARGING 1
#define BMS_STATE_CHARGING 2
#define BMS_STATE_BALANCING 3
#define BMS_STATE_BALANCING_OVERHEATED 4

/**
 * CAN ID definitions
 * Refer to documentation for meaning of abbreviations/
 * general abbreviations used.
 */
#define MC_TEMP_1 0xA0
#define MC_TEMP_2 0xA1
#define MC_TEMP_3 0xA2
#define MC_A_INP_VOLT 0xA3
#define MC_D_INP_STAT 0xA4
#define MC_MOTOR_POS_INFO 0xA5
#define MC_CURR_INFO 0xA6
#define MC_VOLT_INFO 0xA7
#define MC_FLUX_INFO 0xA8               //UNKNOWN
#define MC_INT_INFO 0xA9                //UNKNOWN
#define MC_INT_STATES 0xAA
#define MC_FAULT_CODES 0xAB
#define MC_TOR_TIMER_INFO 0xAC
#define MC_MOD_INDEX_FLUX_WEAKENING_OUT_INFO 0xAD
#define MC_FIRM_INFO 0xAE
#define MC_DIAG_DATA 0xAF               //UNKNOWN
#define MC_COMM_MSG 0xC0
#define MC_RW_PARAM_COMM 0xC1
#define MC_RW_PARAM_RESP 0xC2
#define MCU_STAT 0xC3
#define MCU_PEDAL_READ 0xC4
#define GLV_CURR_READ 0xCC
#define FCU_READ 0xD3
#define BMS_ONB_TEMP 0xD5
#define BMS_ONB_DET_TEMP 0xD6           // TODO rename to bms_detailed_onboard_temperatures when we're not in the middle of a development cycle
#define BMS_VOLT 0xD7
#define BMS_DET_VOLT 0xD8
#define BMS_TEMP 0xD9
#define BMS_DET_TEMP 0xDA
#define BMS_STAT 0xDB
#define FH_WATCHDOG_TEST 0xDC           //UNKNOWN
#define CCU_STAT 0xDD
#define BMS_BAL_STAT 0xDE               // TODO rename to bms_balancing_cells when we're not in the middle of a development cycle
#define FCU_ACCEL 0xDF                  // TODO rename to mcu_accelerometer_readings when we're not in the middle of a development cycle
#define BMS_RW_PARAM_COMM 0xE0          // TODO define this message
#define BMS_PARAM_RESP 0xE1             //UNKNOWN
#define BMS_COUL_COUNTS 0xE2
#define MCU_GPS_READ_ALPHA 0xE7
#define MCU_GPS_READ_BETA 0xE8
#define MCU_GPS_READ_GAMMA 0xE9

// For compatibility with C.
#ifndef __cplusplus
#define bool char
#define true 1
#define false 0
#endif

/*
 * CAN message structs and classes
 */
#pragma pack(push,1)

//A0: Motor Controller Temperatur
typedef struct MCTemp1_t {
    int16_t module_a_temperature;
    int16_t module_b_temperature;
    int16_t module_c_temperature;
    int16_t gate_driver_board_temperature;
} MCTemp1_t;

//A1: Motor Controller Temperatures #2, MC_TEMP_2
typedef struct MCTemp2_t {
    int16_t control_board_temperature;
    int16_t rtd_1_temperature;
    int16_t rtd_2_temperature;
    int16_t rtd_3_temperature;
} MCTemp2_t;

//A2: Motor Controller Temperatures #3, MC_TEMP_3
typedef struct MCTemp3_t {
    int16_t rtd_4_temperature;
    int16_t rtd_5_temperature;
    int16_t motor_temperature;
    int16_t torque_shudder;
} MCTemp3_t;

//A3: MC Analog Inputs Voltages, MC_A_INP_VOLT
typedef struct MCAInpVolt_t {
    int16_t analog_input_1;
    int16_t analog_input_2;
    int16_t analog_input_3;
    int16_t analog_input_4;
} MCAInpVolt_t;

//A4: MC Digital Input Status, MC_D_INP_STAT
typedef struct MCDInpStat_t {
    bool digital_input_1;
    bool digital_input_2;
    bool digital_input_3;
    bool digital_input_4;
    bool digital_input_5;
    bool digital_input_6;
    bool digital_input_7;
    bool digital_input_8;
} MCDInpStat_t;

//A5: MC Motor Position Information, MC_MOTOR_POS_INFO
typedef struct MCMotorPosInfo_t {
    int16_t motor_angle;
    int16_t motor_speed;
    int16_t electrical_output_frequency;
    int16_t delta_resolver_filtered;
} MCMotorPosInfo_t;

//A6: MC Current Information, MC_CURR_INFO
typedef struct MCCurrInfo_t {
    int16_t phase_a_current;
    int16_t phase_b_current;
    int16_t phase_c_current;
    int16_t dc_bus_current;
} MCCurrInfo_t;

//A7: MC Voltage Information, MC_VOLT_INFO
typedef struct MCVoltInfo_t {
    int16_t dc_bus_voltage;
    int16_t output_voltage;
    int16_t phase_ab_voltage;
    int16_t phase_bc_voltage;
} MCVoltInfo_t;

//A8: MC Flux Information, MC_FLUX_INFO
// ???

//A9: MC Internal Voltages, MC_INT_INFO
//????

//AA: MC Internal States, MC_INT_STATES
typedef struct MCIntStates_t {
    uint16_t vsm_state;
    uint8_t inverter_state;
    uint8_t relay_state;
    uint8_t inverter_run_mode_discharge_state;
    uint8_t inverter_command_mode;
    uint8_t inverter_enable;
    uint8_t direction_command;
} MCIntStates_t;

//AB: MC Fault Codes, MC_FAULT_CODES
typedef struct MCFaultCodes_t {
    uint16_t post_fault_lo;
    uint16_t post_fault_hi;
    uint16_t run_fault_lo;
    uint16_t run_fault_hi;
} MCFaultCodes_t;

//AC: MC Torque Timer Information, MC_TOR_TIMER_INFO
typedef struct MCTorTimerInfo_t {
    int16_t commanded_torque;
    int16_t torque_feedback;
    uint32_t power_on_timer;
} MCTorTimerInfo_t;

//AD: MC Modulation Index Flux Weakening Output Information,
//MC_MOD_INDEX_FLUX_WEAKENING_OUT_INFO
typedef struct MCModIndexFluxWeakeningOutInfo_t {
    uint16_t modulation_index; // TODO Signed or Unsigned?
    int16_t flux_weakening_output;
    int16_t id_command;
    int16_t iq_command;
} MCModIndexFluxWeakeningOutInfo_t;

//AE: MC Firmware Information, MC_FIRM_INFO
typedef struct MCFirmInfo_t {
    uint16_t eeprom_version_project_code;
    uint16_t software_version;
    uint16_t date_code_mmdd;
    uint16_t date_code_yyyy;
} MCFirmInfo_t;

//AF: MC Diagnostic Data, MC_DIAG_DATA
// ???

//C0: MC Command Message, MC_COMM_MSG
typedef struct MCCommMsg_t {
    int16_t torque_command;
    int16_t angular_velocity;
    bool direction;
    uint8_t inverter_enable_discharge_enable;
    int16_t commanded_torque_limit;
} MCCommMsg_t;

//C1: MC Read/Write Parameter Command, MC_RW_PARAM_COMM
typedef struct MCRWParamComm_t {
    uint16_t parameter_address;
    bool rw_command;
    uint8_t reserved1;
    uint32_t data;
} MCRWParamComm_t;

//C2: MC Read/Write Parameter Response, MC_RW_PARAM_RESP
typedef struct MCRWParamResp_t {
    uint16_t parameter_address;
    bool write_success;
    uint8_t reserved1;
    uint32_t data;
} MCRWParamResp_t;

//C3: MCU_STAT
typedef struct MCUStat_t {
    uint8_t state;
    uint8_t flags;
    int16_t temperature;
    uint16_t glv_battery_voltage;
} MCUStat_t;

//C4: MCU_PEDAL_READ
typedef struct MCUPedalRead_t {
    uint16_t accelerator_pedal_raw_1;
    uint16_t accelerator_pedal_raw_2;
    uint16_t brake_pedal_raw;
    uint8_t pedal_flags;
    uint8_t torque_map_mode;
} MCUPedalRead_t;

//CC: GLV Current Readings, GLV_CURR_READ
typedef struct GLVCurrRead_t {
	uint16_t ecu_current_value;
	uint16_t cooling_current_value;
} GLVCurrRead_t;

//D3: ID_FCU_READINGS
typedef struct FCURead_t {
	uint16_t accelerator_pedal_raw_1;
	uint16_t accelerator_pedal_raw_2;
	uint16_t brake_pedal_raw;
	int16_t temperature;
} FCURead_t;

//D5: Battery Monitoring System Onboard Temps, BMS_ONB_TEMP
typedef struct BMSOnbTemp_t {
    int16_t average_temperature;
    int16_t low_temperature;
    int16_t high_temperature;
} BMSOnbTemp_t;

//D6: BMS Onboard Detailed Temps, BMS_ONB_DET_TEMP
typedef struct BMSOnbDetTemp_t {
	uint8_t ic_id;
    int16_t temperature_0;
    int16_t temperature_1;
} BMSOnbDetTemp_t;

//D7: BMS Voltages, BMS_VOLT
typedef struct BMSVolt_t {
    uint16_t average_voltage;
    uint16_t low_voltage;
    uint16_t high_voltage;
    uint16_t total_voltage;
} BMSVolt_t;

//D8: BMS Detailed Voltages, BMS_DET_VOLT
typedef struct BMSDetVolt_t {
	uint8_t ic_id_group_id;
    uint16_t voltage_0;
    uint16_t voltage_1;
    uint16_t voltage_2;
} BMSDetVolt_t;

//D9: BMS Temperatures, BMS_TEMP
typedef struct BMSTemp_t {
    int16_t average_temperature;
    int16_t low_temperature;
    int16_t high_temperature;
} BMSTemp_t;

//DA: BMS Detailed Temperatures, BMS_DET_TEMP
typedef struct BMSDetTemp_t {
	uint8_t ic_id;
    int16_t temperature_0;
    int16_t temperature_1;
    int16_t temperature_2;
} BMSDetTemp_t;

//DB: BMS Status, BMS_STAT
typedef struct BMSStat_t {
	uint8_t state;
    uint16_t error_flags;
    int16_t current;
    uint8_t flags;
} BMSStat_t;

//DC: FH Watchdog Test, FH_WATCHDOG_TEST
// ???

//DD: Charge Control Unit Status, CCU_STAT
typedef struct CCUStat_t {
    bool charger_enabled;
} CCUStat_t;

//DE: BMS Balancing Status, BMS_BAL_STAT
typedef struct BMSBalStat_t {
	uint8_t balancing_status[5];
} BMSBalStat_t;

//DF: FCU_ACCEL
typedef struct FCUAccel_t {
   uint8_t XValue_x100;
   uint8_t YValue_x100;
   uint8_t ZValue_x100;
} FCUAccel_t;

//E1: BMS_PARAM_RESP
// ???

//E2: BMS Coulomb Counts, BMS_COUL_COUNTS
typedef struct BMSCoulCounts_t {
    uint32_t total_charge;
    uint32_t total_discharge;
} BMSCoulCounts_t;

//E7: ID_ECU_GPS_READINGS_ALPHA
typedef struct MCUGpsAlpha_t {
    int32_t latitude;
    int32_t longitude;
} MCUGpsAlpha_t;

//E8: ID_ECU_GPS_READINGS_BETA
typedef struct MCUGpsBeta_t {
    int32_t altitude;
    int32_t speed;
} MCUGpsBeta_t;

//E9: MCU_GPS_READ_GAMMA
typedef struct MCUGpsGamma_t {
    uint8_t fix_quality;
    uint8_t satellite_count;
    uint32_t timestamp_seconds;
    uint16_t timestamp_milliseconds;
} MCUGpsGamma_t;

typedef struct telemMsg {
    //bool cobs_flag;
    uint32_t msg_id;
    uint8_t length;
    union {
        /* A0 */MCTemp1_t         mcTemp1;
        /* A1 */MCTemp2_t         mcTemp2;
        /* A2 */MCTemp3_t         mcTemp3;
        /* A3 */MCAInpVolt_t      mcAInpVolt;
        /* A4 */MCDInpStat_t      mcDInpStat;
        /* A5 */MCMotorPosInfo_t  mcMotorPosInfo;
        /* A6 */MCCurrInfo_t      mcCurrInfo;
        /* A7 */MCVoltInfo_t      mcVoltInfo;
        /* AA */MCIntStates_t     mcIntStates;
        /* AB */MCFaultCodes_t    mc_fault_codes;
        /* AC */MCTorTimerInfo_t  mcTorTimerInfo;
        /* AD */MCModIndexFluxWeakeningOutInfo_t
                mcModIndexFluxWeakeningOutInfo;
        /* AE */MCFirmInfo_t      mcFirmInfo;
        /* C0 */MCCommMsg_t       mcCommMsg;
        /* C1 */MCRWParamComm_t   mcRwParamComm;
        /* C2 */MCRWParamResp_t   mcRwParamResp;
        /* C3 */MCUStat_t         mcuStat;
        /* C4 */MCUPedalRead_t    mcuPedalRead;
        /* CC */GLVCurrRead_t     glvCurrRead;
        /* D3 */FCURead_t         fcuRead;
        /* D5 */BMSOnbTemp_t      bmsOnbTemp;
        /* D6 */BMSOnbDetTemp_t   bmsOnbDetTemp;
        /* D7 */BMSVolt_t         bmsVolt;
        /* D8 */BMSDetVolt_t      bmsDetVolt;
        /* D9 */BMSTemp_t         bmsTemp;
        /* DA */BMSDetTemp_t      bmsDetTemp;
        /* DB */BMSStat_t         bmsStat;
        /* DD */CCUStat_t         ccuStat;
        /* DE */BMSBalStat_t      bmsBalStat;
        /* DF */FCUAccel_t        fcuAccel;
        /* E2 */BMSCoulCounts_t   bmsCoulCounts;
        /* E7 */MCUGpsAlpha_t     mcuGpsAlpha;
        /* E8 */MCUGpsBeta_t      mcuGpsBeta;
        /* E9 */MCUGpsGamma_t     mcuGpsGamma;
    } contents;
    uint16_t checksum;
} telemMsg_t;

#pragma pack(pop)

#ifdef __cplusplus

// Relevant classes

//A0: Motor Controller Temperatur
class MC_temperatures_1 {
    public:
        MC_temperatures_1();
        MC_temperatures_1(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int16_t get_module_a_temperature();
        int16_t get_module_b_temperature();
        int16_t get_module_c_temperature();
        int16_t get_gate_driver_board_temperature();
    private:
        MCTemp1_t message;
};

//A1: Motor Controller Temperatures #2, MC_TEMP_2
class MC_temperatures_2 {
    public:
        MC_temperatures_2();
        MC_temperatures_2(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int16_t get_control_board_temperature();
        int16_t get_rtd_1_temperature();
        int16_t get_rtd_2_temperature();
        int16_t get_rtd_3_temperature();
    private:
        MCTemp2_t message;
};

//A2: Motor Controller Temperatures #3, MC_TEMP_3
class MC_temperatures_3 {
    public:
        MC_temperatures_3();
        MC_temperatures_3(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int16_t get_rtd_4_temperature();
        int16_t get_rtd_5_temperature();
        int16_t get_motor_temperature();
        int16_t get_torque_shudder();
    private:
        MCTemp3_t message;
};

//A3: MC Analog Inputs Voltages, MC_A_INP_VOLT
class MC_analog_input_voltages {
    public:
        MC_analog_input_voltages();
        MC_analog_input_voltages(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int16_t get_analog_input_1();
        int16_t get_analog_input_2();
        int16_t get_analog_input_3();
        int16_t get_analog_input_4();
    private:
        MCAInpVolt_t message;
};

//A4: MC Digital Input Status, MC_D_INP_STAT
class MC_digital_input_status {
    public:
        MC_digital_input_status();
        MC_digital_input_status(uint8_t buf[]);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        bool get_digital_input_1();
        bool get_digital_input_2();
        bool get_digital_input_3();
        bool get_digital_input_4();
        bool get_digital_input_5();
        bool get_digital_input_6();
        bool get_digital_input_7();
        bool get_digital_input_8();
    private:
        MCDInpStat_t message;
};

//A5: MC Motor Position Information, MC_MOTOR_POS_INFO
class MC_motor_position_information {
    public:
        MC_motor_position_information();
        MC_motor_position_information(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int16_t get_motor_angle();
        int16_t get_motor_speed();
        int16_t get_electrical_output_frequency();
        int16_t get_delta_resolver_filtered();
    private:
        MCMotorPosInfo_t message;
};

//A6: MC Current Information, MC_CURR_INFO
class MC_current_information {
    public:
        MC_current_information();
        MC_current_information(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int16_t get_phase_a_current();
        int16_t get_phase_b_current();
        int16_t get_phase_c_current();
        int16_t get_dc_bus_current();
    private:
        MCCurrInfo_t message;
};

//A7: MC Voltage Information, MC_VOLT_INFO
class MC_voltage_information {
    public:
        MC_voltage_information();
        MC_voltage_information(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int16_t get_dc_bus_voltage();
        int16_t get_output_voltage();
        int16_t get_phase_ab_voltage();
        int16_t get_phase_bc_voltage();
    private:
        MCVoltInfo_t message;
};

//AA: MC Internal States: MC_INT_STATES
class MC_internal_states {
    public:
        MC_internal_states();
        MC_internal_states(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint8_t get_vsm_state();
        uint8_t get_inverter_state();
        bool get_relay_active_1();
        bool get_relay_active_2();
        bool get_relay_active_3();
        bool get_relay_active_4();
        bool get_relay_active_5();
        bool get_relay_active_6();
        bool get_inverter_run_mode();
        uint8_t get_inverter_active_discharge_state();
        bool get_inverter_command_mode();
        bool get_inverter_enable_state();
        bool get_inverter_enable_lockout();
        bool get_direction_command();
    private:
        MCIntStates_t message;
};

//AB: MC Fault Codes, MC_FAULT_CODES
class MC_fault_codes {
    public:
        MC_fault_codes();
        MC_fault_codes(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint16_t get_post_fault_lo();
        uint16_t get_post_fault_hi();
        uint16_t get_run_fault_lo();
        uint16_t get_run_fault_hi();
        bool get_post_lo_hw_gate_desaturation_fault();
        bool get_post_lo_hw_overcurrent_fault();
        bool get_post_lo_accelerator_shorted();
        bool get_post_lo_accelerator_open();
        bool get_post_lo_current_sensor_low();
        bool get_post_lo_current_sensor_high();
        bool get_post_lo_module_temperature_low();
        bool get_post_lo_module_temperature_high();
        bool get_post_lo_ctrl_pcb_temperature_low();
        bool get_post_lo_ctrl_pcb_temperature_high();
        bool get_post_lo_gate_drive_pcb_temperature_low();
        bool get_post_lo_gate_drive_pcb_temperature_high();
        bool get_post_lo_5v_sense_voltage_low();
        bool get_post_lo_5v_sense_voltage_high();
        bool get_post_lo_12v_sense_voltage_low();
        bool get_post_lo_12v_sense_voltage_high();
        bool get_post_hi_25v_sense_voltage_low();
        bool get_post_hi_25v_sense_voltage_high();
        bool get_post_hi_15v_sense_voltage_low();
        bool get_post_hi_15v_sense_voltage_high();
        bool get_post_hi_dc_bus_voltage_high();
        bool get_post_hi_dc_bus_voltage_low();
        bool get_post_hi_precharge_timeout();
        bool get_post_hi_precharge_voltage_failure();
        bool get_post_hi_eeprom_checksum_invalid();
        bool get_post_hi_eeprom_data_out_of_range();
        bool get_post_hi_eeprom_update_required();
        bool get_post_hi_reserved1(); // TODO delete these?
        bool get_post_hi_reserved2();
        bool get_post_hi_reserved3();
        bool get_post_hi_brake_shorted();
        bool get_post_hi_brake_open();
        bool get_run_lo_motor_overspeed_fault();
        bool get_run_lo_overcurrent_fault();
        bool get_run_lo_overvoltage_fault();
        bool get_run_lo_inverter_overtemperature_fault();
        bool get_run_lo_accelerator_input_shorted_fault();
        bool get_run_lo_accelerator_input_open_fault();
        bool get_run_lo_direction_command_fault();
        bool get_run_lo_inverter_response_timeout_fault();
        bool get_run_lo_hardware_gatedesaturation_fault();
        bool get_run_lo_hardware_overcurrent_fault();
        bool get_run_lo_undervoltage_fault();
        bool get_run_lo_can_command_message_lost_fault();
        bool get_run_lo_motor_overtemperature_fault();
        bool get_run_lo_reserved1(); // TODO delete these?
        bool get_run_lo_reserved2();
        bool get_run_lo_reserved3();
        bool get_run_hi_brake_input_shorted_fault();
        bool get_run_hi_brake_input_open_fault();
        bool get_run_hi_module_a_overtemperature_fault();
        bool get_run_hi_module_b_overtemperature_fault();
        bool get_run_hi_module_c_overtemperature_fault();
        bool get_run_hi_pcb_overtemperature_fault();
        bool get_run_hi_gate_drive_board_1_overtemperature_fault();
        bool get_run_hi_gate_drive_board_2_overtemperature_fault();
        bool get_run_hi_gate_drive_board_3_overtemperature_fault();
        bool get_run_hi_current_sensor_fault();
        bool get_run_hi_reserved1(); // TODO delete these?
        bool get_run_hi_reserved2();
        bool get_run_hi_reserved3();
        bool get_run_hi_reserved4();
        bool get_run_hi_resolver_not_connected();
        bool get_run_hi_inverter_discharge_active();
    private:
        MCFaultCodes_t message;
};

//AC: MC Torque Timer Information, MC_TOR_TIMER_INFO
class MC_torque_timer_information {
    public:
        MC_torque_timer_information();
        MC_torque_timer_information(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int16_t get_commanded_torque();
        int16_t get_torque_feedback();
        uint32_t get_power_on_timer();
    private:
        MCTorTimerInfo_t message;
};

//AD: MC Modulation Index Flux Weakening Output Information,
//MC_MOD_INDEX_FLUX_WEAKENING_OUT_INFO
class MC_modulation_index_flux_weakening_output_information {
    public:
        MC_modulation_index_flux_weakening_output_information();
        MC_modulation_index_flux_weakening_output_information(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint16_t get_modulation_index();
        int16_t get_flux_weakening_output();
        int16_t get_id_command();
        int16_t get_iq_command();
    private:
        MCModIndexFluxWeakeningOutInfo_t message;
};

//AE: MC Firmware Information, MC_FIRM_INFO
class MC_firmware_information {
    public:
        MC_firmware_information();
        MC_firmware_information(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint16_t get_eeprom_version_project_code();
        uint16_t get_software_version();
        uint16_t get_date_code_mmdd();
        uint16_t get_date_code_yyyy();
    private:
        MCFirmInfo_t message;
};

//C0: MC Command Message, MC_COMM_MSG
class MC_command_message {
    public:
        MC_command_message();
        MC_command_message(uint8_t buf[8]);
        MC_command_message(int16_t torque_command, int16_t angular_velocity, bool direction, bool inverter_enable, bool discharge_enable, int16_t commanded_torque_limit);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int16_t get_torque_command();
        int16_t get_angular_velocity();
        bool get_direction();
        bool get_inverter_enable();
        bool get_discharge_enable();
        int16_t get_commanded_torque_limit();
        void set_torque_command(int16_t torque_command);
        void set_angular_velocity(int16_t angular_velocity);
        void set_direction(bool direction);
        void set_inverter_enable(bool inverter_enable);
        void set_discharge_enable(bool discharge_enable);
        void set_commanded_torque_limit(int16_t commanded_torque_limit);
    private:
        MCCommMsg_t message;
};

//C1: MC Read/Write Parameter Command, MC_RW_PARAM_COMM
class MC_read_write_parameter_command {
    public:
        MC_read_write_parameter_command();
        MC_read_write_parameter_command(uint8_t buf[8]);
        MC_read_write_parameter_command(uint16_t parameter_address, bool rw_command, uint32_t data);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint16_t get_parameter_address();
        bool get_rw_command();
        uint32_t get_data();
        void set_parameter_address(uint16_t parameter_address);
        void set_rw_command(bool rw_command);
        void set_data(uint32_t data);
    private:
        MCRWParamComm_t message;
};

//C2: MC Read/Write Parameter Response, MC_RW_PARAM_RESP
class MC_read_write_parameter_response {
    public:
        MC_read_write_parameter_response();
        MC_read_write_parameter_response(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint16_t get_parameter_address();
        bool get_write_success();
        uint32_t get_data();
    private:
        MCRWParamResp_t message;
};

//C3: MCU_STAT
class MCU_status {
    public:
        MCU_status();
        MCU_status(uint8_t buf[8]);
        MCU_status(uint8_t state, uint8_t flags, int16_t temperature, uint16_t glv_battery_voltage);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint8_t get_state();
        uint8_t get_flags();
        bool get_bms_ok_high();
        bool get_imd_okhs_high();
        bool get_inverter_powered();
        bool get_shutdown_b_above_threshold();
        bool get_shutdown_c_above_threshold();
        bool get_shutdown_d_above_threshold();
        bool get_shutdown_e_above_threshold();
        bool get_shutdown_f_above_threshold();
        int16_t get_temperature();
        uint16_t get_glv_battery_voltage();
        void set_state(uint8_t state);
        void set_flags(uint8_t flags);
        void set_bms_ok_high(bool bms_ok_high);
        void set_imd_okhs_high(bool imd_okhs_high);
        void set_inverter_powered(bool inverter_powered);
        void set_shutdown_b_above_threshold(bool shutdown_b_above_threshold);
        void set_shutdown_c_above_threshold(bool shutdown_c_above_threshold);
        void set_shutdown_d_above_threshold(bool shutdown_d_above_threshold);
        void set_shutdown_e_above_threshold(bool shutdown_e_above_threshold);
        void set_shutdown_f_above_threshold(bool shutdown_f_above_threshold);
        void set_temperature(int16_t temperature);
        void set_glv_battery_voltage(uint16_t glv_battery_voltage);
    private:
        MCUStat_t message;
};

//C4: MCU_PEDAL_READ
class MCU_pedal_readings {
    public:
        MCU_pedal_readings();
        MCU_pedal_readings(uint8_t buf[8]);
        MCU_pedal_readings(uint16_t accelerator_pedal_raw_1, uint16_t accelerator_pedal_raw_2, uint16_t brake_pedal_raw, uint8_t pedal_flags, uint8_t torque_map_mode);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint16_t get_accelerator_pedal_raw_1();
        uint16_t get_accelerator_pedal_raw_2();
        uint16_t get_brake_pedal_raw();
        uint8_t get_pedal_flags();
        bool get_accelerator_implausibility();
        bool get_brake_implausibility();
        bool get_brake_pedal_active();
        uint8_t get_torque_map_mode();
        void set_accelerator_pedal_raw_1(uint16_t accelerator_pedal_raw_1);
        void set_accelerator_pedal_raw_2(uint16_t accelerator_pedal_raw_2);
        void set_brake_pedal_raw(uint16_t brake_pedal_raw);
        void set_pedal_flags(uint8_t pedal_flags);
        void set_accelerator_implausibility(bool accelerator_implausibility);
        void set_brake_implausibility(bool brake_implausibility);
        void set_brake_pedal_active(bool brake_pedal_active);
        void set_torque_map_mode(uint8_t torque_map_mode);
    private:
        MCUPedalRead_t message;
};

//CC: GLV Current Readings, GLV_CURR_READ
class GLV_current_readings {
	public:
		GLV_current_readings();
		GLV_current_readings(uint8_t buf[8]);
		GLV_current_readings(uint16_t ecu_current_value, uint16_t cooling_current_value);
		void load(uint8_t buf[8]);
		void write(uint8_t buf[8]);
		uint16_t get_ecu_current_value();
		uint16_t get_cooling_current_value();
		void set_ecu_current_value(uint16_t ecu_current_value);
		void set_cooling_current_value(uint16_t cooling_current_value);
	private:
		GLVCurrRead_t message;
};

//D3: ID_FCU_READINGS
class FCU_readings {
    public:
        FCU_readings();
        FCU_readings(uint8_t buf[8]);
        FCU_readings(uint16_t accelerator_pedal_raw_1, uint16_t accelerator_pedal_raw_2, uint16_t brake_pedal_raw, int16_t temperature);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint16_t get_accelerator_pedal_raw_1();
        uint16_t get_accelerator_pedal_raw_2();
        uint16_t get_brake_pedal_raw();
        int16_t get_temperature();
        void set_accelerator_pedal_raw_1(uint16_t accelerator_pedal_raw_1);
        void set_accelerator_pedal_raw_2(uint16_t accelerator_pedal_raw_2);
        void set_brake_pedal_raw(uint16_t brake_pedal_raw);
        void set_temperature(int16_t temperature);
    private:
        FCURead_t message;
};

//D5: Battery Monitoring System Onboard Temps, BMS_ONB_TEMP
class BMS_onboard_temperatures {
    public:
        BMS_onboard_temperatures();
        BMS_onboard_temperatures(uint8_t buf[]);
        BMS_onboard_temperatures(int16_t average_temperature, int16_t low_temperature, int16_t high_temperature);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        int16_t get_average_temperature();
        int16_t get_low_temperature();
        int16_t get_high_temperature();
        void set_average_temperature(int16_t average_temperature);
        void set_low_temperature(int16_t low_temperature);
        void set_high_temperature(int16_t high_temperature);
    private:
        BMSOnbTemp_t message;
};

//D6: BMS Onboard Detailed Temps, BMS_ONB_DET_TEMP
class BMS_onboard_detailed_temperatures {
    public:
        BMS_onboard_detailed_temperatures();
        BMS_onboard_detailed_temperatures(uint8_t buf[]);
        BMS_onboard_detailed_temperatures(uint8_t ic_id, int16_t temperature_0, int16_t temperature_1);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        uint8_t get_ic_id();
        int16_t get_temperature_0();
        int16_t get_temperature_1();
        int16_t get_temperature(uint8_t temperature_id);
        void set_ic_id(uint8_t ic_id);
        void set_temperature_0(int16_t temperature_0);
        void set_temperature_1(int16_t temperature_1);
        void set_temperature(uint8_t temperature_id, int16_t temperature);
    private:
        BMSOnbDetTemp_t message;
};

//D7: BMS Voltages, BMS_VOLT
class BMS_voltages {
    public:
        BMS_voltages();
        BMS_voltages(uint8_t buf[]);
        BMS_voltages(uint16_t average_voltage, uint16_t low_voltage, uint16_t high_voltage, uint16_t total_voltage);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        uint16_t get_average();
        uint16_t get_low();
        uint16_t get_high();
        uint16_t get_total();
        void set_average(uint16_t average_voltage);
        void set_low(uint16_t low_voltage);
        void set_high(uint16_t high_voltage);
        void set_total(uint16_t total_voltage);
    private:
        BMSVolt_t message;
};

//D8: BMS Detailed Voltages, BMS_DET_VOLT
class BMS_detailed_voltages {
    public:
        BMS_detailed_voltages();
        BMS_detailed_voltages(uint8_t buf[]);
        BMS_detailed_voltages(uint8_t ic_id, uint8_t group_id, uint16_t voltage_0, uint16_t voltage_1, uint16_t voltage_2);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        uint8_t get_ic_id();
        uint8_t get_group_id();
        uint16_t get_voltage_0();
        uint16_t get_voltage_1();
        uint16_t get_voltage_2();
        uint16_t get_voltage(uint8_t voltage_id);
        void set_ic_id(uint8_t ic_id);
        void set_group_id(uint8_t group_id);
        void set_voltage_0(uint16_t voltage_0);
        void set_voltage_1(uint16_t voltage_1);
        void set_voltage_2(uint16_t voltage_2);
        void set_voltage(uint8_t voltage_id, uint16_t voltage);
    private:
        BMSDetVolt_t message;
};

//D9: BMS Temperatures, BMS_TEMP
class BMS_temperatures {
    public:
        BMS_temperatures();
        BMS_temperatures(uint8_t buf[]);
        BMS_temperatures(int16_t average_temperature, int16_t low_temperature, int16_t high_temperature);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        int16_t get_average_temperature();
        int16_t get_low_temperature();
        int16_t get_high_temperature();
        void set_average_temperature(int16_t average_temperature);
        void set_low_temperature(int16_t low_temperature);
        void set_high_temperature(int16_t high_temperature);
    private:
        BMSTemp_t message;
};

//DA: BMS Detailed Temperatures, BMS_DET_TEMP
class BMS_detailed_temperatures {
    public:
        BMS_detailed_temperatures();
        BMS_detailed_temperatures(uint8_t buf[]);
        BMS_detailed_temperatures(uint8_t ic_id, int16_t temperature_0, int16_t temperature_1, int16_t temperature_2);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        uint8_t get_ic_id();
        int16_t get_temperature_0();
        int16_t get_temperature_1();
        int16_t get_temperature_2();
        int16_t get_temperature(uint8_t temperature_id);
        void set_ic_id(uint8_t ic_id);
        void set_temperature_0(int16_t temperature_0);
        void set_temperature_1(int16_t temperature_1);
        void set_temperature_2(int16_t temperature_2);
        void set_temperature(uint8_t temperature_id, int16_t temperature);
    private:
        BMSDetTemp_t message;
};

//DB: BMS Status, BMS_STAT
class BMS_status {
    public:
        BMS_status();
        BMS_status(uint8_t buf[]);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        uint8_t get_state();
        uint16_t get_error_flags();
        bool get_overvoltage();
        bool get_undervoltage();
        bool get_total_voltage_high();
        bool get_discharge_overcurrent();
        bool get_charge_overcurrent();
        bool get_discharge_overtemp();
        bool get_charge_overtemp();
        bool get_undertemp();
        bool get_onboard_overtemp();
        int16_t get_current();
        uint8_t get_flags();
        bool get_shutdown_g_above_threshold();
        bool get_shutdown_h_above_threshold();

        void set_state(uint8_t state);
        void set_error_flags(uint16_t error_flags);
        void set_overvoltage(bool overvoltage);
        void set_undervoltage(bool undervoltage);
        void set_total_voltage_high(bool total_voltage_high);
        void set_discharge_overcurrent(bool discharge_overcurrent);
        void set_charge_overcurrent(bool charge_overcurrent);
        void set_discharge_overtemp(bool discharge_overtemp);
        void set_charge_overtemp(bool charge_overtemp);
        void set_undertemp(bool undertemp);
        void set_onboard_overtemp(bool onboard_overtemp);
        void set_current(int16_t current);
        void set_flags(uint8_t flags);
        void set_shutdown_g_above_threshold(bool shutdown_g_above_threshold);
        void set_shutdown_h_above_threshold(bool shutdown_h_above_threshold);
    private:
        BMSStat_t message;
};

//DD: Charge Control Unit Status, CCU_STAT
class CCU_status {
    public:
        CCU_status();
        CCU_status(uint8_t buf[]);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        bool get_charger_enabled();
        void set_charger_enabled(bool charger_enabled);
    private:
        CCUStat_t message;
};

//DE: BMS Balancing Status, BMS_BAL_STAT
class BMS_balancing_status {
    public:
        BMS_balancing_status();
        BMS_balancing_status(uint8_t buf[]);
        BMS_balancing_status(uint8_t group_id, int64_t balancing_status);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        uint8_t get_group_id();
        uint64_t get_balancing();
        uint16_t get_ic_balancing(uint8_t ic_id);
        bool get_cell_balancing(uint8_t ic_id, uint16_t cell_id);

        void set_group_id(uint8_t group_id);
        void set_balancing(uint64_t balancing_status);
        void set_ic_balancing(uint8_t ic_id, uint16_t balancing_status);
        void set_cell_balancing(uint8_t ic_id, uint8_t cell_id, bool balancing_status);
    private:
        uint64_t message; // Using a 64-bit datatype here instead of BMSBalStat_t because it is much easier than dealing with an array
};

//DF: FCU_ACCEL
class FCU_accelerometer_values {
    public:
        FCU_accelerometer_values();
        FCU_accelerometer_values(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint8_t get_x();
        uint8_t get_y();
        uint8_t get_z();
        void set_values(uint8_t x, uint8_t y, uint8_t z);
    private:
        FCUAccel_t message;
};

//E2: BMS Coulomb Counts, BMS_COUL_COUNTS
class BMS_coulomb_counts {
    public:
        BMS_coulomb_counts();
        BMS_coulomb_counts(uint8_t buf[]);
        BMS_coulomb_counts(uint32_t total_charge, uint32_t total_discharge);
        void load(uint8_t buf[]);
        void write(uint8_t buf[]);
        uint32_t get_total_charge();
        uint32_t get_total_discharge();
        void set_total_charge(uint32_t total_charge);
        void set_total_discharge(uint32_t total_discharge);
    private:
        BMSCoulCounts_t message;
};

//E7: ID_ECU_GPS_READINGS_ALPHA
class MCU_GPS_readings_alpha {
    public:
        MCU_GPS_readings_alpha();
        MCU_GPS_readings_alpha(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int32_t get_latitude();
        int32_t get_longitude();
        void set_latitude(int32_t latitude);
        void set_longitude(int32_t longitude);
    private:
        MCUGpsAlpha_t message;
};

//E8: ID_ECU_GPS_READINGS_BETA
class MCU_GPS_readings_beta {
    public:
        MCU_GPS_readings_beta();
        MCU_GPS_readings_beta(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        int32_t get_altitude();
        int32_t get_speed();
        void set_altitude(int32_t altitude);
        void set_speed(int32_t speed);
    private:
        MCUGpsBeta_t message;
};

//E9: MCU_GPS_READ_GAMMA
class MCU_GPS_readings_gamma {
    public:
        MCU_GPS_readings_gamma();
        MCU_GPS_readings_gamma(uint8_t buf[8]);
        void load(uint8_t buf[8]);
        void write(uint8_t buf[8]);
        uint8_t get_fix_quality();
        uint8_t get_satellite_count();
        uint32_t get_timestamp_seconds();
        uint16_t get_timestamp_milliseconds();
        void set_fix_quality(uint8_t fix_quality);
        void set_satellite_count(uint8_t satellite_count);
        void set_timestamp_seconds(uint32_t timestamp_seconds);
        void set_timestamp_milliseconds(uint16_t timestamp_milliseconds);
    private:
        MCUGpsGamma_t message;
};

#endif
#endif

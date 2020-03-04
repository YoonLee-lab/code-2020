![HyTech logo](https://hytechracing.gatech.edu/images/hytech_logo_small.png)

# HyTech_CAN Library
This library serves as a central location for ECU state IDs, CAN message IDs, and CAN message parsers.
## Getting started
You will need to add this directory to your Arduino libraries directory. The preferred way to do this is with a symlink to this directory's location in the cloned repository. This way it will be updated whenever you pull updates from this repository.
## CAN message parsing
Each message type that is sent on the vehicle CAN bus is implemented as a struct and a class in this library. Some messages are only sent and others are only read by team-designed ECUs. Each class for messages sent by team-designed ECUs has functions to create and read the message, while all other classes only have functions to read the message.
## CAN message data formats
To avoid issues with floating point numbers, HyTech's CAN messages instead use scaled integers, predefined based on the data type. For consistency, we use the RMS CAN Protocol Data Formats when possible which can be referenced in [their documentation](https://app.box.com/s/vf9259qlaadhzxqiqrt5cco8xpsn84hk/1/2822171827/27334613044/1) (page 11). This library does not attempt to convert these values into floating point numbers.

CAN Message Types and Information
---------------------------------

Date Modified: March 4, 2020

Note: Right now I'm changing the order of CAN messages here. I'm not changing the contents, just making it in hex-number order. This is to make it more readable and easier to add things to. If I change any of the contents, this is to add the message types that haven't been recorded yet.

Contact: Varnika/Meghavarnika Budati/Vivacia <mbudati3@gatech.edu>

Please document all CAN message types here.

<https://docs.google.com/document/d/1Wt2YLL93o2cYXGNJFQi_t4vT7_77oTf3JhUFi9HwC8g/edit?usp=sharing>

CAN IDs in Use
--------------

### A0:
**Description:** Motor Controller Temperatures #1  
**Macro:** `MC_TEMP_1`, previously(`ID_MC_TEMPERATURES_1`)  
**Struct:** `MCTemp1_t`, previously(`CAN_message_mc_temperatures_1_t`)  
**Class:** `MC_temperatures_1`  
**Data:**
*  `module_a_temperature` (2 bytes, signed)
*  `module_b_temperature` (2 bytes, signed)
*  `module_c_temperature` (2 bytes, signed)
*  `gate_driver_board_temperature` (2 bytes, signed)

### A1:
**Description:** Motor Controller Temperatures #2  
**Macro:** `MC_TEMP_2`, previously(`ID_MC_TEMPERATURES_2`)  
**Struct:** `MCTemp2_t`, previously(`CAN_message_mc_temperatures_2_t`)  
**Class:** `MC_temperatures_2`  
**Data:**
*  `control_board_temperature` (2 bytes, signed)
*  `rtd_1_temperature` (2 bytes, signed)
*  `rtd_2_temperature` (2 bytes, signed)
*  `rtd_3_temperature` (2 bytes, signed)

### A2:
**Description:** Motor Controller Temperatures #3  
**Macro:** `MC_TEMP_3`, previously(`ID_MC_TEMPERATURES_3`)  
**Struct:** `MCTemp3_t`, previously(`CAN_message_mc_temperatures_3_t`)  
**Class:** `MC_temperatures_3`  
**Data:**
*  `rtd_4_temperature` (2 bytes, signed)
*  `rtd_5_temperature` (2 bytes, signed)
*  `motor_temperature` (2 bytes, signed)
*  `torque_shudder` (2 bytes, signed)
RTD: Resistance Temperature Detector

### A3:
**Description:** MC Analog Inputs Voltages  
**Macro:** `MC_A_INP_VOLT`, previously(`ID_MC_ANALOG_INPUTS_VOLTAGES`)  
**Struct:** `MCAInpVolt_t`, previously(`CAN_message_mc_analog_input_voltages_t`)  
**Class:** `MC_analog_input_voltages`  
**Data:**
*  `analog_input_1` (2 bytes, signed)
*  `analog_input_2` (2 bytes, signed)
*  `analog_input_3` (2 bytes, signed)
*  `analog_input_4` (2 bytes, signed)

### A4:
**Description:** MC Digital Input Status  
**Macro:** `MC_D_INP_VOLT`, previously(`ID_MC_DIGITAL_INPUT_STATUS`)  
**Struct:** `MCDInpVolt_t`, previously(`CAN_message_mc_digital_input_status_t`)  
**Class:** `MC_digital_input_status`  
**Data:**
*  `digital_input_1` (1 byte, Boolean)
*  `digital_input_2` (1 byte, Boolean)
*  `digital_input_3` (1 byte, Boolean
*  `digital_input_4` (1 byte, Boolean)
*  `digital_input_5` (1 byte, Boolean)
*  `digital_input_6` (1 byte, Boolean)
*  `digital_input_7` (1 byte, Boolean)
*  `digital_input_8` (1 byte, Boolean)

### A5:
**Description:** MC Motor Position Information  
**Macro:** `MC_MOTOR_POS_INFO`, previously(`ID_MC_MOTOR_POSITION_INFORMATION`)  
**Struct:** `MCMotorPosInfo_t`, previously(`CAN_message_mc_motor_position_information_t`)  
**Class:** `MC_motor_position_information`  
**Data:**
*  `motor_angle` (2 bytes, signed)
*  `motor_speed` (2 bytes, signed)
*  `electrical_output_frequency` (2 bytes, signed)
*  `delta_resolver_filtered` (2 bytes, signed)

### A6:
**Description:** MC Current Information  
**Macro:** `MC_CURR_INFO`, previously(`ID_MC_CURRENT_INFORMATION`)  
**Struct:** `MCCurrInfo_t`, previously(`CAN_message_mc_current_information_t`)  
**Class:** `MC_current_information`  
**Data:**
*  `phase_a_current` (2 bytes, signed)
*  `phase_b_current` (2 bytes, signed)
*  `phase_c_current` (2 bytes, signed)
*  `dc_bus_current` (2 bytes, signed)

### A7:
**Description:** MC Voltage Information  
**Macro:** `MC_VOLT_INFO`, previously(`ID_MC_VOLTAGE_INFORMATION`)  
**Struct:** `MCVoltInfo_t`, previously(`CAN_message_mc_voltage_information_t`)  
**Class:** `MC_voltage_information`  
**Data:**
*  `dc_bus_voltage` (2 bytes, signed)
*  `output_voltage` (2 bytes, signed)
*  `phase_ab_voltage` (2 bytes, signed)
*  `phase_bc_voltage` (2 bytes, signed)

### AA:
**Description:** MC Internal States  
**Macro:** `MC_INT_STATES`, previously(`ID_MC_INTERNAL_STATES`)  
**Struct:** `MCIntStates_t`, previously(`CAN_message_mc_internal_states_t`)  
**Class:** `MC_internal_states`  
**Data:**
*  `vsm_state` (2 bytes)
*  `inverter_state` (1 byte)
*  `relay_state` (1 byte)
*  `inverter_run_mode_discharge_state` (1 byte)
*  `inverter_command_mode` (1 byte)
*  `inverter_enable` (1 byte)
*  `direction_command` (1 byte)

### AB:
**Description:** MC Fault Codes  
**Macro:** `MC_FAULT_CODES`, previously(`ID_MC_FAULT_CODES`)  
**Struct:** `MCFaultCodes_t`, previously(`CAN_message_mc_fault_codes_t`)  
**Class:** `MC_fault_codes`  
**Data:**
*  `post_fault_lo` (2 bytes)
*  `post_fault_hi` (2 bytes)
*  `run_fault_lo` (2 bytes)
*  `run_fault_hi` (2 bytes)

### AC:
**Description:** MC Torque Timer Information  
**Macro:** `MC_TOR_TIMER_INFO`, previously(`ID_MC_TORQUE_TIMER_INFORMATION`)  
**Struct:** `MCTorTimerInfo_t`, previously(`CAN_message_mc_torque_timer_information_t`)  
**Class:** `MC_torque_timer_information`  
**Data:**
*  `commanded_torque` (2 bytes, signed)
*  `torque_feedback` (2 bytes, signed)
*  `power_on_timer` (4 bytes)

### AD:
**Description:** MC Modulation Index Flux Weakening Output Information  
**Macro:** `MC_MOD_INDEX_FLUX_WEAKENING_OUT_INFO`, previously(`ID_MC_MODULATION_INDEX_FLUX_WEAKENING_OUTPUT_INFORMATION`)  
**Struct:** `MCModIndexFluxWeakeningOutInfo_t`, previously(`CAN_message_mc_modulation_index_flux_weakening_output_information_t`)  
**Class:** `MC_modulation_index_flux_weakening_output_information`  
**Data:**
*  `modulation_index` (2 bytes)
*  `flux_weakening_output` (2 bytes, signed)
*  `id_command` (2 bytes, signed)
*  `iq_command` (2 bytes, signed)

### AE:
**Description:** MC Firmware Information  
**Macro:** `MC_FIRM_INFO`, previously(`ID_MC_FIRMWARE_INFORMATION`)  
**Struct:** `MCFirmInfo_t`, previously(`CAN_message_mc_firmware_information_t`)  
**Class:** `MC_firmware_information`  
**Data:**
*  `eeprom_version_project_code` (2 bytes)
*  `software_version` (2 bytes)
*  `date_code_mmdd` (2 bytes)
*  `date_code_yyyy` (2 bytes)

### C0:
**Description:** MC Command Message  
**Macro:** `MC_COMM_MSG`, previously(`ID_MC_COMMAND_MESSAGE`)  
**Struct:** `MCCommMsg_t`, previously(`CAN_message_mc_command_message_t`)  
**Class:** `MC_command_message`  
**Data:**
*  `torque_command` (2 bytes, signed)
*  `angular_velocity` (2 bytes, signed)
*  `direction` (1 byte, Boolean)
*  `inverter_enable_discharge_enable` (1 byte)
*  `commanded_torque_limit` (2 bytes, signed)

### C1:
**Description:** MC Read/Write Parameter Command  
**Macro:** `MC_RW_PARAM_COMM`, previously(`ID_MC_READ_WRITE_PARAMETER_COMMAND`)  
**Struct:** `MCRWParamComm_t`, previously(`CAN_message_mc_read_write_parameter_command_t`)  
**Class:** `MC_read_write_parameter_command`  
**Data:**
*  `parameter_address` (2 bytes)
*  `rw_command` (1 byte, Boolean)
*  `reserved` (1 byte)
*  `data` (4 bytes)

### C2:
**Description:** MC Read/Write Parameter Response  
**Macro:** `MC_RW_PARAM_RESP`, previously(`ID_MC_READ_WRITE_PARAMETER_RESPONSE`)  
**Struct:** `MCRWParamResp_t`, previously(`CAN_message_mc_read_write_parameter_response_t`)  
**Class:** `MC_read_write_parameter_response`  
**Data:**
*  `parameter_address` (2 bytes)
*  `write_success` (1 byte, Boolean)
*  `reserved` (1 byte)
*  `data` (4 bytes)

### C3:
**Macro:** `MC_STAT`, previously(`ID_MCU_STATUS`)  
**Struct:** `MCUStat_t`, previously(`CAN_message_mcu_status_t`)  
**Class:** ` MCU_status`  
**Data:**
*  `state` (1 byte)
*  `flags` (1 byte)
*  `temperature` (2 bytes, signed)
*  `glv_battery_voltage` (2 bytes)

### C4:
**Macro:** `MCU_PEDAL_READ`, previously(`ID_MCU_PEDAL_READINGS`)  
**Struct:** `MCUPedalRead_t`, previously(`CAN_message_mcu_pedal_readings_t`)  
**Class:** ` MCU_pedal_readings`  
**Data:**
*  `accelerator_pedal_raw_1` (2 bytes)
*  `accelerator_pedal_raw_2` (2 bytes)
*  `brake_pedal_raw` (2 bytes)
*  `pedal_flags` (1 byte)
*  `torque_map_mode` (1 byte)

### CC:
**Description:** GLV Current Readings  
**Macro:** `GLV_CURR_READ`, previously(`ID_GLV_CURRENT_READINGS`)  
**Struct:** `GLVCurrRead_t`, previously(`CAN_message_glv_current_readings_t`)  
**Class:** `GLV_current_readings`  
**Data:**
*  `ecu_current_value` (4 bytes, signed)
*  `cooling_current_value` (4 bytes, signed)

### D3:
**Macro:** `FCU_READ`, previously(`ID_FCU_READINGS`)  
**Struct:** `FCURead_t`, previously(`CAN_message_fcu_readings_t`)  
**Class:** `FCU_readings`  
**Data:**
*  `accelerator_pedal_raw_1` (2 bytes)
*  `accelerator_pedal_raw_2` (2 bytes)
*  `brake_pedal_raw` (2 bytes)
*  `temperature` (2 byte)

### D5:
**Description:** Battery Monitoring System Onboard Temps  
**Macro:** `BMS_ONB_TEMP`, previously(`ID_BMS_ONBOARD_TEMPERATURES`)  
**Struct:** `BMSOnbTemp_t`, previously(`CAN_message_bms_onboard_temperatures_t`)  
**Class:** `BMS_onboard_temperatures`  
**Data:**
*  `average_temperature` (2 bytes, signed)
*  `low_temperature` (2 bytes, signed)
*  `high_temperature` (2 bytes, signed)

### D6:
**Description:** BMS Onboard Detailed Temps  
**Macro:** `BMS_ONB_DET_TEMP`, previously(`ID_BMS_ONBOARD_DETAILED_TEMPERATURES`)  
**Struct:** `BMSOnbDetTemp_t`, previously(`CAN_message_bms_onboard_detailed_temperatures_t`)  
**Class:** `BMS_onboard_detailed_temperatures`  
**Data:**
*  `ic_id` (2 bytes, signed)
*  `temperature_0` (2 bytes, signed)
*  `temperature_1` (2 bytes, signed)

### D7:
**Description:** BMS Voltages  
**Macro:** `BMS_VOLT`, previously(`ID_BMS_VOLTAGES`)  
**Struct:** `BMSVolt_t`, previously(`CAN_message_bms_voltages_t`)  
**Class:** `BMS_voltages`  
**Data:**
*  `average_voltage` (2 bytes, signed)
*  `low_voltage` (2 bytes, signed)
*  `high_voltage` (2 bytes, signed)
*  `total_voltage` (2 bytes, signed)

### D8:
**Description:** BMS Detailed Voltages  
**Macro:** `BMS_DET_VOLT`, previously(`ID_BMS_DETAILED_VOLTAGES`)  
**Struct:** `BMSDetVolt_t`, previously(`CAN_message_bms_detailed_voltages_t`)  
**Class:** `BMS_detailed_voltages`  
**Data:**
*  `ic_id` (1 byte)
*  `group_id` (1 byte)
*  `voltage_0` (2 bytes)
*  `voltage_1` (2 bytes)
*  `voltage_2` (2 bytes)

### D9:
**Description:** BMS Temperatures  
**Macro:** `BMS_TEMP`, previously(`ID_BMS_TEMPERATURES`)  
**Struct:** `BMSTemp_t`, previously(`CAN_message_bms_temperatures_t`)  
**Class:** `BMS_temperatures`  
**Data:**
*  `average_temperature` (2 bytes, signed)
*  `low_temperature` (2 bytes, signed)
*  `high_temperature` (2 bytes, signed)

### DA:
**Description:** BMS Detailed Temperatures  
**Macro:** `BMS_DET_TEMP`, previously(`ID_BMS_DETAILED_TEMPERATURES`)  
**Struct:** `BMSDetTemp_t`, previously(`CAN_message_bms_detailed_temperatures_t`)  
**Class:** `BMS_detailed_temperatures`  
**Data:**
*  `ic_id` (1 byte)
*  `temperature_0` (2 bytes, signed)
*  `temperature_1` (2 bytes, signed)
*  `temperature_2` (2 bytes, signed)

### DB:
**Description:** BMS Status  
**Macro:** `BMS_STAT`, previously(`ID_BMS_STATUS`)  
**Struct:** `BMSStat_t`, previously(`CAN_message_bms_status_t`)  
**Class:** `BMS_status`  
**Data:**
*  `state` (1 byte)
*  `error_flags` (2 bytes)
*  `current` (2 bytes, signed)
*  `flags` (1 byte)

### DD:
**Description:** Charge Control Unit Status  
**Macro:** `CCU_STAT`, previously(`ID_CCU_STATUS`)  
**Struct:** `CCUStat_t`, previously(`CAN_message_ccu_status_t`)  
**Class:** `CCU_status`  
**Data:**
*  `charger_enabled` (1 byte, Boolean)

### DE:
**Description:** BMS Balancing Status  
**Macro:** `BMS_BAL_STAT`, previously(`ID_BMS_BALANCING_STATUS`)  
**Struct:** `BMSBalStat_t`, previously(`CAN_message_bms_balancing_status_t`)  
**Class:** `BMS_balancing_status`  
**Data:**
*  ``balancing_status[]`` (1 byte × 5)

### DF:
**Macro:** `FCU_ACCEL`, previously(`ID_FCU_ACCELEROMETER`)  
**Struct:** `FCUAccel_t`, previously(`CAN_message_fcu_accelerometer_values_t`)  
**Class:** `FCU_accelerometer_values`  
**Data:**
*  `XValue_x100` (2 bytes)
*  `YValue_x100` (2 bytes)
*  `ZValue_x100` (2 bytes)

### E2:
**Description:** BMS Coulomb Counts  
**Macro:** `BMS_COUL_COUNTS`, previously(`ID_BMS_COULOMB_COUNTS`)  
**Struct:** `BMSCoulCounts_t`, previously(`CAN_message_bms_coulomb_counts_t`)  
**Class:** `BMS_coulomb_counts`  
**Data:**
*  `total_charge` (4 bytes)
*  `total_discharge` (4 bytes)

### E7:
**Macro:** `MCU_GPS_READ_ALPHA`, previously(`ID_MCU_GPS_READINGS_ALPHA`)  
**Struct:** `MCUGpsAlpha_t`, previously(`CAN_message_mcu_gps_readings_alpha_t`)  
**Class:** `MCU_GPS_readings_alpha`  
**Data:**
*  `latitude` (4 bytes)
*  `longitude` (4 bytes)

### E8:
**Macro:** `MCU_GPS_READ_BETA`, previously(`ID_MCU_GPS_READINGS_BETA`)  
**Struct:** `MCUGpsBeta_t`, previously(`CAN_message_mcu_gps_readings_beta_t`)  
**Class:** `MCU_GPS_readings_beta`  
**Data:**
*  `altitude` (4 bytes)
*  `speed` (4 bytes)

### E9:
**Macro:** `MCU_GPS_READ_GAMMA`, previously(`ID_MCU_GPS_READINGS_GAMMA`)  
**Struct:** `MCUGpsGamma_t`, previously(`CAN_message_mcu_gps_readings_gamma_t`)  
**Class:** `MCU_GPS_readings_gamma`  
**Data:**
*  `fix_quality` (1 byte)
*  `satellite_count` (1 byte)
*  `timestamp_seconds` (4 bytes)
*  `timestamp_milliseconds` (2 bytes)

Old/Obsolete IDs
----------------

These can be reused → they were in the older versions of code, simply lying there and not used. I removed the classes and structs associated with them.

### D0:
**Description:** Rear Control Unit Status  
**Macro:** `ID_RCU_STATUS`  
**Struct:** `CAN_msg_rcu_status`  
**Class:** `RCU_status`

### D2:
**Description:** Front Control Unit Status  
**Macro:** `ID_FCU_STATUS`  
**Struct:** `CAN_msg_fcu_status`  
**Class:** `FCU_status`

Unknown IDs
-----------

If you find any information about them, add it here. If there are any CAN IDs with little to no details, feel free to put them in here. If deemed useless, move it to Old/Obsolete IDs or reuse them.

### A8:
**Macro:** `MC_FLUX_INFO`

### A9:
**Macro:** `MC_INT_VOLT`

### AF:
**Macro:** `MC_DIAGN_DATA`

### DC:
**Macro:** `FH_WATCHDOG_TEST`

### E1:
**Macro:** `BMS_PARAM_RESP`

Abbreviations Used:
===================

*  TEMP: temperature
*  INP: input
*  VOLT: voltage
*  STAT: status
*  A: analog
*  D: digital
*  POS: position 
*  INFO: information 
*  CURR: current 
*  INT: internal 
*  TOR: torque 
*  MOD: modulation 
*  OUT: output 
*  FIRM: firmware
*  DIAG: diagnostic
*  COMM: command
*  MSG: message
*  RESP: response
*  PARAM: parameter
*  READ: reading
*  DET: detailed
*  BAL: balancing
*  RW: read/write
*  COUL: coulomb
*  ACCEL: accelerometer
*  RTD: Resistance Temperature Detector
*  ONB: onboard

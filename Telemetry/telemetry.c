#include <stdlib.h>
#include <stdio.h>
#include "../Libraries/HyTech_CAN/HyTech_CAN.h"
#include "../Libraries/XBTools/XBTools.h"
#include <MQTTClient.h>

struct {
    uint64_t                         timestamp;
    FCURead_t                        fcu_readings;
    BMSVolt_t                        bms_voltages;
    BMSDetVolt_t                     bms_detailed_voltages;
    BMSTemp_t                        bms_temperatures;
    BMSDetTemp_t                     bms_detailed_temperatures;
    BMSOnbTemp_t                     bms_onboard_temperatures;
    BMSOnbDetTemp_t                  bms_onboard_detailed_temperatures;
    BMSStat_t                        bms_status;
    BMSBalStat_t                     bms_balancing_status;
    CCUStat_t                        ccu_status;
    MCTemp1_t                        mc_temperatures_1;
    MCTemp2_t                        mc_temperatures_2;
    MCTemp3_t                        mc_temperatures_3;
    MCAInpVolt_t                     mc_analog_input_voltages;
    MCDInpVolt_t                     mc_digital_input_status;
    MCMotorPosInfo_t                 mc_motor_position_information;
    MCCurrInfo_t                     mc_current_information;
    MCVoltInfo_t                     mc_voltage_information;
    MCIntStates_t                    mc_internal_states;
    MCFaultCodes_t                   mc_fault_codes;
    MCTorTimerInfo_t                 mc_torque_timer_information;
    MCModIndexFluxWeakeningOutInfo_t mc_modulation_index_flux_weakening_output_information;
    MCFirmInfo_t                     mc_firmware_information;
    MCCommMsg_t                      mc_command_message;
    MCRWParamComm_t                  mc_read_write_parameter_command;
    MCRWParamResp_t                  mc_read_write_parameter_response;
    FCUAccel_t                       fcu_accelerometer_values;
} current_status;

static void process_message(uint64_t timestamp, Telem_message_t *msg)
{
    // Do logging stuff.
    printf("\nTimestamp: %llu\n", timestamp);
    current_status.timestamp = timestamp;

    switch (msg->msg_id) {
        // OBSOLETE
        // case ID_RCU_STATUS:
        // {
        //     CAN_msg_rcu_status *data = &msg->contents.rcu_status;
        //     printf("RCU STATE: %hhu\n"
        //            "RCU FLAGS: 0x%hhX\n"
        //            "GLV BATT VOLTAGE: %f V\n"
        //            "RCU BMS FAULT: %hhu\n"
        //            "RCU IMD FAULT: %hhu\n",
        //            data->state,
        //            data->flags,
        //            data->glv_battery_voltage / 100.0,
        //            (char)(!(data->flags & 1)),
        //            (char)(!(data->flags & 2)));
        //     current_status.rcu_status = *data;
        //     break;
        // }
        // case ID_FCU_STATUS:
        // {
        //     CAN_msg_fcu_status *data = &msg->contents.fcu_status;
        //     printf("FCU STATE: %hhu\n"
        //            "FCU FLAGS: %hhX\n"
        //            "FCU START BUTTON ID: %hhu\n"
        //            "FCU BRAKE ACT: %hhu\n"
        //            "FCU IMPLAUS ACCEL: %hhu\n"
        //            "FCU IMPLAUSE BRAKE: %hhu\n",
        //            data->state,
        //            data->flags,
        //            data->start_button_press_id,
        //            (char)((data->flags & 8) >> 3),
        //            (char)(data->flags & 1),
        //            (char)((data->flags & 4) >> 2));
        //     current_status.fcu_status = *data;
        //     break;
        // }
        case FCU_READ:
        {
            FCURead_t *data = &msg->contents.fcu_readings;
            printf("FCU PEDAL ACCEL 1: %hu\n"
                   "FCU PEDAL ACCEL 2: %hu\n"
                   "FCU PEDAL BRAKE: %hu\n",
                   data->accelerator_pedal_raw_1,
                   data->accelerator_pedal_raw_2,
                   data->brake_pedal_raw);
            current_status.fcu_readings = *data;
            break;
        }
        case FCU_ACCEL:
        {
            break;  // TODO
        }
        // OBSOLETE
        // case ID_RCU_RESTART_MC:
        // {
        //     break;  // TODO
        // }
        case BMS_ONB_TEMP:
        case BMS_ONB_DET_TEMP:
        case BMS_VOLT:
        {
            BMSVolt_t *data = &msg->contents.bms_voltages;
            printf("BMS VOLTAGE AVERAGE: %f V\n"
                   "BMS VOLTAGE LOW: %f V\n"
                   "BMS VOLTAGE HIGH: %f V\n"
                   "BMS VOLTAGE TOTAL: %f V\n",
                   data->average_voltage / 1000.0,
                   data->low_voltage / 1000.0,
                   data->high_voltage / 1000.0,
                   data->total_voltage / 100.0);
            current_status.bms_voltages = *data;
            break;
        }
        case BMS_DET_VOLT:
        {
            break;  // TODO need to check docs
        }
        case BMS_TEMP:
        {
            BMSTemp_t *data = &msg->contents.bms_temperatures;
            printf("BMS AVERAGE TEMPERATURE: %f C\n"
                   "BMS LOW TEMPERATURE: %f C\n"
                   "BMS HIGH TEMPERATURE: %f C\n",
                   data->average_temperature / 100.0,
                   data->low_temperature / 100.0,
                   data->high_temperature / 100.0);
            current_status.bms_temperatures = *data;
            break;
        }
        case BMS_DET_TEMP:
        {
            break;  // TODO need to check docs
        }
        case BMS_STAT:
        {
            BMSStat_t *data = &msg->contents.bms_status;
            printf("BMS STATE: %hhu\n"
                   "BMS ERROR FLAGS: 0x%hX\n"
                   "BMS CURRENT: %f A\n",
                   data->state,
                   data->error_flags,
                   data->current / 100.0);
            current_status.bms_status = *data;
            break;
        }
        case BMS_BAL_STAT:
            break;
        // OBSOLETE
        // case ID_FH_WATCHDOG_TEST:
        //     break;
        case CCU_STAT:
            break;
        case MC_TEMP_1: {
            MCTemp1_t *data = &msg->contents.mc_temperatures_1;
            printf("MODULE A TEMP: %f C\n"
                   "MODULE B TEMP: %f C\n"
                   "MODULE C TEMP: %f C\n"
                   "GATE DRIVER BOARD TEMP: %f C\n",
                    data->module_a_temperature / 10.,
                    data->module_b_temperature / 10.,
                    data->module_c_temperature / 10.,
                    data->gate_driver_board_temperature / 10.);
            current_status.mc_temperatures_1 = *data;
            break;
        }
        case MC_TEMP_2: {
            MCTemp2_t *data = &msg->contents.mc_temperatures_2;
            printf("CONTROL BOARD TEMP: %f C\n"
                   "RTD 1 TEMP: %f C\n"
                   "RTD 2 TEMP: %f C\n"
                   "RTD 3 TEMP: %f C\n",
                    data->control_board_temperature / 10.,
                    data->rtd_1_temperature / 10.,
                    data->rtd_2_temperature / 10.,
                    data->rtd_3_temperature / 10.);
            current_status.mc_temperatures_2 = *data;
            break;
        }
        case MC_TEMP_3: {
            MCTemp3_t *data = &msg->contents.mc_temperatures_3;
            printf("RTD 4 TEMP: %f C\n"
                   "RTD 5 TEMP: %f C\n"
                   "MOTOR TEMP: %f C\n"
                   "TORQUE SHUDDER: %f Nm\n",
                    data->rtd_4_temperature / 10.,
                    data->rtd_5_temperature / 10.,
                    data->motor_temperature / 10.,
                    data->torque_shudder / 10.);
            current_status.mc_temperatures_3 = *data;
            break;
        }
        case MC_A_INP_VOLT:
        {
            break;
        }
        case MC_D_INP_VOLT:
            break;
        case MC_MOTOR_POS_INFO:
        {
            MCMotorPosInfo_t *data =
                &msg->contents.mc_motor_position_information;
            printf("MOTOR ANGLE: %f\n"
                   "MOTOR SPEED: %hd RPM\n"
                   "ELEC OUTPUT FREQ: %f\n"
                   "DELTA RESOLVER FILT: %hd\n",
                   data->motor_angle / 10.,
                   data->motor_speed,
                   data->electrical_output_frequency / 10.,
                   data->delta_resolver_filtered);
            current_status.mc_motor_position_information = *data;
            break;
        }
        case MC_CURR_INFO:
        {
            MCCurrInfo_t *data = &msg->contents.mc_current_information;
            printf("PHASE A CURRENT: %f A\n"
                   "PHASE B CURRENT: %f A\n"
                   "PHASE C CURRENT: %f A\n"
                   "DC BUS CURRENT: %f A\n",
                   data->phase_a_current / 10.,
                   data->phase_b_current / 10.,
                   data->phase_c_current / 10.,
                   data->dc_bus_current / 10.);
            current_status.mc_current_information = *data;
            break;
        }
        case MC_VOLT_INFO:
        {
            MCVoltInfo_t *data = &msg->contents.mc_voltage_information;
            printf("DC BUS VOLTAGE: %f V\n"
                   "OUTPUT VOLTAGE: %f V\n"
                   "PHASE AB VOLTAGE: %f V\n"
                   "PHASE BC VOLTAGE: %f V\n",
                   data->dc_bus_voltage / 10.,
                   data->output_voltage / 10.,
                   data->phase_ab_voltage / 10.,
                   data->phase_bc_voltage / 10.);
            current_status.mc_voltage_information = *data;
            break;
        }
        // OBSOLETE
        // case ID_MC_FLUX_INFORMATION:
        //     break;
        // case ID_MC_INTERNAL_VOLTAGES:
        //     break;
        case MC_INT_STATES:
        {
            MCIntStates_t *data = &msg->contents.mc_internal_states;
            printf("VSM STATE: %hu\n"
                   "INVERTER STATE: %hhu\n"
                   "INVERTER RUN MODE: %hhu\n"
                   "INVERTER ACTIVE DISCHARGE STATE: %hhu\n"
                   "INVERTER COMMAND MODE: %hhu\n"
                   "INVERTER ENABLE: %hhu\n"
                   "INVERTER LOCKOUT: %hhu\n"
                   "DIRECTION COMMAND: %hhu\n",
                   data->vsm_state,
                   data->inverter_state,
                   (char)(data->inverter_run_mode_discharge_state & 1),
                   (char)((data->inverter_run_mode_discharge_state & 0xE0) >> 5),
                   data->inverter_command_mode,
                   (char)(data->inverter_enable & 1),
                   (char)((data->inverter_enable & 0x80) >> 7),
                   data->direction_command);
            current_status.mc_internal_states = *data;
        }
        case MC_FAULT_CODES:
        {
            MCFaultCodes_t *data = &msg->contents.mc_fault_codes;
            printf("POST FAULT LO: 0x%hX\n"
                   "POST FAULT HI: 0x%hX\n"
                   "RUN FAULT LO: 0x%hX\n"
                   "RUN FAULT HI: 0x%hx\n",
                   data->post_fault_lo,
                   data->post_fault_hi,
                   data->run_fault_lo,
                   data->run_fault_hi);
            current_status.mc_fault_codes = *data;
            break;
        }
        case MC_TOR_TIMER_INFO:
        {
            MCTorTimerInfo_t *data =
                &msg->contents.mc_torque_timer_information;
            printf("COMMANDED TORQUE: %f Nm\n"
                   "TORQUE FEEDBACK: %f Nm\n"
                   "RMS UPTIME: %f s\n",
                   data->commanded_torque / 10.,
                   data->torque_feedback / 10.,
                   data->power_on_timer * 0.003);
            current_status.mc_torque_timer_information = *data;
            break;
        }
        case MC_MOD_INDEX_FLUX_WEAKENING_OUT_INFO:
            break;
        case MC_FIRM_INFO:
            break;
        // OBSOLETE
        // case ID_MC_DIAGNOSTIC_DATA:
        //     break;
        case MC_COMM_MSG: {
            MCCommMsg_t *data = &msg->contents.mc_command_message;
            // TODO add the other members of this struct??
            printf("FCU REQUESTED TORQUE: %f\n",
                   data->torque_command / 10.);
            current_status.mc_command_message = *data;
        }
        case MC_RW_PARAM_COMM:
            break;
        case MC_RW_PARAM_RESP:
            break;
        default:
            fprintf(stderr, "Error: unknown message type!\n");
            for (int i = 0; i < sizeof(*msg); i++) {
                fprintf(stderr, "%hhX ", ((char*)msg)[i]);
            }
            fprintf(stderr, "\n");
    }
}

#define ADDRESS     "tcp://hytech-telemetry.ryangallaway.me:1883"
#define CLIENTID    "HyTechCANReceiver"
#define TOPIC       "hytech_car/telemetry"
#define QOS         1
#define TIMEOUT     10000L

static void connection_lost(void *context, char *cause)
{
    printf("Connection lost: %s\n", cause);
}

static void msg_delivered(void *context, MQTTClient_deliveryToken dt)
{
    // Do nothing for now.
}

static int msg_arrived(void *context, char *topic_name, int topic_len,
        MQTTClient_message *msg)
{
    char *payload_str = msg->payload;
    int payload_len = msg->payloadlen;
    for (int i = 0; payload_str[i] != 0; i++) {
        if (payload_str[i] == ',') {
            payload_str[i] = 0;
            uint64_t timestamp = atoll(payload_str);
            telemMsg_t payload;
            cobs_decode((uint8_t *)&payload_str[i + 1], 32, (uint8_t *)&payload);
            uint16_t checksum_calc = fletcher16((uint8_t *)&payload, sizeof(payload));
            if (false) {//payload.checksum != checksum_calc) {
                fprintf(stderr, "Error: checksum mismatch: "
                        "calculated: %hu received: %hu\n",
                        checksum_calc, payload.checksum);
                goto cleanup;
            }
            process_message(timestamp, &payload);
            goto cleanup;
        }
    }
    fprintf(stderr, "Message formatted improperly\n");

cleanup:
    MQTTClient_freeMessage(&msg);
    MQTTClient_free(topic_name);
    return 1;
}

int main(int argc, char* argv[])
{
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;
    int ch;
    MQTTClient_create(&client, ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    MQTTClient_setCallbacks(client, NULL, connection_lost,
                            msg_arrived, msg_delivered);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }
    printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n"
           "Press Q<Enter> to quit\n\n", TOPIC, CLIENTID, QOS);
    MQTTClient_subscribe(client, TOPIC, QOS);
    while (1) if (getchar() == 'q') break;
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    return rc;
}

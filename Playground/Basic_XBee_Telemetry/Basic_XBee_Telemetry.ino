/*
 * HyTech 2019 Vehicle Telemetry Control Unit
 * Send wireless telemetry data via XBee.
 * Configured for Main Control Unit rev6
 */
#include <HyTech_FlexCAN.h>
#include <HyTech_CAN.h>
#include <kinetis_flexcan.h>
#include <Metro.h>
#include <XBTools.h>
#include <TimeLib.h>

/*
 * Pin definitions
 */

#define XB Serial2

/*
 * Constant definitions
 */
#define XBEE_PKT_LEN 15

/*
 * Timers
 */
Metro timer_bms_print_fault = Metro(500);
Metro timer_debug_bms_status = Metro(1000);
Metro timer_debug_bms_temperatures = Metro(3000);
Metro timer_debug_bms_detailed_temperatures = Metro(3000);
Metro timer_debug_bms_voltages = Metro(1000);
Metro timer_debug_bms_detailed_voltages = Metro(3000);
Metro timer_debug_rms_command_message = Metro(200);
Metro timer_debug_rms_current_information = Metro(100);
Metro timer_debug_rms_fault_codes = Metro(2000);
Metro timer_debug_rms_internal_states = Metro(2000);
Metro timer_debug_rms_motor_position_information = Metro(100);
Metro timer_debug_rms_temperatures_1 = Metro(3000);
Metro timer_debug_rms_temperatures_3 = Metro(3000);
Metro timer_debug_rms_torque_timer_information = Metro(200);
Metro timer_debug_rms_voltage_information = Metro(100);
Metro timer_debug_fcu_status = Metro(2000);
Metro timer_debug_fcu_readings = Metro(200);
Metro timer_debug_mcu_status = Metro(2000);
Metro timer_debug_mcu_pedal_readings = Metro(200);
Metro timer_detailed_voltages = Metro(1000);
Metro timer_imd_print_fault = Metro(500);
Metro timer_status_send = Metro(100);
Metro timer_status_send_xbee = Metro(2000);

/*
 * Global variables
 */
RCU_status rcu_status;
MC_command_message mc_command_message;
MC_temperatures_1 mc_temperatures_1;
MC_temperatures_3 mc_temperatures_3;
MC_motor_position_information mc_motor_position_information;
MC_current_information mc_current_information;
MC_voltage_information mc_voltage_information;
MC_internal_states mc_internal_states;
MC_fault_codes mc_fault_codes;
MC_torque_timer_information mc_torque_timer_information;
BMS_voltages bms_voltages;
BMS_detailed_voltages bms_detailed_voltages[8][3];
BMS_temperatures bms_temperatures;
BMS_detailed_temperatures bms_detailed_temperatures[8];
BMS_status bms_status;
FCU_status fcu_status;
FCU_readings fcu_readings;
MCU_status mcu_status;
MCU_pedal_readings mcu_pedal_readings;

FlexCAN CAN(500000);
static CAN_message_t rx_msg;
static CAN_message_t tx_msg;
static CAN_message_t xb_msg;

void setup() {

    setSyncProvider(getTeensy3Time); // Registers Teensy RTC as system time

    Serial.begin(115200); // Init serial for PC communication
    XB.begin(115200); // Init serial for XBee communication
    CAN.begin(); // Init CAN for vehicle communication

    /* Configure CAN rx interrupt */
    interrupts();
    NVIC_ENABLE_IRQ(IRQ_CAN0_MESSAGE);                                   // Enables interrupts on the teensy for CAN messages
    attachInterruptVector(IRQ_CAN0_MESSAGE, parse_can_message);          // Attaches parse_can_message() as ISR
    FLEXCAN0_IMASK1 = FLEXCAN_IMASK1_BUF5M;                             // Allows "CAN message arrived" bit in flag register to throw interrupt
    /* Configure CAN rx interrupt */

    delay(100);
    Serial.println("CAN system, serial communication, and XBee initialized");
    Serial.print("Current RTC time: ");
    digitalClockDisplay();

}

void loop() {
    /*
     * Handle incoming CAN messages
     */
    send_xbee();

    /*if (timer_detailed_voltages.check()) {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 3; j++) {
                Serial.print("IC ");
                Serial.print(i);
                Serial.print(" Group ");
                Serial.print(j);
                Serial.print(" V0: ");
                Serial.print(bms_detailed_voltages[i][j].get_voltage(0) / (double) 10000, 4);
                Serial.print(" V1: ");
                Serial.print(bms_detailed_voltages[i][j].get_voltage(1) / (double) 10000, 4);
                Serial.print(" V2: ");
                Serial.println(bms_detailed_voltages[i][j].get_voltage(2) / (double) 10000, 4);
            }
        }
        Serial.println();
    }*/


}

void parse_can_message() {
    while (CAN.read(rx_msg)) {
        // OBSOLETE
        // if (rx_msg.id == ID_FCU_STATUS) {
        //     fcu_status.load(rx_msg.buf);
        // }
        if (rx_msg.id == FCU_READ) {
            fcu_readings.load(rx_msg.buf);
        }
        if (rx_msg.id == MC_COMM_MSG) {
            mc_command_message.load(rx_msg.buf);
        }
        if (rx_msg.id == MC_TEMP_1) {
            mc_temperatures_1.load(rx_msg.buf);
        }
        if (rx_msg.id == MC_TEMP_3) {
            mc_temperatures_3.load(rx_msg.buf);
        }
        if (rx_msg.id == MC_MOTOR_POS_INFO) {
            mc_motor_position_information.load(rx_msg.buf);
        }
        if (rx_msg.id == MC_CURR_INFO) {
            mc_current_information.load(rx_msg.buf);
        }
        if (rx_msg.id == MC_VOLT_INFO) {
            mc_voltage_information.load(rx_msg.buf);
        }
        if (rx_msg.id == MC_INT_STATES) {
            mc_internal_states.load(rx_msg.buf);
        }
        if (rx_msg.id == MC_FAULT_CODES) {
            mc_fault_codes.load(rx_msg.buf);
        }
        if (rx_msg.id == MC_TOR_TIMER_INFO) {
            mc_torque_timer_information.load(rx_msg.buf);
        }
        if (rx_msg.id == BMS_VOLT) {
            bms_voltages.load(rx_msg.buf);
        }
        if (rx_msg.id == BMS_TEMP) {
            bms_temperatures.load(rx_msg.buf);
        }
        if (rx_msg.id == BMS_DET_TEMP) {
            BMS_detailed_temperatures temp = BMS_detailed_temperatures(rx_msg.buf);
            bms_detailed_temperatures[temp.get_ic_id()].load(rx_msg.buf);
        }
        if (rx_msg.id == BMS_STAT) {
            bms_status.load(rx_msg.buf);
        }
        if (rx_msg.id == BMS_DET_VOLT) {
            BMS_detailed_voltages temp = BMS_detailed_voltages(rx_msg.buf);
            bms_detailed_voltages[temp.get_ic_id()][temp.get_group_id()].load(rx_msg.buf);
        }
        if (rx_msg.id == MC_STAT) {
            mcu_status.load(rx_msg.buf);
        }
        if (rx_msg.id == MCU_PEDAL_READ) {
            mcu_pedal_readings.load(rx_msg.buf);
        }
    }
}

/**
 * Writes data currently in global xb_msg variable to the Xbee serial bus.
 * Calculates Fletcher checksum and byte-stuffs so that messages are
 * delimited by 0x0 bytes.
 *
 * returns: number of bytes written to the Xbee serial bus
 */
int write_xbee_data() {
    /*
     * DECODED FRAME STRUCTURE:
     * [ msg id (4) | msg len (1) | msg contents (8) | checksum (2) ]
     * ENCODED FRAME STRUCTURE:
     * [ fletcher (1) | msg id (4) | msg len (1) | msg contents (8) | checksum (2) | delimiter (1) ]
     */
    uint8_t xb_buf[XBEE_PKT_LEN];
    memcpy(xb_buf, &xb_msg.id, sizeof(xb_msg.id));        // msg id
    memcpy(xb_buf + sizeof(xb_msg.id), &xb_msg.len, sizeof(uint8_t));     // msg len
    memcpy(xb_buf + sizeof(xb_msg.id) + sizeof(uint8_t), xb_msg.buf, xb_msg.len); // msg contents

    // calculate checksum
    uint16_t checksum = fletcher16(xb_buf, XBEE_PKT_LEN - 2);
    Serial.print("CHECKSUM: ");
    Serial.println(checksum, HEX);
    memcpy(&xb_buf[XBEE_PKT_LEN - 2], &checksum, sizeof(uint16_t));

    for (int i = 0; i < XBEE_PKT_LEN; i++) {
      Serial.print(xb_buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    uint8_t cobs_buf[2 + XBEE_PKT_LEN];
    cobs_encode(xb_buf, XBEE_PKT_LEN, cobs_buf);
    cobs_buf[XBEE_PKT_LEN+1] = 0x0;

    for (int i = 0; i < XBEE_PKT_LEN+2; i++) {
      Serial.print(cobs_buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    int written = XB.write(cobs_buf, 2 + XBEE_PKT_LEN);
    Serial.print("Wrote ");
    Serial.print(written);
    Serial.println(" bytes");

    memset(xb_buf, 0, sizeof(CAN_message_t));

    return written;
}

void send_xbee() {
    if (timer_debug_rms_temperatures_1.check()) {
        mc_temperatures_1.write(xb_msg.buf);
        xb_msg.len = sizeof(MCTemp1_t);
        xb_msg.id = MC_TEMP_1;
        write_xbee_data();
    }

    if (timer_debug_rms_temperatures_3.check()) {
        mc_temperatures_3.write(xb_msg.buf);
        xb_msg.len = sizeof(MCTemp1_3);
        xb_msg.id = MC_TEMP_3;
        write_xbee_data();
    }

    if (timer_debug_rms_motor_position_information.check()) {
        mc_motor_position_information.write(xb_msg.buf);
        xb_msg.len = sizeof(MCMotorPosInfo_t);
        xb_msg.id = MC_MOTOR_POS_INFO;
        write_xbee_data();
    }

    if (timer_debug_rms_current_information.check()) {
        mc_current_information.write(xb_msg.buf);
        xb_msg.len = sizeof(MCCurrInfo_t);
        xb_msg.id = MC_CURR_INFO;
        write_xbee_data();
    }

    if (timer_debug_rms_voltage_information.check()) {
        mc_voltage_information.write(xb_msg.buf);
        xb_msg.len = sizeof(MCVoltInfo_t);
        xb_msg.id = MC_VOLT_INFO;
        write_xbee_data();
    }

    if (timer_debug_rms_internal_states.check()) {
        mc_internal_states.write(xb_msg.buf);
        xb_msg.len = sizeof(MCIntStates_t);
        xb_msg.id = MC_INT_STATES;
        write_xbee_data();
    }

    if (timer_debug_rms_fault_codes.check()) {
        mc_fault_codes.write(xb_msg.buf);
        xb_msg.len = sizeof(MCFaultCodes_t);
        xb_msg.id = MC_FAULT_CODES;
        write_xbee_data();
    }

    if (timer_debug_rms_torque_timer_information.check()) {
        mc_torque_timer_information.write(xb_msg.buf);
        xb_msg.len = sizeof(MCTorTimerInfo_t);
        xb_msg.id = MC_TOR_TIMER_INFO;
        write_xbee_data();
    }

    if (timer_debug_bms_voltages.check()) {
        bms_voltages.write(xb_msg.buf);
        xb_msg.len = sizeof(BMSVolt_t);
        xb_msg.id = BMS_VOLT;
        write_xbee_data();
    }

    if (timer_debug_bms_detailed_voltages.check()) {
        for (int ic = 0; ic < 8; ic++) {
            for (int group = 0; group < 3; group++) {
                bms_detailed_voltages[ic][group].write(xb_msg.buf);
                xb_msg.len = sizeof(BMSDetVolt_t);
                xb_msg.id = BMS_DET_VOLT;
                write_xbee_data();
            }
        }
    }

    if (timer_debug_bms_temperatures.check()) {
        bms_temperatures.write(xb_msg.buf);
        xb_msg.len = sizeof(BMSTemp_t);
        xb_msg.id = BMS_TEMP;
        write_xbee_data();
    }

    if (timer_debug_bms_detailed_temperatures.check()) {
        for (int ic = 0; ic < 8; ic++) {
            bms_detailed_temperatures[ic].write(xb_msg.buf);
            xb_msg.len = sizeof(BMSDetTemp_t);
            xb_msg.id = BMS_DET_TEMP;
            write_xbee_data();
        }
    }

    if (timer_debug_bms_status.check()) {
        bms_status.write(xb_msg.buf);
        xb_msg.len = sizeof(BMSStat_t);
        xb_msg.id = BMS_STAT;
        write_xbee_data();
    }

    // OBSOLETE
    // if (timer_debug_fcu_status.check()) {
    //     fcu_status.write(xb_msg.buf);
    //     xb_msg.len = sizeof(CAN_message_fcu_status_t);
    //     xb_msg.id = ID_FCU_STATUS;
    //     write_xbee_data();
    // }

    if (timer_debug_fcu_readings.check()) {
        fcu_readings.write(xb_msg.buf);
        xb_msg.len = sizeof(FCURead_t);
        xb_msg.id = FCU_READ;
        write_xbee_data();
    }

    if (timer_debug_rms_command_message.check()) {
        mc_command_message.write(xb_msg.buf);
        xb_msg.len = sizeof(MCCommMsg_t);
        xb_msg.id = MC_COMM_MSG;
        write_xbee_data();
    }

    if (timer_debug_mcu_status.check()) {
        mcu_status.write(xb_msg.buf);
        xb_msg.len = sizeof(MCUStat_t);
        xb_msg.id = MC_STAT;
        write_xbee_data();
    }

    if (timer_debug_mcu_pedal_readings.check()) {
        mcu_pedal_readings.write(xb_msg.buf);
        xb_msg.len = sizeof(MCUPedalRead_t);
        xb_msg.id = MCU_PEDAL_READ;
        write_xbee_data();
    }
}

time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

void digitalClockDisplay() {
    // digital clock display of the time
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year()); 
    Serial.println(); 
}

void printDigits(int digits){
    // utility function for digital clock display: prints preceding colon and leading 0
    Serial.print(":");
    if(digits < 10)
        Serial.print('0');
    Serial.print(digits);
}

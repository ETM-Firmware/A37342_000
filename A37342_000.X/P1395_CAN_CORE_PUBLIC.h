#ifndef __P1395_CAN_CORE_PUBLIC_H
#define __P1395_CAN_CORE_PUBLIC_H


#define ETM_CAN_HIGH_ENERGY           1
#define ETM_CAN_LOW_ENERGY            0


#define ETM_CAN_ADDR_HV_LAMBDA_BOARD                                    0x0000
#define ETM_CAN_ADDR_ION_PUMP_BOARD                                     0x0001
#define ETM_CAN_ADDR_AFC_CONTROL_BOARD                                  0x0002
#define ETM_CAN_ADDR_COOLING_INTERFACE_BOARD                            0x0003
#define ETM_CAN_ADDR_HEATER_MAGNET_BOARD                                0x0004
#define ETM_CAN_ADDR_GUN_DRIVER_BOARD                                   0x0005
#define ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD                            0x0006
#define ETM_CAN_ADDR_TARGET_CURRENT_BOARD                               0x0007
#define ETM_CAN_ADDR_DOSE_MONITOR_BOARD                                 0x0008
#define ETM_CAN_ADDR_PFN_BOARD                                          0x0009
#define ETM_CAN_ADDR_ETHERNET_BOARD                                     0x000F

#define DISCRETE_CMD_BUFFER_EMPTY                                       0x00
#define DISCRETE_CMD_AFC_DO_AUTO_ZERO                                   0x01
#define DISCRETE_CMD_AFC_SELECT_MANUAL_MODE                             0x02
#define DISCRETE_CMD_AFC_SELECT_AUTOMATIC_MODE                          0x03
#define DISCRETE_CMD_COOLING_RESET_BOTTLE_COUNT                         0x04


#define CAN_PORT_2  2
#define CAN_PORT_1  1


#define SYSTEM_CONFIGURATION_6_4_R                   0xA64A
#define SYSTEM_CONFIGURATION_6_4_M                   0xA64B
#define SYSTEM_CONFIGURATION_6_4_S                   0xA64C
#define SYSTEM_CONFIGURATION_2_5_R                   0xA25A




typedef struct {
  unsigned control_not_ready:1;
  unsigned control_not_configured:1;
  unsigned control_self_check_error:1;
  unsigned control_3_unused:1;
  unsigned control_4_unused:1;
  unsigned control_5_unused:1;
  unsigned control_6_unused:1;
  unsigned control_7_unused:1;
  
  unsigned notice_0:1;
  unsigned notice_1:1;
  unsigned notice_2:1;
  unsigned notice_3:1;
  unsigned notice_4:1;
  unsigned notice_5:1;
  unsigned notice_6:1;
  unsigned notice_7:1;
} ETMCanStatusRegisterControlAndNoticeBits;

typedef struct {
  unsigned fault_0:1;
  unsigned fault_1:1;
  unsigned fault_2:1;
  unsigned fault_3:1;
  unsigned fault_4:1;
  unsigned fault_5:1;
  unsigned fault_6:1;
  unsigned fault_7:1;
  unsigned fault_8:1;
  unsigned fault_9:1;
  unsigned fault_A:1;
  unsigned fault_B:1;
  unsigned fault_C:1;
  unsigned fault_D:1;
  unsigned fault_E:1;
  unsigned fault_F:1;
} ETMCanStatusRegisterFaultBits;

typedef struct {
  unsigned warning_0:1;
  unsigned warning_1:1;
  unsigned warning_2:1;
  unsigned warning_3:1;
  unsigned warning_4:1;
  unsigned warning_5:1;
  unsigned warning_6:1;
  unsigned warning_7:1;
  unsigned warning_8:1;
  unsigned warning_9:1;
  unsigned warning_A:1;
  unsigned warning_B:1;
  unsigned warning_C:1;
  unsigned warning_D:1;
  unsigned warning_E:1;
  unsigned warning_F:1;
} ETMCanStatusRegisterWarningBits;

typedef struct {
  unsigned not_logged_0:1;
  unsigned not_logged_1:1;
  unsigned not_logged_2:1;
  unsigned not_logged_3:1;
  unsigned not_logged_4:1;
  unsigned not_logged_5:1;
  unsigned not_logged_6:1;
  unsigned not_logged_7:1;
  unsigned not_logged_8:1;
  unsigned not_logged_9:1;
  unsigned not_logged_A:1;
  unsigned not_logged_B:1;
  unsigned not_logged_C:1;
  unsigned not_logged_D:1;
  unsigned not_logged_E:1;
  unsigned not_logged_F:1;
} ETMCanStatusRegisterNotLoggedBits;



typedef struct {
  ETMCanStatusRegisterControlAndNoticeBits   control_notice_bits;  // 16 bits
  ETMCanStatusRegisterFaultBits              fault_bits;    // 16 bits
  ETMCanStatusRegisterWarningBits            warning_bits;  // 16 bits
  ETMCanStatusRegisterNotLoggedBits          not_logged_bits;   // 16 bits
} ETMCanStatusRegister;




// DPARKER status needs to be public, but not the rest of this.  Is this possible?
// DPARKER does the can module need to handle the faults someday???  what would this look like

// --------------- Board Logging Data ----------------------- //

typedef struct {
  ETMCanStatusRegister    status;             // This is 4 words of status data for the slave

  // Can data log - 0x00Z -> 0x05Z
  unsigned int            log_data[24];       // This is 24 words (6 registers) of logging data passed to the GUI
                                              // This data should be managed by the user application program
                                              // use #define to map the relevant variables to these locations

  // Can data log - 0x06Z
  unsigned int            device_id_high_word;
  unsigned int            device_id_low_word;
  unsigned int            device_id_dash_number;
  unsigned int            device_rev_2x_ASCII;

  // Can data log - 0x07Z
  unsigned int            device_serial_number;
  unsigned int            device_firmware_rev_agile;
  unsigned int            device_firmware_branch;
  unsigned int            device_firmware_branch_rev;

  unsigned int            connection_timeout;                // On the ECB this is used to flag if the board connection has timed out or not.
  unsigned long           time_last_status_message_recieved; // On the ECB this is used to track the last time a status message was recieved from this board.  
  unsigned int            spare;
                                            // This should be read only to the user application program
} ETMCanBoardData;





#endif









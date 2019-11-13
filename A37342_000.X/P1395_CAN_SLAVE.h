#ifndef __P1395_CAN_SLAVE_PUBLIC_H
#define __P1395_CAN_SLAVE_PUBLIC_H

#include "P1395_CAN_CORE_PUBLIC.h"


#define P1395_CAN_SLAVE_VERSION   0x0310


//------------ SLAVE PUBLIC FUNCTIONS AND VARIABLES ------------------- //

typedef struct {
  unsigned int  data[68];
  unsigned int  average;
  unsigned int  integral;
  unsigned char pulse_count;
  unsigned      arc_this_pulse:1;
  unsigned      low_this_pulse:1;
  unsigned      data_full:1;
  unsigned      unused:5;
} TYPE_PULSE_DATA;




//------------ SLAVE PUBLIC FUNCTIONS AND VARIABLES ------------------- //

// Public Functions
void ETMCanSlaveInitialize(unsigned int requested_can_port, unsigned long fcy, unsigned int etm_can_address,
			   unsigned long can_operation_led, unsigned int can_interrupt_priority,
			   unsigned long flash_led, unsigned long not_ready_led,
			   unsigned long trigger_pin);
/*
  This is called once when the processor starts up to initialize the can bus and all of the can variables
*/


void ETMCanSlaveLoadConfiguration(unsigned long agile_id, unsigned int agile_dash,
				  unsigned int firmware_agile_rev, unsigned int firmware_branch, 
				  unsigned int firmware_branch_rev);
/*
  This is called once when the prcoessor starts up to load the board configuration into RAM so it can be sent over CAN to the ECB
*/


void ETMCanSlaveDoCan(void);
/*
  This function should be called every time through the processor execution loop (which should be on the order of 10-100s of uS)
  If will do the following
  1) Look for an execute can commands
  2) Look for changes in status bits, update the Fault/Inhibit bits and send out a new status command if nessesary
  3) Send out regularly schedule communication (On slaves this is status update and logging data)
*/


unsigned char ETMCanSlaveGetDiscreteCMD(void);
/*
  Returns the next discrete command id to be executed.
  If there are no commands, will return 0.
*/

unsigned int ETMCanSlaveGetComFaultStatus(void);
/*
  Returns 0 if communication with the ECB is normal
  Returns 1 if the communication with the ECB has timed out
  On com fault all boards should inhibit triggering and shut down to a safe level
*/


// ------------- SYNC MESSAGE COMMANDS ------------------ //
//unsigned int ETMCanSlaveGetSyncMsgResetEnable(void); // DPARKER CHANGE HOW THIS OPERATES
/*
  returns 0xFFFF if reset is active, 0 otherwise
*/

unsigned int ETMCanSlaveGetSyncMsgHighSpeedLogging(void);
/*
  returns 0xFFFF if high speed logging is enabled, 0 otherwise
*/

unsigned int ETMCanSlaveGetSyncMsgCoolingFault(void);
/*
  returns 0xFFFF if there is cooling fault, 0 otherwise
  boards that require cooling should shutdown on this
*/

unsigned int ETMCanSlaveGetSyncMsgSystemHVDisable(void);
/*
  returns 0xFFFF if high voltage shutdown is requested, 0 otherwise
  boards that produce high voltage should shut down high voltage on this
*/

unsigned int ETMCanSlaveGetSyncMsgGunDriverDisableHeater(void);
/*
  returns 0xFFFF if the ECB has requested a shutdown of the gun heater
  the gun driver should shut down the heater (and everything else it needs to safely) on this
*/

unsigned int ETMCanSlaveGetSyncMsgSystemInitializationActive(void);
/*
  returns 0xFFFF if the ECB is in intialization mode, 0 otherwise
  because the ECB sends out triggers while initializing, boards should stay in their initialzation state while this bit is set
*/


unsigned char ETMCanSlaveGetPulseLevel(void);
/*
  Returns the dose level of the next pulse
  This is not exactly what is in the sync message because the sync message also contains interleaving information.
*/

#define DOSE_SELECT_DOSE_LEVEL_0           0x1
#define DOSE_SELECT_DOSE_LEVEL_1           0x2
#define DOSE_SELECT_DOSE_LEVEL_2           0x3
#define DOSE_SELECT_DOSE_LEVEL_3           0x4

unsigned char ETMCanSlaveGetPulseCount(void);
/*
  Returns the count from the sync message
*/

void ETMCanSlaveTriggerRecieved(unsigned char pulse_count_at_trigger);
/*
  Call this once after a trigger to automatically update pulse count and level based on the mode of operation.
  That way if a sync_level command is missed the system will still pulse at the correct level on the next trigger.
*/


// ------------ SCOPE / DATA LOGGING FUNCTIONS ----------------------- //
void ETMCanSlaveSetLogDataRegister(unsigned int log_register, unsigned int data_for_log);
/*
  There are 24 logging registers, 0x00->0x17
  This stores the value to a particular data log register
  This information is available on the GUI and is used to display the system parameters
*/


void ETMCanSlaveSetDebugRegister(unsigned int debug_register, unsigned int debug_value);
/*
  There are 16 debug registers, 0x0->0xF
  This stores the value to a particular register
  This information is available on the GUI and should be used to display real time inforamtion to help in debugging.
*/


void ETMCanSlaveLogPulseData(unsigned int pulse_count, unsigned int log_data_1, unsigned int log_data_0);
/*
  This is used to log pulse by pulse data
  
  If the pulse count is even, the data will be saved.
  If the pusle count is odd, the the data will be sent out
*/


void ETMCanSlaveLogPulseCurrent(TYPE_PULSE_DATA *pulse_data);
/*
  This is used to log magnetron or target current for a particular pulse
  The Can module can not save every sample, so it will decide which data to save
*/

void ETMCanSlaveSetScopeDataAddress(unsigned int scope_channel, unsigned int *data_address);
/*
  scope_channel must be less than 16
  This is used to set the data monitoring address for each of the scope channels.
  To make a particular address visable to the scope, assign it to one of the 16 channels
*/

void ETMCanSlaveLogHVVmonData(unsigned int sp4, unsigned int sp3, unsigned int sp2, unsigned int sp1, unsigned int sp0);
/*
  This will compress the 5 12 bit readings sp0 -> sp4 into 4 words and send the data to the ECB.
*/

unsigned int ETMCanSlaveGetSetting(unsigned char setting_select);
/*
  Returns the dose settings specified below
*/
#define HVPS_SET_POINT_DOSE_3                        0
#define HVPS_SET_POINT_DOSE_2                        1
#define HVPS_SET_POINT_DOSE_1                        2
#define HVPS_SET_POINT_DOSE_0                        3

#define ETECTROMANGET_SET_POINT_DOSE_3               4
#define ETECTROMANGET_SET_POINT_DOSE_2               5
#define ETECTROMANGET_SET_POINT_DOSE_1               6
#define ETECTROMANGET_SET_POINT_DOSE_0               7

#define GUN_DRIVER_PULSE_TOP_DOSE_3                  8
#define GUN_DRIVER_PULSE_TOP_DOSE_2                  9
#define GUN_DRIVER_PULSE_TOP_DOSE_1                  10
#define GUN_DRIVER_PULSE_TOP_DOSE_0                  11

#define GUN_DRIVER_CATHODE_DOSE_3                    12
#define GUN_DRIVER_CATHODE_DOSE_2                    13
#define GUN_DRIVER_CATHODE_DOSE_1                    14
#define GUN_DRIVER_CATHODE_DOSE_0                    15

#define AFC_HOME_POSITION_DOSE_3                     16
#define AFC_HOME_POSITION_DOSE_2                     17
#define AFC_HOME_POSITION_DOSE_1                     18
#define AFC_HOME_POSITION_DOSE_0                     19

#define MAGNETRON_HEATER_SCALED_HEATER_CURRENT       20
#define AFC_AUX_CONTROL_OR_OFFSET                    21
#define GUN_DRIVER_BIAS_VOLTAGE                      22
#define GUN_DRIVER_HEATER_VOLTAGE                    23

#define AFC_MANUAL_TARGET_POSTION                    24
#define SYSTEM_CONFIGURATION_SELECT                  25
#define SPARE_ECB_DATA_A                             26
#define SPARE_ECB_DATA_B                             27

#define AUX_SET_POINT_7                              28
#define AUX_SET_POINT_6                              29
#define AUX_SET_POINT_5                              30
#define AUX_SET_POINT_4                              31

#define AUX_SET_POINT_3                              32
#define AUX_SET_POINT_2                              33
#define AUX_SET_POINT_1                              34
#define AUX_SET_POINT_0                              35



void ETMCanSlaveStatusUpdateBitNotReady(unsigned int value);
#define NOT_READY         1
#define READY             0

void ETMCanSlaveStatusSetSelfCheckError(void);

void ETMCanSlaveStatusSetNoticeBit(unsigned int notice_bit);

unsigned int ETMCanSlaveStatusCheckNotConfigured(void);
// Will return 0xFFFF if not configured, 0 otherwise

void ETMCanSlaveStatusUpdateFaultBit(unsigned int fault_bit, unsigned int value);

unsigned int ETMCanSlaveStatusReadFaultBit(unsigned int fault_bit);
// Will return 0xFFFF if the fault bit is set, 0 otherwise

unsigned int ETMCanSlaveStatusReadFaultRegister(void);
// Will return 0xFFFF if ANY fault bit is set, 0 otherwise

void ETMCanSlaveStatusUpdateLoggedBit(unsigned int logged_bit, unsigned int value);

unsigned int ETMCanSlaveStatusReadLoggedBit(unsigned int logged_bit);
// Will return 0xFFFF if the bit is set, 0 otherwise

void ETMCanSlaveStatusUpdateNotLoggedBit(unsigned int not_logged_bit, unsigned int value);

unsigned int ETMCanSlaveStatusReadNotLoggedBit(unsigned int not_logged_bit);
// Will return 0xFFFF if the bit is set, 0 otherwise




#define _FAULT_CAN_COMMUNICATION      0x0001
#define _FAULT_1                      0x0002
#define _FAULT_2                      0x0004 
#define _FAULT_3                      0x0008
#define _FAULT_4                      0x0010
#define _FAULT_5                      0x0020
#define _FAULT_6                      0x0040
#define _FAULT_7                      0x0080
#define _FAULT_8                      0x0100
#define _FAULT_9                      0x0200
#define _FAULT_A                      0x0400
#define _FAULT_B                      0x0800
#define _FAULT_C                      0x1000
#define _FAULT_D                      0x2000
#define _FAULT_E                      0x4000
#define _FAULT_F                      0x8000

#define _NOTICE_0                     0x0100
#define _NOTICE_1                     0x0200
#define _NOTICE_2                     0x0400
#define _NOTICE_3                     0x0800
#define _NOTICE_4                     0x1000
#define _NOTICE_5                     0x2000
#define _NOTICE_6                     0x4000
#define _NOTICE_7                     0x8000

#define _LOGGED_STATUS_0              0x0001
#define _LOGGED_STATUS_1              0x0002
#define _LOGGED_STATUS_2              0x0004
#define _LOGGED_STATUS_3              0x0008
#define _LOGGED_STATUS_4              0x0010
#define _LOGGED_STATUS_5              0x0020
#define _LOGGED_STATUS_6              0x0040
#define _LOGGED_STATUS_7              0x0080
#define _LOGGED_STATUS_8              0x0100
#define _LOGGED_STATUS_9              0x0200
#define _LOGGED_STATUS_A              0x0400
#define _LOGGED_STATUS_B              0x0800
#define _LOGGED_STATUS_C              0x1000
#define _LOGGED_STATUS_D              0x2000
#define _LOGGED_STATUS_E              0x4000
#define _LOGGED_STATUS_F              0x8000

#define _NOT_LOGGED_STATUS_0          0x0001
#define _NOT_LOGGED_STATUS_1          0x0002
#define _NOT_LOGGED_STATUS_2          0x0004
#define _NOT_LOGGED_STATUS_3          0x0008
#define _NOT_LOGGED_STATUS_4          0x0010
#define _NOT_LOGGED_STATUS_5          0x0020
#define _NOT_LOGGED_STATUS_6          0x0040
#define _NOT_LOGGED_STATUS_7          0x0080
#define _NOT_LOGGED_STATUS_8          0x0100
#define _NOT_LOGGED_STATUS_9          0x0200
#define _NOT_LOGGED_STATUS_A          0x0400
#define _NOT_LOGGED_STATUS_B          0x0800
#define _NOT_LOGGED_STATUS_C          0x1000
#define _NOT_LOGGED_STATUS_D          0x2000
#define _NOT_LOGGED_STATUS_E          0x4000
#define _NOT_LOGGED_STATUS_F          0x8000




#endif

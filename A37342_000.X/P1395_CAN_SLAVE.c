#include <xc.h>
#include <libpic30.h>
#include "P1395_CAN_SLAVE.h"
#include "ETM.h"



unsigned int pulse_data_transmit_index = 0xFF00;
unsigned int pulse_data_to_transmit[40];



// GLOBAL VARIABLES
ETMCanBoardData           slave_board_data;            // This contains information that is always mirrored on ECB


unsigned int ETMEEPromPrivateReadSinglePage(unsigned int page_number, unsigned int *page_data);
// From ETM_EEPROM - this module only needs access to this fuction

// DOSE LEVEL DEFINITIONS

#define DOSE_SELECT_REPEATE_DOSE_LEVEL_0           0x11
#define DOSE_SELECT_REPEATE_DOSE_LEVEL_1           0x12
#define DOSE_SELECT_REPEATE_DOSE_LEVEL_2           0x13
#define DOSE_SELECT_REPEATE_DOSE_LEVEL_3           0x14

#define DOSE_SELECT_INTERLEAVE_0_1_DOSE_LEVEL_0    0x21
#define DOSE_SELECT_INTERLEAVE_0_1_DOSE_LEVEL_1    0x22
#define DOSE_SELECT_INTERLEAVE_2_3_DOSE_LEVEL_2    0x23
#define DOSE_SELECT_INTERLEAVE_2_3_DOSE_LEVEL_3    0x24


#define ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_0               0x100
#define ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_1               0x110
#define ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_2               0x120
#define ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_3               0x130
#define ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_4               0x140
#define ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_5               0x150
#define ETM_CAN_DATA_LOG_REGISTER_CONFIG_0                       0x160
#define ETM_CAN_DATA_LOG_REGISTER_CONFIG_1                       0x170

#define ETM_CAN_DATA_LOG_REGISTER_SCOPE_A                        0x180
#define ETM_CAN_DATA_LOG_REGISTER_SCOPE_B                        0x190
#define ETM_CAN_DATA_LOG_REGISTER_PULSE_SCOPE_DATA               0x1A0
#define ETM_CAN_DATA_LOG_REGISTER_HV_VMON_DATA                   0x1B0

#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_0                0x1C0
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_1                0x1D0
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_2                0x1E0
#define ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_3                0x1F0

#define ETM_CAN_DATA_LOG_REGISTER_RAM_WATCH_DATA                 0x200
#define ETM_CAN_DATA_LOG_REGISTER_STANDARD_ANALOG_DATA           0x210

#define ETM_CAN_DATA_LOG_REGISTER_CAN_DEBUG_0                    0x220
#define ETM_CAN_DATA_LOG_REGISTER_CAN_DEBUG_1                    0x230
#define ETM_CAN_DATA_LOG_REGISTER_CAN_DEBUG_2                    0x240
#define ETM_CAN_DATA_LOG_REGISTER_CAN_DEBUG_3                    0x250

#define ETM_CAN_DATA_LOG_REGISTER_EEPROM_INTERNAL_DEBUG          0x260
#define ETM_CAN_DATA_LOG_REGISTER_EEPROM_I2C_DEBUG               0x270
#define ETM_CAN_DATA_LOG_REGISTER_EEPROM_SPI_DEBUG               0x280
#define ETM_CAN_DATA_LOG_REGISTER_EEPROM_CRC_DEBUG               0x290

#define ETM_CAN_DATA_LOG_REGISTER_RESET_VERSION_DEBUG            0x2A0
#define ETM_CAN_DATA_LOG_REGISTER_I2C_SPI_SCALE_SELF_TEST_DEBUG  0x2B0
#define ETM_CAN_DATA_LOG_REGISTER_STANDARD_DEBUG_TBD_3           0x2C0
#define ETM_CAN_DATA_LOG_REGISTER_STANDARD_DEBUG_TBD_2           0x2D0
#define ETM_CAN_DATA_LOG_REGISTER_STANDARD_DEBUG_TBD_1           0x2E0
#define ETM_CAN_DATA_LOG_REGISTER_STANDARD_DEBUG_TBD_0           0x2F0

#define ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_0                  0x300
#define ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_1                  0x310
#define ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_2                  0x320
#define ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_3                  0x330
#define ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_4                  0x340
#define ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_5                  0x350
#define ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_6                  0x360
#define ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_7                  0x370


#define SLAVE_TRANSMIT_MILLISECONDS       100
#define SLAVE_TIMEOUT_MILLISECONDS        250

#define SCOPE_CHANNEL_DISABLED  0
#define SCOPE_CHANNEL_ACTIVE    1

#define LOG_DEFAULT_VALUE_SLAVE     0xFFDF
#define LOG_DEFAULT_VALUE_SLAVE_ALT 0xFFCF


#if defined(__dsPIC30F6014A__)
#define RAM_SIZE_WORDS  4096
#endif

#if defined(__dsPIC30F6010A__)
#define RAM_SIZE_WORDS  4096
#endif

#ifndef RAM_SIZE_WORDS
#define RAM_SIZE_WORDS 0
#endif



#define EEPROM_PAGE_ANALOG_CALIBRATION_0_1_2   0x04
#define EEPROM_PAGE_ANALOG_CALIBRATION_3_4_5   0x05
#define EEPROM_PAGE_ANALOG_CALIBRATION_6_7     0x06
#define EEPROM_PAGE_BOARD_CONFIGURATION        0x00
#define EEPROM_TEST_PAGE                       0x03




//Local Funcations
static void ETMCanSlaveProcessMessage(void);
static void ETMCanSlaveExecuteCMD(ETMCanMessage* message_ptr);           
static void ETMCanSlaveTimedTransmit(void);
static void ETMCanSlaveSendScopeData(void);
static void ETMCanSlaveSendStatus(void);
static void ETMCanSlaveLogData(unsigned int packet_id, unsigned int word3, unsigned int word2, unsigned int word1, unsigned int word0);
static void ETMCanSlaveCheckForTimeOut(void);
static void ETMCanSlaveSendUpdateIfNewNotReady(void);
static void ETMCanSlaveClearDebug(void);
static void Compact12BitInto16Bit(unsigned int *transmit_data, unsigned int *source_data, unsigned int count);
static void ETMCanSlaveLoadDefaultCalibration(void);
static void DoCanInterrupt(void);  //  Helper function for Can interupt handler



// -------------------- local variables -------------------------- //

static ETMCanBoardDebuggingData  etm_can_slave_debug_data;    // This information is only mirrored on ECB if this module is selected on the GUI
static ETMCanSyncMessage         etm_can_slave_sync_message;  // This is the most recent sync message recieved from the ECB

static ETMCanMessageBuffer etm_can_slave_rx_message_buffer;   // RX message buffer
static ETMCanMessageBuffer etm_can_slave_tx_message_buffer;   // TX Message buffer

static unsigned long etm_can_slave_communication_timeout_holding_var;  // Used to check for timeout of SYNC messsage from the ECB
static unsigned char etm_can_slave_com_loss;  // Keeps track of if communicantion from ECB has timed out
static unsigned int setting_data_recieved;  // This keeps track of which settings registers have been recieved 
static BUFFERBYTE64 discrete_cmd_buffer;  // Buffer for discrete commands that come in.  This is quiered and processed by the main code as needed



#define SCOPE_DATA_SELECT_SIZE         16
static unsigned int *scope_data_sources[SCOPE_DATA_SELECT_SIZE];
static unsigned int *ram_ptr_a = 0;
static unsigned int *ram_ptr_b = 0;
static unsigned int *ram_ptr_c = 0;
static unsigned int *ptr_scope_a_data_source = 0;
static unsigned int *ptr_scope_b_data_source = 0;



typedef struct {
  unsigned int hvps_set_point_dose_level_3;
  unsigned int hvps_set_point_dose_level_2;
  unsigned int hvps_set_point_dose_level_1;
  unsigned int hvps_set_point_dose_level_0;

  unsigned int electromagnet_set_point_dose_level_3;
  unsigned int electromagnet_set_point_dose_level_2;
  unsigned int electromagnet_set_point_dose_level_1;
  unsigned int electromagnet_set_point_dose_level_0;

  unsigned int gun_driver_pulse_top_voltage_dose_level_3;
  unsigned int gun_driver_pulse_top_voltage_dose_level_2;
  unsigned int gun_driver_pulse_top_voltage_dose_level_1;
  unsigned int gun_driver_pulse_top_voltage_dose_level_0;

  unsigned int gun_driver_cathode_voltage_dose_level_3;
  unsigned int gun_driver_cathode_voltage_dose_level_2;
  unsigned int gun_driver_cathode_voltage_dose_level_1;
  unsigned int gun_driver_cathode_voltage_dose_level_0;

  unsigned int afc_home_position_dose_level_3;
  unsigned int afc_home_position_dose_level_2;
  unsigned int afc_home_position_dose_level_1;
  unsigned int afc_home_position_dose_level_0;

  unsigned int magnetron_heater_scaled_heater_current;
  unsigned int afc_aux_control_or_offset;
  unsigned int gun_driver_grid_bias_voltage;
  unsigned int gun_driver_heater_voltage;
  
  unsigned int afc_manual_target_position;

  unsigned int scope_a_settings;
  unsigned int scope_b_settings;
  unsigned int scope_hv_vmon_settings;
  
} TYPE_SLAVE_DATA;

static TYPE_SLAVE_DATA slave_data;



typedef struct {
  unsigned int reset_count;
  unsigned int can_timeout_count;
} PersistentData;
static volatile PersistentData etm_can_persistent_data __attribute__ ((persistent));

typedef struct {
  unsigned int  address;
  unsigned char eeprom_intialization_error;
  unsigned char eeprom_calibration_write_error;
  unsigned long led;
  unsigned long flash_led;
  unsigned long not_ready_led;
  
} TYPE_CAN_PARAMETERS;
static TYPE_CAN_PARAMETERS can_params;


// ---------- Pointers to CAN stucutres so that we can use CAN1 or CAN2
volatile unsigned int *CXEC_ptr;
volatile unsigned int *CXINTF_ptr;
volatile unsigned int *CXRX0CON_ptr;
volatile unsigned int *CXRX1CON_ptr;
volatile unsigned int *CXTX0CON_ptr;
volatile unsigned int *CXTX1CON_ptr;
volatile unsigned int *CXTX2CON_ptr;


void ETMCanSlaveInitialize(unsigned int requested_can_port, unsigned long fcy, unsigned int etm_can_address,
			   unsigned long can_operation_led, unsigned int can_interrupt_priority,
			   unsigned long flash_led, unsigned long not_ready_led) {

  unsigned int  eeprom_page_data[16];
  
  if (ETMTickNotInitialized()) {
    ETMTickInitialize(fcy, ETM_TICK_USE_TIMER_5);
  }

  BufferByte64Initialize(&discrete_cmd_buffer);

  setting_data_recieved &= 0b00000000;
  _CONTROL_NOT_CONFIGURED = 1;
  
  etm_can_slave_debug_data.can_build_version = P1395_CAN_SLAVE_VERSION;
  etm_can_slave_debug_data.library_build_version = ETM_LIBRARY_VERSION;

  if (can_interrupt_priority > 7) {
    can_interrupt_priority = 7;
  }
  
  can_params.address = etm_can_address;
  can_params.led = can_operation_led;
  can_params.flash_led = flash_led;
  can_params.not_ready_led = not_ready_led;

  ETMPinTrisOutput(can_params.led);
  ETMPinTrisOutput(can_params.flash_led);
  ETMPinTrisOutput(can_params.not_ready_led);


  etm_can_persistent_data.reset_count++;
  
  *(unsigned int*)&etm_can_slave_sync_message.sync_0_control_word = 0;
  etm_can_slave_sync_message.pulse_count = 0;
  etm_can_slave_sync_message.next_energy_level = 0;
  etm_can_slave_sync_message.prf_from_ecb = 0;
  etm_can_slave_sync_message.scope_A_select = 0;
  etm_can_slave_sync_message.scope_B_select = 0;
  
  
  
  etm_can_slave_com_loss = 0;

  etm_can_slave_debug_data.reset_count = etm_can_persistent_data.reset_count;
  etm_can_slave_debug_data.can_timeout = etm_can_persistent_data.can_timeout_count;

  ETMCanBufferInitialize(&etm_can_slave_rx_message_buffer);
  ETMCanBufferInitialize(&etm_can_slave_tx_message_buffer);
  
  ETMPinTrisOutput(can_params.led);
  
  if (requested_can_port != CAN_PORT_2) {
    // Use CAN1
    
    CXEC_ptr     = &C1EC;
    CXINTF_ptr   = &C1INTF;
    CXRX0CON_ptr = &C1RX0CON;
    CXRX1CON_ptr = &C1RX1CON;
    CXTX0CON_ptr = &C1TX0CON;
    CXTX1CON_ptr = &C1TX1CON;
    CXTX2CON_ptr = &C1TX2CON;

    _C1IE = 0;
    _C1IF = 0;
    _C1IP = can_interrupt_priority;
    
    C1INTF = 0;
    
    C1INTEbits.RX0IE = 1; // Enable RXB0 interrupt
    C1INTEbits.RX1IE = 1; // Enable RXB1 interrupt
    C1INTEbits.TX0IE = 1; // Enable TXB0 interrupt
    C1INTEbits.ERRIE = 1; // Enable Error interrupt
  
    // ---------------- Set up CAN Control Registers ---------------- //
    
    // Set Baud Rate
    C1CTRL = CXCTRL_CONFIG_MODE_VALUE;
    while(C1CTRLbits.OPMODE != 4);
    
    if (fcy == 25000000) {
      C1CFG1 = CXCFG1_25MHZ_FCY_VALUE;    
    } else if (fcy == 20000000) {
      C1CFG1 = CXCFG1_20MHZ_FCY_VALUE;    
    } else if (fcy == 10000000) {
      C1CFG1 = CXCFG1_10MHZ_FCY_VALUE;    
    } else {
      // If you got here we can't configure the can module
      // DPARKER WHAT TO DO HERE
    }
    
    C1CFG2 = CXCFG2_VALUE;
    
    
    // Load Mask registers for RX0 and RX1
    C1RXM0SID = ETM_CAN_SLAVE_RX0_MASK;
    C1RXM1SID = ETM_CAN_SLAVE_RX1_MASK;
    
    // Load Filter registers
    C1RXF0SID = ETM_CAN_SLAVE_MSG_FILTER_RF0;
    C1RXF1SID = ETM_CAN_SLAVE_MSG_FILTER_RF1;
    C1RXF2SID = (ETM_CAN_SLAVE_MSG_FILTER_RF2 | (can_params.address << 2));
    //C1RXF3SID = ETM_CAN_MSG_FILTER_OFF;
    //C1RXF4SID = ETM_CAN_MSG_FILTER_OFF;
    //C1RXF5SID = ETM_CAN_MSG_FILTER_OFF;
    
    // Set Transmitter Mode
    C1TX0CON = CXTXXCON_VALUE_LOW_PRIORITY;
    C1TX1CON = CXTXXCON_VALUE_MEDIUM_PRIORITY;
    C1TX2CON = CXTXXCON_VALUE_HIGH_PRIORITY;
    
    C1TX0DLC = CXTXXDLC_VALUE;
    C1TX1DLC = CXTXXDLC_VALUE;
    C1TX2DLC = CXTXXDLC_VALUE;
    
    
    // Set Receiver Mode
    C1RX0CON = CXRXXCON_VALUE;
    C1RX1CON = CXRXXCON_VALUE;
    

    // Switch to normal operation
    C1CTRL = CXCTRL_OPERATE_MODE_VALUE;
    while(C1CTRLbits.OPMODE != 0);
    
    // Enable Can interrupt
    _C1IE = 1;
    _C2IE = 0;
    
  } else {
    // Use CAN2
    CXEC_ptr     = &C2EC;
    CXINTF_ptr   = &C2INTF;
    CXRX0CON_ptr = &C2RX0CON;
    CXRX1CON_ptr = &C2RX1CON;
    CXTX0CON_ptr = &C2TX0CON;
    CXTX1CON_ptr = &C2TX1CON;
    CXTX2CON_ptr = &C2TX2CON;

    _C2IE = 0;
    _C2IF = 0;
    _C2IP = can_interrupt_priority;
    
    C2INTF = 0;
    
    C2INTEbits.RX0IE = 1; // Enable RXB0 interrupt
    C2INTEbits.RX1IE = 1; // Enable RXB1 interrupt
    C2INTEbits.TX0IE = 1; // Enable TXB0 interrupt
    C2INTEbits.ERRIE = 1; // Enable Error interrupt
  
    // ---------------- Set up CAN Control Registers ---------------- //
    
    // Set Baud Rate
    C2CTRL = CXCTRL_CONFIG_MODE_VALUE;
    while(C2CTRLbits.OPMODE != 4);
    
    if (fcy == 25000000) {
      C2CFG1 = CXCFG1_25MHZ_FCY_VALUE;    
    } else if (fcy == 20000000) {
      C2CFG1 = CXCFG1_20MHZ_FCY_VALUE;    
    } else if (fcy == 10000000) {
      C2CFG1 = CXCFG1_10MHZ_FCY_VALUE;    
    } else {
      // If you got here we can't configure the can module
      // DPARKER WHAT TO DO HERE
    }
    
    C2CFG2 = CXCFG2_VALUE;
    
    
    // Load Mask registers for RX0 and RX1
    C2RXM0SID = ETM_CAN_SLAVE_RX0_MASK;
    C2RXM1SID = ETM_CAN_SLAVE_RX1_MASK;
    
    // Load Filter registers
    C2RXF0SID = ETM_CAN_SLAVE_MSG_FILTER_RF0;
    C2RXF1SID = ETM_CAN_SLAVE_MSG_FILTER_RF1;
    C2RXF2SID = (ETM_CAN_SLAVE_MSG_FILTER_RF2 | (can_params.address << 2));
    //C2RXF3SID = ETM_CAN_MSG_FILTER_OFF;
    //C2RXF4SID = ETM_CAN_MSG_FILTER_OFF;
    //C2RXF5SID = ETM_CAN_MSG_FILTER_OFF;
    
    // Set Transmitter Mode
    C2TX0CON = CXTXXCON_VALUE_LOW_PRIORITY;
    C2TX1CON = CXTXXCON_VALUE_MEDIUM_PRIORITY;
    C2TX2CON = CXTXXCON_VALUE_HIGH_PRIORITY;
    
    C2TX0DLC = CXTXXDLC_VALUE;
    C2TX1DLC = CXTXXDLC_VALUE;
    C2TX2DLC = CXTXXDLC_VALUE;
    
    
    // Set Receiver Mode
    C2RX0CON = CXRXXCON_VALUE;
    C2RX1CON = CXRXXCON_VALUE;
    

    // Switch to normal operation
    C2CTRL = CXCTRL_OPERATE_MODE_VALUE;
    while(C2CTRLbits.OPMODE != 0);
    
    // Enable Can interrupt
    _C2IE = 1;
    _C1IE = 0;
  }

  // AUTO Configuration of the EEPROM if it blank
  can_params.eeprom_intialization_error = 1;
  if (ETMEEPromReadPage(EEPROM_TEST_PAGE, &eeprom_page_data[0]) == 0xFFFF) {
    // Test page read correctly
    can_params.eeprom_intialization_error = 0;
  } else {
    etm_can_slave_debug_data.debugging_TBD_17 = 1;
    // Test Page did not read properly, try and write it
    if(ETMEEPromWritePageWithConfirmation(EEPROM_TEST_PAGE, &eeprom_page_data[0]) == 0xFFFF) {
      can_params.eeprom_intialization_error = 0;
      // The device is now working
      // Update the pages if they need it
      
      if (ETMEEPromReadPage(EEPROM_PAGE_ANALOG_CALIBRATION_0_1_2, &eeprom_page_data[0]) == 0) {
	eeprom_page_data[0] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[1] = ETM_ANALOG_OFFSET_ZERO;
	eeprom_page_data[2] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[3] = ETM_ANALOG_OFFSET_ZERO;
	
	eeprom_page_data[4] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[5] = ETM_ANALOG_OFFSET_ZERO;
	eeprom_page_data[6] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[7] = ETM_ANALOG_OFFSET_ZERO;
	
	eeprom_page_data[8] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[9] = ETM_ANALOG_OFFSET_ZERO;
	eeprom_page_data[10] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[11] = ETM_ANALOG_OFFSET_ZERO;
	
	eeprom_page_data[12] = 0;
	eeprom_page_data[13] = 0;
	eeprom_page_data[14] = 0;
	
	ETMEEPromWritePage(EEPROM_PAGE_ANALOG_CALIBRATION_0_1_2, &eeprom_page_data[0]);
      }
      
      
      if (ETMEEPromReadPage(EEPROM_PAGE_ANALOG_CALIBRATION_3_4_5, &eeprom_page_data[0]) == 0) {
	eeprom_page_data[0] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[1] = ETM_ANALOG_OFFSET_ZERO;
	eeprom_page_data[2] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[3] = ETM_ANALOG_OFFSET_ZERO;
	
	eeprom_page_data[4] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[5] = ETM_ANALOG_OFFSET_ZERO;
	eeprom_page_data[6] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[7] = ETM_ANALOG_OFFSET_ZERO;
	
	eeprom_page_data[8] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[9] = ETM_ANALOG_OFFSET_ZERO;
	eeprom_page_data[10] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[11] = ETM_ANALOG_OFFSET_ZERO;
	
	eeprom_page_data[12] = 0;
	eeprom_page_data[13] = 0;
	eeprom_page_data[14] = 0;
	
	ETMEEPromWritePage(EEPROM_PAGE_ANALOG_CALIBRATION_3_4_5, &eeprom_page_data[0]);
      }
      
      if (ETMEEPromReadPage(EEPROM_PAGE_ANALOG_CALIBRATION_6_7, &eeprom_page_data[0]) == 0) {
	eeprom_page_data[0] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[1] = ETM_ANALOG_OFFSET_ZERO;
	eeprom_page_data[2] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[3] = ETM_ANALOG_OFFSET_ZERO;
	
	eeprom_page_data[4] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[5] = ETM_ANALOG_OFFSET_ZERO;
	eeprom_page_data[6] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[7] = ETM_ANALOG_OFFSET_ZERO;
	
	eeprom_page_data[8] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[9] = ETM_ANALOG_OFFSET_ZERO;
	eeprom_page_data[10] = ETM_ANALOG_CALIBRATION_SCALE_1;
	eeprom_page_data[11] = ETM_ANALOG_OFFSET_ZERO;
	
	eeprom_page_data[12] = 0;
	eeprom_page_data[13] = 0;
	eeprom_page_data[14] = 0;
	
	ETMEEPromWritePage(EEPROM_PAGE_ANALOG_CALIBRATION_6_7, &eeprom_page_data[0]);
      }
      
      
      if (ETMEEPromReadPage(EEPROM_PAGE_BOARD_CONFIGURATION, &eeprom_page_data[0]) == 0) {
	eeprom_page_data[0] = 0x4E41;
	eeprom_page_data[1] = 33333;
	eeprom_page_data[2] = 0;
	eeprom_page_data[3] = 0;
	
	eeprom_page_data[4] = 0;
	eeprom_page_data[5] = 0;
	eeprom_page_data[6] = 0;
	eeprom_page_data[7] = 0;
	
	eeprom_page_data[8] = 0;
	eeprom_page_data[9] = 0;
	eeprom_page_data[10] = 0;
	eeprom_page_data[11] = 0;
	
	eeprom_page_data[12] = 0;
	eeprom_page_data[13] = 0;
	eeprom_page_data[14] = 0;
	
	ETMEEPromWritePage(EEPROM_PAGE_BOARD_CONFIGURATION, &eeprom_page_data[0]);
      }

    }
  }


  // Load the Analog Calibration
  if (ETMEEPromReadPage(EEPROM_PAGE_ANALOG_CALIBRATION_0_1_2, &eeprom_page_data[0]) == 0xFFFF) {
    // No error with the EEPROM read
    etm_can_slave_debug_data.calibration_0_internal_gain   = eeprom_page_data[0];
    etm_can_slave_debug_data.calibration_0_internal_offset = eeprom_page_data[1];
    etm_can_slave_debug_data.calibration_0_external_gain   = eeprom_page_data[2];
    etm_can_slave_debug_data.calibration_0_external_offset = eeprom_page_data[3];

    etm_can_slave_debug_data.calibration_1_internal_gain   = eeprom_page_data[4];
    etm_can_slave_debug_data.calibration_1_internal_offset = eeprom_page_data[5];
    etm_can_slave_debug_data.calibration_1_external_gain   = eeprom_page_data[6];
    etm_can_slave_debug_data.calibration_1_external_offset = eeprom_page_data[7];

    etm_can_slave_debug_data.calibration_2_internal_gain   = eeprom_page_data[8];
    etm_can_slave_debug_data.calibration_2_internal_offset = eeprom_page_data[9];
    etm_can_slave_debug_data.calibration_2_external_gain   = eeprom_page_data[10];
    etm_can_slave_debug_data.calibration_2_external_offset = eeprom_page_data[11];
    
  } else {
    // Data error, set default values and error flag
    can_params.eeprom_intialization_error = 1;
    etm_can_slave_debug_data.debugging_TBD_16++;
    etm_can_slave_debug_data.calibration_0_internal_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_0_internal_offset = ETM_ANALOG_OFFSET_ZERO;
    etm_can_slave_debug_data.calibration_0_external_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_0_external_offset = ETM_ANALOG_OFFSET_ZERO;

    etm_can_slave_debug_data.calibration_1_internal_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_1_internal_offset = ETM_ANALOG_OFFSET_ZERO;
    etm_can_slave_debug_data.calibration_1_external_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_1_external_offset = ETM_ANALOG_OFFSET_ZERO;

    etm_can_slave_debug_data.calibration_2_internal_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_2_internal_offset = ETM_ANALOG_OFFSET_ZERO;
    etm_can_slave_debug_data.calibration_2_external_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_2_external_offset = ETM_ANALOG_OFFSET_ZERO;
  }
  
  if (ETMEEPromReadPage(EEPROM_PAGE_ANALOG_CALIBRATION_3_4_5, &eeprom_page_data[0]) == 0xFFFF) {
    // No error with the EEPROM read
    etm_can_slave_debug_data.calibration_3_internal_gain   = eeprom_page_data[0];
    etm_can_slave_debug_data.calibration_3_internal_offset = eeprom_page_data[1];
    etm_can_slave_debug_data.calibration_3_external_gain   = eeprom_page_data[2];
    etm_can_slave_debug_data.calibration_3_external_offset = eeprom_page_data[3];

    etm_can_slave_debug_data.calibration_4_internal_gain   = eeprom_page_data[4];
    etm_can_slave_debug_data.calibration_4_internal_offset = eeprom_page_data[5];
    etm_can_slave_debug_data.calibration_4_external_gain   = eeprom_page_data[6];
    etm_can_slave_debug_data.calibration_4_external_offset = eeprom_page_data[7];

    etm_can_slave_debug_data.calibration_5_internal_gain   = eeprom_page_data[8];
    etm_can_slave_debug_data.calibration_5_internal_offset = eeprom_page_data[9];
    etm_can_slave_debug_data.calibration_5_external_gain   = eeprom_page_data[10];
    etm_can_slave_debug_data.calibration_5_external_offset = eeprom_page_data[11];
    
  } else {
    // Data error, set default values and error flag
    can_params.eeprom_intialization_error = 1;
    etm_can_slave_debug_data.debugging_TBD_16++;
    
    etm_can_slave_debug_data.calibration_3_internal_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_3_internal_offset = ETM_ANALOG_OFFSET_ZERO;
    etm_can_slave_debug_data.calibration_3_external_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_3_external_offset = ETM_ANALOG_OFFSET_ZERO;

    etm_can_slave_debug_data.calibration_4_internal_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_4_internal_offset = ETM_ANALOG_OFFSET_ZERO;
    etm_can_slave_debug_data.calibration_4_external_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_4_external_offset = ETM_ANALOG_OFFSET_ZERO;

    etm_can_slave_debug_data.calibration_5_internal_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_5_internal_offset = ETM_ANALOG_OFFSET_ZERO;
    etm_can_slave_debug_data.calibration_5_external_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_5_external_offset = ETM_ANALOG_OFFSET_ZERO;
  }

  if (ETMEEPromReadPage(EEPROM_PAGE_ANALOG_CALIBRATION_6_7, &eeprom_page_data[0]) == 0xFFFF) {
    // No error with the EEPROM read
    etm_can_slave_debug_data.calibration_6_internal_gain   = eeprom_page_data[0];
    etm_can_slave_debug_data.calibration_6_internal_offset = eeprom_page_data[1];
    etm_can_slave_debug_data.calibration_6_external_gain   = eeprom_page_data[2];
    etm_can_slave_debug_data.calibration_6_external_offset = eeprom_page_data[3];

    etm_can_slave_debug_data.calibration_7_internal_gain   = eeprom_page_data[4];
    etm_can_slave_debug_data.calibration_7_internal_offset = eeprom_page_data[5];
    etm_can_slave_debug_data.calibration_7_external_gain   = eeprom_page_data[6];
    etm_can_slave_debug_data.calibration_7_external_offset = eeprom_page_data[7];
    
  } else {
    // Data error, set default values and error flag
    can_params.eeprom_intialization_error = 1;
    etm_can_slave_debug_data.debugging_TBD_16++;
    
    etm_can_slave_debug_data.calibration_6_internal_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_6_internal_offset = ETM_ANALOG_OFFSET_ZERO;
    etm_can_slave_debug_data.calibration_6_external_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_6_external_offset = ETM_ANALOG_OFFSET_ZERO;

    etm_can_slave_debug_data.calibration_7_internal_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_7_internal_offset = ETM_ANALOG_OFFSET_ZERO;
    etm_can_slave_debug_data.calibration_7_external_gain   = ETM_ANALOG_CALIBRATION_SCALE_1;
    etm_can_slave_debug_data.calibration_7_external_offset = ETM_ANALOG_OFFSET_ZERO;
  }

  // Load the Board Configuration Data
  if (ETMEEPromReadPage(EEPROM_PAGE_BOARD_CONFIGURATION, &eeprom_page_data[0]) == 0xFFFF) {
    // No error with the EEPROM read
    slave_board_data.device_rev_2x_ASCII           = eeprom_page_data[0];
    slave_board_data.device_serial_number          = eeprom_page_data[1];
    
  } else {
    can_params.eeprom_intialization_error = 1;
    etm_can_slave_debug_data.debugging_TBD_16++;
	
    slave_board_data.device_rev_2x_ASCII           = 0x3333;
    slave_board_data.device_serial_number          = 44444;
  }

  // Debugging Pins Used
  _TRISA7 = 0;
  _TRISF8 = 0;
}

void ETMCanSlaveLoadConfiguration(unsigned long agile_id, unsigned int agile_dash,
				  unsigned int firmware_agile_rev, unsigned int firmware_branch, 
				  unsigned int firmware_branch_rev) {
  
  slave_board_data.device_id_low_word = (agile_id & 0xFFFF);
  agile_id >>= 16;
  slave_board_data.device_id_high_word = agile_id;
  slave_board_data.device_id_dash_number = agile_dash;
  
  slave_board_data.device_firmware_rev_agile = firmware_agile_rev;
  slave_board_data.device_firmware_branch = firmware_branch;
  slave_board_data.device_firmware_branch_rev = firmware_branch_rev;

}


void ETMCanSlaveDoCan(void) {
  ETMCanSlaveProcessMessage();
  ETMCanSlaveTimedTransmit();
  ETMCanSlaveSendScopeData();
  ETMCanSlaveCheckForTimeOut();
  ETMCanSlaveSendUpdateIfNewNotReady();

  // Log debugging information
  etm_can_slave_debug_data.RCON_value = RCON;
  
  // Record the max TX counter
  if ((*CXEC_ptr & 0xFF00) > (etm_can_slave_debug_data.CXEC_reg_max & 0xFF00)) {
    etm_can_slave_debug_data.CXEC_reg_max &= 0x00FF;
    etm_can_slave_debug_data.CXEC_reg_max += (*CXEC_ptr & 0xFF00);
  }
  
  // Record the max RX counter
  if ((*CXEC_ptr & 0x00FF) > (etm_can_slave_debug_data.CXEC_reg_max & 0x00FF)) {
    etm_can_slave_debug_data.CXEC_reg_max &= 0xFF00;
    etm_can_slave_debug_data.CXEC_reg_max += (*CXEC_ptr & 0x00FF);
  }
}


unsigned char ETMCanSlaveGetDiscreteCMD(void) {
  if (BufferByte64IsNotEmpty(&discrete_cmd_buffer)) {
    return BufferByte64ReadByte(&discrete_cmd_buffer);
  }
  return 0;
}


void ETMCanSlaveProcessMessage(void) {
  ETMCanMessage next_message;
  while (ETMCanBufferNotEmpty(&etm_can_slave_rx_message_buffer)) {
    ETMCanReadMessageFromBuffer(&etm_can_slave_rx_message_buffer, &next_message);
    ETMCanSlaveExecuteCMD(&next_message);      
  }
  
  etm_can_slave_debug_data.can_tx_buf_overflow = etm_can_slave_tx_message_buffer.message_overwrite_count;
  etm_can_slave_debug_data.can_rx_buf_overflow = etm_can_slave_rx_message_buffer.message_overwrite_count;
}


void ETMCanSlaveExecuteCMD(ETMCanMessage* message_ptr) {
  unsigned int cmd_id;
  unsigned int eeprom_page_data[16];
  unsigned int eeprom_write_error;
  unsigned char discrete_cmd_id;
  unsigned int page_to_read;
  unsigned int location;
  
  cmd_id = message_ptr->identifier;
  cmd_id >>= 2;
  cmd_id &= 0x003F;

  switch(cmd_id) {

  case ETM_CAN_CMD_ID_RESET_MCU:
    if (message_ptr->word3 == can_params.address) {
      __delay32(10000000); // Delay for a while so that any EEPROM write process can complete
      __asm__ ("Reset");
    }
    break;

  case ETM_CAN_CMD_ID_LOAD_DEFAULT_CALIBRATION:
    if (message_ptr->word3 == can_params.address) {
      ETMCanSlaveLoadDefaultCalibration();
    }
    break;

  case ETM_CAN_CMD_ID_LOAD_REV_AND_SERIAL_NUMBER:
    if (message_ptr->word3 == can_params.address) {
      eeprom_write_error = 0xFFFF;
      eeprom_write_error &= ETMEEPromReadPage(EEPROM_PAGE_BOARD_CONFIGURATION, &eeprom_page_data[0]);
      if (eeprom_write_error == 0xFFFF) {
	eeprom_page_data[0] = message_ptr->word1; // update the device_rev_2x_ASCII
	eeprom_page_data[1] = message_ptr->word0; // update the device_serial_number
	eeprom_write_error &= ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_BOARD_CONFIGURATION, &eeprom_page_data[0]);
      }
      if (eeprom_write_error != 0xFFFF) {
	can_params.eeprom_calibration_write_error = 1;
      } else {
	slave_board_data.device_rev_2x_ASCII           = eeprom_page_data[0];
	slave_board_data.device_serial_number          = eeprom_page_data[1];
      }
    }
    break;

  case ETM_CAN_CMD_ID_SET_CAL_PAIR:
    if (message_ptr->word3 == can_params.address) {
      if ((message_ptr->word2 & 0x000F) < 3) {
	// Working on Pair 0,1,2
	if (ETMEEPromReadPage(EEPROM_PAGE_ANALOG_CALIBRATION_0_1_2, &eeprom_page_data[0])) {
	  location = (message_ptr->word2 & 0x000F);
	  location *= 4;
	  if (message_ptr->word2 & 0x0010) {
	    location += 2;
	  }
	  if (location <= 10) {
	    eeprom_page_data[location] = message_ptr->word1;
	    eeprom_page_data[location +1] = message_ptr->word0;
	  }
	  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ANALOG_CALIBRATION_0_1_2, &eeprom_page_data[0])) {
	    etm_can_slave_debug_data.calibration_0_internal_gain   = eeprom_page_data[0];
	    etm_can_slave_debug_data.calibration_0_internal_offset = eeprom_page_data[1];
	    etm_can_slave_debug_data.calibration_0_external_gain   = eeprom_page_data[2];
	    etm_can_slave_debug_data.calibration_0_external_offset = eeprom_page_data[3];
	    
	    etm_can_slave_debug_data.calibration_1_internal_gain   = eeprom_page_data[4];
	    etm_can_slave_debug_data.calibration_1_internal_offset = eeprom_page_data[5];
	    etm_can_slave_debug_data.calibration_1_external_gain   = eeprom_page_data[6];
	    etm_can_slave_debug_data.calibration_1_external_offset = eeprom_page_data[7];
	    
	    etm_can_slave_debug_data.calibration_2_internal_gain   = eeprom_page_data[8];
	    etm_can_slave_debug_data.calibration_2_internal_offset = eeprom_page_data[9];
	    etm_can_slave_debug_data.calibration_2_external_gain   = eeprom_page_data[10];
	    etm_can_slave_debug_data.calibration_2_external_offset = eeprom_page_data[11];
	  }
	}
      } else if((message_ptr->word2 & 0x000F) < 6) {
	// Working on Pair 3,4,5
	if (ETMEEPromReadPage(EEPROM_PAGE_ANALOG_CALIBRATION_3_4_5, &eeprom_page_data[0])) {
	  location = (message_ptr->word2 & 0x000F) - 3;
	  location *= 4;
	  if (message_ptr->word2 & 0x0010) {
	    location += 2;
	  }
	  if (location <= 10) {
	    eeprom_page_data[location] = message_ptr->word1;
	    eeprom_page_data[location +1] = message_ptr->word0;
	  }
	  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ANALOG_CALIBRATION_3_4_5, &eeprom_page_data[0])) {
	    etm_can_slave_debug_data.calibration_3_internal_gain   = eeprom_page_data[0];
	    etm_can_slave_debug_data.calibration_3_internal_offset = eeprom_page_data[1];
	    etm_can_slave_debug_data.calibration_3_external_gain   = eeprom_page_data[2];
	    etm_can_slave_debug_data.calibration_3_external_offset = eeprom_page_data[3];
	    
	    etm_can_slave_debug_data.calibration_4_internal_gain   = eeprom_page_data[4];
	    etm_can_slave_debug_data.calibration_4_internal_offset = eeprom_page_data[5];
	    etm_can_slave_debug_data.calibration_4_external_gain   = eeprom_page_data[6];
	    etm_can_slave_debug_data.calibration_4_external_offset = eeprom_page_data[7];
	    
	    etm_can_slave_debug_data.calibration_5_internal_gain   = eeprom_page_data[8];
	    etm_can_slave_debug_data.calibration_5_internal_offset = eeprom_page_data[9];
	    etm_can_slave_debug_data.calibration_5_external_gain   = eeprom_page_data[10];
	    etm_can_slave_debug_data.calibration_5_external_offset = eeprom_page_data[11]; 
	  }
	}
      } else if((message_ptr->word2 & 0x000F) < 8) {
	// Working on Pair 6,7
	if (ETMEEPromReadPage(EEPROM_PAGE_ANALOG_CALIBRATION_6_7, &eeprom_page_data[0])) {
	  location = (message_ptr->word2 & 0x000F) - 6;
	  location *= 4;
	  if (message_ptr->word2 & 0x0010) {
	    location += 2;
	  }
	  if (location <= 6) {
	    eeprom_page_data[location] = message_ptr->word1;
	    eeprom_page_data[location +1] = message_ptr->word0;
	  }
	  if (ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ANALOG_CALIBRATION_6_7, &eeprom_page_data[0])) {
	    etm_can_slave_debug_data.calibration_6_internal_gain   = eeprom_page_data[0];
	    etm_can_slave_debug_data.calibration_6_internal_offset = eeprom_page_data[1];
	    etm_can_slave_debug_data.calibration_6_external_gain   = eeprom_page_data[2];
	    etm_can_slave_debug_data.calibration_6_external_offset = eeprom_page_data[3];
	    
	    etm_can_slave_debug_data.calibration_7_internal_gain   = eeprom_page_data[4];
	    etm_can_slave_debug_data.calibration_7_internal_offset = eeprom_page_data[5];
	    etm_can_slave_debug_data.calibration_7_external_gain   = eeprom_page_data[6];
	    etm_can_slave_debug_data.calibration_7_external_offset = eeprom_page_data[7];
	  }
	}
      }
    }
    break;
    
    
  case ETM_CAN_CMD_ID_SET_RAM_DEBUG:
    if (message_ptr->word3 == can_params.address) {
      message_ptr->word2 <<= 1;
      message_ptr->word1 <<= 1;
      message_ptr->word0 <<= 1;
      if (message_ptr->word2 < RAM_SIZE_WORDS) {
	ram_ptr_c = (unsigned int*)message_ptr->word2;
      }
      if (message_ptr->word1 < RAM_SIZE_WORDS) {
	ram_ptr_b = (unsigned int*)message_ptr->word1;
      }
      if (message_ptr->word0 < RAM_SIZE_WORDS) {
	ram_ptr_a = (unsigned int*)message_ptr->word0;
      }
      scope_data_sources[SCOPE_DATA_SELECT_SIZE - 2] = ram_ptr_a;
      scope_data_sources[SCOPE_DATA_SELECT_SIZE - 1] = ram_ptr_b;
    }
    break;

  case ETM_CAN_CMD_ID_SET_EEPROM_DEBUG:
    if (message_ptr->word3 == can_params.address) {
      page_to_read = message_ptr->word2>>4;
      message_ptr->word2 &= 0x000F;
      if (ETMEEPromPrivateReadSinglePage(page_to_read, eeprom_page_data) == 0xFFFF) {
	etm_can_slave_debug_data.eeprom_read_result = eeprom_page_data[message_ptr->word2];
      } else {
	etm_can_slave_debug_data.eeprom_read_result = 0xFFEF;
      }
    }
    break;

  case ETM_CAN_CMD_ID_SET_IGNORE_FAULTS:
    // DPARKER IMPLIMENT THIS
    break;

  case ETM_CAN_CMD_ID_CLEAR_DEBUG:
    ETMCanSlaveClearDebug();
    break;
    
  case ETM_CAN_CMD_ID_HVPS_SET_POINTS:
    slave_data.hvps_set_point_dose_level_3 = message_ptr->word3;
    slave_data.hvps_set_point_dose_level_2 = message_ptr->word2;
    slave_data.hvps_set_point_dose_level_1 = message_ptr->word1;
    slave_data.hvps_set_point_dose_level_0 = message_ptr->word0;
    setting_data_recieved |= 0b00000001;
    break;

  case ETM_CAN_CMD_ID_MAGNET_SET_POINTS:
    slave_data.electromagnet_set_point_dose_level_3 = message_ptr->word3;
    slave_data.electromagnet_set_point_dose_level_2 = message_ptr->word2;
    slave_data.electromagnet_set_point_dose_level_1 = message_ptr->word1;
    slave_data.electromagnet_set_point_dose_level_0 = message_ptr->word0;
    setting_data_recieved |= 0b00000010;
    break;

  case ETM_CAN_CMD_ID_AFC_HOME_POSTION:
    slave_data.afc_home_position_dose_level_3 = message_ptr->word3;
    slave_data.afc_home_position_dose_level_2 = message_ptr->word2;
    slave_data.afc_home_position_dose_level_1 = message_ptr->word1;
    slave_data.afc_home_position_dose_level_0 = message_ptr->word0;
    setting_data_recieved |= 0b00000100;
    break;

  case ETM_CAN_CMD_ID_GUN_PULSE_TOP_SET_POINTS:
    slave_data.gun_driver_pulse_top_voltage_dose_level_3 = message_ptr->word3;
    slave_data.gun_driver_pulse_top_voltage_dose_level_2 = message_ptr->word2;
    slave_data.gun_driver_pulse_top_voltage_dose_level_1 = message_ptr->word1;
    slave_data.gun_driver_pulse_top_voltage_dose_level_0 = message_ptr->word0;
    setting_data_recieved |= 0b00001000;
    break;

  case ETM_CAN_CMD_ID_GUN_CATHODE_SET_POINTS:
    slave_data.gun_driver_cathode_voltage_dose_level_3 = message_ptr->word3;
    slave_data.gun_driver_cathode_voltage_dose_level_2 = message_ptr->word2;
    slave_data.gun_driver_cathode_voltage_dose_level_1 = message_ptr->word1;
    slave_data.gun_driver_cathode_voltage_dose_level_0 = message_ptr->word0;
    setting_data_recieved |= 0b00010000;
    break;

  case ETM_CAN_CMD_ID_ALL_DOSE_SET_POINTS_REGISTER_A:
    slave_data.magnetron_heater_scaled_heater_current = message_ptr->word3;
    slave_data.afc_aux_control_or_offset              = message_ptr->word2;
    slave_data.gun_driver_grid_bias_voltage           = message_ptr->word1;
    slave_data.gun_driver_heater_voltage              = message_ptr->word0;
    setting_data_recieved |= 0b00100000;
    break;

  case ETM_CAN_CMD_ID_ALL_DOSE_SET_POINTS_REGISTER_B:
    slave_data.afc_manual_target_position             = message_ptr->word0;
    setting_data_recieved |= 0b01000000;
    break;

  case ETM_CAN_CMD_ID_DISCRETE_CMD:
    discrete_cmd_id = message_ptr->word3;
    BufferByte64WriteByte(&discrete_cmd_buffer, discrete_cmd_id);
    break;

  case ETM_CAN_CMD_ID_SCOPE_SETTINGS:
    // Scope A settings
    slave_data.scope_a_settings = SCOPE_CHANNEL_DISABLED;
    slave_data.scope_b_settings = SCOPE_CHANNEL_DISABLED;
    slave_data.scope_hv_vmon_settings = SCOPE_CHANNEL_DISABLED;
       
    if ((message_ptr->word3 >> 8) == can_params.address) {
      // This board is transmitting scope_a
      if ((message_ptr->word3 & 0x00FF) < SCOPE_DATA_SELECT_SIZE) {
	ptr_scope_a_data_source = scope_data_sources[message_ptr->word3 & 0x00FF];
	slave_data.scope_a_settings = SCOPE_CHANNEL_ACTIVE;
      }
    }

    // Scope B settings
    if ((message_ptr->word2 >> 8) == can_params.address) {
      // This board is transmitting scope_b
      if ((message_ptr->word2 & 0x00FF) <= SCOPE_DATA_SELECT_SIZE) {
	ptr_scope_b_data_source = scope_data_sources[message_ptr->word2 & 0x00FF];
	slave_data.scope_b_settings = SCOPE_CHANNEL_ACTIVE;
      }
    }

    if ((message_ptr->word1 >> 8) == can_params.address) {
      slave_data.scope_hv_vmon_settings = SCOPE_CHANNEL_ACTIVE;
    }

    break;

    
  default:
    // DPARKER increment fault counter
    etm_can_slave_debug_data.can_invalid_index++;
    break;
			  
  }

  if (setting_data_recieved == 0b01111111) {
    _CONTROL_NOT_CONFIGURED = 0;
  }
}


void ETMCanSlaveLoadDefaultCalibration(void) {
  unsigned int eeprom_page_data[16];
  unsigned int write_error;
  eeprom_page_data[0] = ETM_ANALOG_CALIBRATION_SCALE_1;
  eeprom_page_data[1] = ETM_ANALOG_OFFSET_ZERO;
  eeprom_page_data[2] = ETM_ANALOG_CALIBRATION_SCALE_1;
  eeprom_page_data[3] = ETM_ANALOG_OFFSET_ZERO;

  eeprom_page_data[4] = ETM_ANALOG_CALIBRATION_SCALE_1;
  eeprom_page_data[5] = ETM_ANALOG_OFFSET_ZERO;
  eeprom_page_data[6] = ETM_ANALOG_CALIBRATION_SCALE_1;
  eeprom_page_data[7] = ETM_ANALOG_OFFSET_ZERO;
  
  eeprom_page_data[8] = ETM_ANALOG_CALIBRATION_SCALE_1;
  eeprom_page_data[9] = ETM_ANALOG_OFFSET_ZERO;
  eeprom_page_data[10] = ETM_ANALOG_CALIBRATION_SCALE_1;
  eeprom_page_data[11] = ETM_ANALOG_OFFSET_ZERO;

  eeprom_page_data[12] = 0;
  eeprom_page_data[13] = 0;
  eeprom_page_data[14] = 0;

  write_error = 0xFFFF;
  write_error &= ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ANALOG_CALIBRATION_0_1_2 ,&eeprom_page_data[0]);
  if (write_error != 0xFFFF) {
    can_params.eeprom_calibration_write_error = 1;
  } else {
    etm_can_slave_debug_data.calibration_0_internal_gain   = eeprom_page_data[0];
    etm_can_slave_debug_data.calibration_0_internal_offset = eeprom_page_data[1];
    etm_can_slave_debug_data.calibration_0_external_gain   = eeprom_page_data[2];
    etm_can_slave_debug_data.calibration_0_external_offset = eeprom_page_data[3];

    etm_can_slave_debug_data.calibration_1_internal_gain   = eeprom_page_data[4];
    etm_can_slave_debug_data.calibration_1_internal_offset = eeprom_page_data[5];
    etm_can_slave_debug_data.calibration_1_external_gain   = eeprom_page_data[6];
    etm_can_slave_debug_data.calibration_1_external_offset = eeprom_page_data[7];

    etm_can_slave_debug_data.calibration_2_internal_gain   = eeprom_page_data[8];
    etm_can_slave_debug_data.calibration_2_internal_offset = eeprom_page_data[9];
    etm_can_slave_debug_data.calibration_2_external_gain   = eeprom_page_data[10];
    etm_can_slave_debug_data.calibration_2_external_offset = eeprom_page_data[11];
  }

  write_error = 0xFFFF;
  write_error &= ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ANALOG_CALIBRATION_3_4_5 ,&eeprom_page_data[0]);
  if (write_error != 0xFFFF) {
    can_params.eeprom_calibration_write_error = 1;
  } else {
    etm_can_slave_debug_data.calibration_3_internal_gain   = eeprom_page_data[0];
    etm_can_slave_debug_data.calibration_3_internal_offset = eeprom_page_data[1];
    etm_can_slave_debug_data.calibration_3_external_gain   = eeprom_page_data[2];
    etm_can_slave_debug_data.calibration_3_external_offset = eeprom_page_data[3];

    etm_can_slave_debug_data.calibration_4_internal_gain   = eeprom_page_data[4];
    etm_can_slave_debug_data.calibration_4_internal_offset = eeprom_page_data[5];
    etm_can_slave_debug_data.calibration_4_external_gain   = eeprom_page_data[6];
    etm_can_slave_debug_data.calibration_4_external_offset = eeprom_page_data[7];

    etm_can_slave_debug_data.calibration_5_internal_gain   = eeprom_page_data[8];
    etm_can_slave_debug_data.calibration_5_internal_offset = eeprom_page_data[9];
    etm_can_slave_debug_data.calibration_5_external_gain   = eeprom_page_data[10];
    etm_can_slave_debug_data.calibration_5_external_offset = eeprom_page_data[11]; 
  }

  write_error = 0xFFFF;
  write_error &= ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_ANALOG_CALIBRATION_6_7 ,&eeprom_page_data[0]);
  if (write_error != 0xFFFF) {
    can_params.eeprom_calibration_write_error = 1;
  } else {
    etm_can_slave_debug_data.calibration_6_internal_gain   = eeprom_page_data[0];
    etm_can_slave_debug_data.calibration_6_internal_offset = eeprom_page_data[1];
    etm_can_slave_debug_data.calibration_6_external_gain   = eeprom_page_data[2];
    etm_can_slave_debug_data.calibration_6_external_offset = eeprom_page_data[3];

    etm_can_slave_debug_data.calibration_7_internal_gain   = eeprom_page_data[4];
    etm_can_slave_debug_data.calibration_7_internal_offset = eeprom_page_data[5];
    etm_can_slave_debug_data.calibration_7_external_gain   = eeprom_page_data[6];
    etm_can_slave_debug_data.calibration_7_external_offset = eeprom_page_data[7];
  }


  
  eeprom_page_data[0] = 0x2123;
  eeprom_page_data[1] = 22222;
  eeprom_page_data[2] = 0;
  eeprom_page_data[3] = 0;
  
  eeprom_page_data[4] = 0;
  eeprom_page_data[5] = 0;
  eeprom_page_data[6] = 0;
  eeprom_page_data[7] = 0;
  
  eeprom_page_data[8] = 0;
  eeprom_page_data[9] = 0;
  eeprom_page_data[10] = 0;
  eeprom_page_data[11] = 0;
  
  eeprom_page_data[12] = 0;
  eeprom_page_data[13] = 0;
  eeprom_page_data[14] = 0;

  write_error = 0xFFFF;
  write_error &= ETMEEPromWritePageWithConfirmation(EEPROM_PAGE_BOARD_CONFIGURATION ,&eeprom_page_data[0]);
  if (write_error != 0xFFFF) {
    can_params.eeprom_calibration_write_error = 1;
  } else {
    slave_board_data.device_rev_2x_ASCII           = eeprom_page_data[0];
    slave_board_data.device_serial_number          = eeprom_page_data[1];
  }
}


void ETMCanSlaveTimedTransmit(void) {
  static unsigned int slave_data_log_index;      
  static unsigned int slave_data_log_sub_index;
  static unsigned long etm_can_slave_timed_transmit_holding_var; // Used to time the transmit of CAN Messaged back to the ECB
  
  // Sends the debug information up as log data  
  if (ETMTickRunOnceEveryNMilliseconds(SLAVE_TRANSMIT_MILLISECONDS, &etm_can_slave_timed_transmit_holding_var)) {
    // Will be true once every 100ms

    ETMCanSlaveSendStatus(); // Send out the status every 100mS
    
    // Set the Ready LED
    if (_CONTROL_NOT_READY) {
      ETMClearPin(can_params.not_ready_led); // This turns on the LED
    } else {
      ETMSetPin(can_params.not_ready_led);  // This turns off the LED
    }
        
    slave_data_log_index++;
    if (slave_data_log_index >= 18) {
      slave_data_log_index = 0;
      slave_data_log_sub_index++;
      slave_data_log_sub_index &= 0b11;
      // Flash the flashing LED
      if (ETMReadPinLatch(can_params.flash_led)) {
	ETMClearPin(can_params.flash_led);
      } else {
	ETMSetPin(can_params.flash_led);
      }
    }
    
    // Also send out one logging data message every 100mS
    switch (slave_data_log_index) 
      {
	
      case 0:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_0,
			   slave_board_data.log_data[3],
			   slave_board_data.log_data[2],
			   slave_board_data.log_data[1],
			   slave_board_data.log_data[0]);
	break;
	
      case 1:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_1,
			   slave_board_data.log_data[7],
			   slave_board_data.log_data[6],
			   slave_board_data.log_data[5],
			   slave_board_data.log_data[4]);
	break;
	
      case 2:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_2,
			   slave_board_data.log_data[11],
			   slave_board_data.log_data[10],
			   slave_board_data.log_data[9],
			   slave_board_data.log_data[8]);
	break;
	
      case 3:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_3,
			   slave_board_data.log_data[15],
			   slave_board_data.log_data[14],
			   slave_board_data.log_data[13],
			   slave_board_data.log_data[12]);
	break;
	
      case 4:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_4,
			   slave_board_data.log_data[19],
			   slave_board_data.log_data[18],
			   slave_board_data.log_data[17],
			   slave_board_data.log_data[16]);
	break;
	
      case 5:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_BOARD_SPECIFIC_5,
			   slave_board_data.log_data[23],
			   slave_board_data.log_data[22],
			   slave_board_data.log_data[21],
			   slave_board_data.log_data[20]);
	break;

      case 6:
	if (slave_data_log_sub_index == 0) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CONFIG_0,
			     slave_board_data.device_id_high_word,
			     slave_board_data.device_id_low_word,
			     slave_board_data.device_id_dash_number,
			     slave_board_data.device_rev_2x_ASCII);
	  
	  
	} else if (slave_data_log_sub_index == 1) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CONFIG_1,
			     slave_board_data.device_serial_number,
			     slave_board_data.device_firmware_rev_agile,
			     slave_board_data.device_firmware_branch,
			     slave_board_data.device_firmware_branch_rev);
	  
	} else if (slave_data_log_sub_index == 2) {
    	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_STANDARD_DEBUG_TBD_1, 
			     etm_can_slave_debug_data.debugging_TBD_7, 
			     etm_can_slave_debug_data.debugging_TBD_6,
			     etm_can_slave_debug_data.debugging_TBD_5,
			     etm_can_slave_debug_data.debugging_TBD_4);      
	} else {
    	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_STANDARD_DEBUG_TBD_0, 
			     etm_can_slave_debug_data.debugging_TBD_3, 
			     etm_can_slave_debug_data.debugging_TBD_2,
			     etm_can_slave_debug_data.debugging_TBD_1,
			     etm_can_slave_debug_data.debugging_TBD_0);      
	}
	
      case 7:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_0,
			   etm_can_slave_debug_data.debug_reg[0],
			   etm_can_slave_debug_data.debug_reg[1],
			   etm_can_slave_debug_data.debug_reg[2],
			   etm_can_slave_debug_data.debug_reg[3]);
	break;
	
      case 8:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_1,
			   etm_can_slave_debug_data.debug_reg[4],
			   etm_can_slave_debug_data.debug_reg[5],
			   etm_can_slave_debug_data.debug_reg[6],
			   etm_can_slave_debug_data.debug_reg[7]);
	break;
	
      case 9:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_2,
			   etm_can_slave_debug_data.debug_reg[8],
			   etm_can_slave_debug_data.debug_reg[9],
			   etm_can_slave_debug_data.debug_reg[10],
			   etm_can_slave_debug_data.debug_reg[11]);
	break;
	
      case 10:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_3,
			   etm_can_slave_debug_data.debug_reg[12],
			   etm_can_slave_debug_data.debug_reg[13],
			   etm_can_slave_debug_data.debug_reg[14],
			   etm_can_slave_debug_data.debug_reg[15]);
	break;

      case 11:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_RAM_WATCH_DATA,
			   etm_can_slave_debug_data.ram_monitor_a,
			   etm_can_slave_debug_data.ram_monitor_b,
			   etm_can_slave_debug_data.ram_monitor_c,
			   etm_can_slave_debug_data.eeprom_read_result);
	break;
	
      case 12:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_STANDARD_ANALOG_DATA,
			   etm_can_slave_debug_data.analog_1_nominal_5V,
			   etm_can_slave_debug_data.analog_2_nominal_pos_15,
			   etm_can_slave_debug_data.analog_3_nominal_neg_15,
			   etm_can_slave_debug_data.analog_4_nominal_24);
	break;
	
      case 13:
	if (slave_data_log_sub_index == 0) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CAN_DEBUG_0,
			     etm_can_slave_debug_data.can_tx_0,
			     etm_can_slave_debug_data.can_tx_1,
			     etm_can_slave_debug_data.can_tx_2,
			     etm_can_slave_debug_data.CXEC_reg_max);
	  
	} else if (slave_data_log_sub_index == 1) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CAN_DEBUG_1,
			     etm_can_slave_debug_data.can_rx_0_filt_0,
			     etm_can_slave_debug_data.can_rx_0_filt_1,
			     etm_can_slave_debug_data.can_rx_1_filt_2,
			     etm_can_slave_debug_data.CXINTF_max);
	  
	} else if (slave_data_log_sub_index == 2) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CAN_DEBUG_2,
			     etm_can_slave_debug_data.can_unknown_msg_id,
			     etm_can_slave_debug_data.can_invalid_index,
			     etm_can_slave_debug_data.can_address_error,
			     etm_can_slave_debug_data.can_error_flag);
	  
	} else {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CAN_DEBUG_3,
			     etm_can_slave_debug_data.can_tx_buf_overflow,
			     etm_can_slave_debug_data.can_rx_buf_overflow,
			     etm_can_slave_debug_data.can_rx_log_buf_overflow,
			     etm_can_slave_debug_data.can_timeout);
	}
	
	break;
	

      case 14:
	if (slave_data_log_sub_index == 0) {
	  etm_can_slave_debug_data.eeprom_internal_read_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_INTERNAL_COUNT);
	  etm_can_slave_debug_data.eeprom_internal_read_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_INTERNAL_ERROR);
	  etm_can_slave_debug_data.eeprom_internal_write_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_INTERNAL_COUNT);
	  etm_can_slave_debug_data.eeprom_internal_write_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_INTERNAL_ERROR);

	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_EEPROM_INTERNAL_DEBUG,
			     etm_can_slave_debug_data.eeprom_internal_read_count, 
			     etm_can_slave_debug_data.eeprom_internal_read_error,
			     etm_can_slave_debug_data.eeprom_internal_write_count,
			     etm_can_slave_debug_data.eeprom_internal_write_error);      
	  
	} else if (slave_data_log_sub_index == 1) {
	  etm_can_slave_debug_data.eeprom_i2c_read_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_I2C_COUNT);
	  etm_can_slave_debug_data.eeprom_i2c_read_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_I2C_ERROR);
	  etm_can_slave_debug_data.eeprom_i2c_write_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_I2C_COUNT);
	  etm_can_slave_debug_data.eeprom_i2c_write_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_I2C_ERROR);

	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_EEPROM_I2C_DEBUG,
			     etm_can_slave_debug_data.eeprom_i2c_read_count, 
			     etm_can_slave_debug_data.eeprom_i2c_read_error,
			     etm_can_slave_debug_data.eeprom_i2c_write_count,
			     etm_can_slave_debug_data.eeprom_i2c_write_error);      

	  
	} else if (slave_data_log_sub_index == 2) {
	  etm_can_slave_debug_data.eeprom_spi_read_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_SPI_COUNT);
	  etm_can_slave_debug_data.eeprom_spi_read_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_READ_SPI_ERROR);
	  etm_can_slave_debug_data.eeprom_spi_write_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_SPI_COUNT);
	  etm_can_slave_debug_data.eeprom_spi_write_error = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_WRITE_SPI_ERROR);
	  
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_EEPROM_SPI_DEBUG,
			     etm_can_slave_debug_data.eeprom_spi_read_count, 
			     etm_can_slave_debug_data.eeprom_spi_read_error,
			     etm_can_slave_debug_data.eeprom_spi_write_count,
			     etm_can_slave_debug_data.eeprom_spi_write_error);      

	  
	} else {
	  etm_can_slave_debug_data.eeprom_crc_error_count = ETMEEPromReturnDebugData(ETM_EEPROM_DEBUG_DATA_CRC_ERROR);

	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_EEPROM_CRC_DEBUG, 
			     etm_can_slave_debug_data.eeprom_crc_error_count, 
			     etm_can_slave_debug_data.cmd_data_register_read_invalid_index,
			     etm_can_slave_debug_data.debugging_TBD_17,
			     etm_can_slave_debug_data.debugging_TBD_16);      
	}
	break;

	
      case 15:
	if (slave_data_log_sub_index == 0) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_RESET_VERSION_DEBUG, 
			     etm_can_slave_debug_data.reset_count, 
			     etm_can_slave_debug_data.RCON_value,
			     etm_can_slave_debug_data.can_build_version, 
			     etm_can_slave_debug_data.library_build_version);
	  
	} else if (slave_data_log_sub_index == 1) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_I2C_SPI_SCALE_SELF_TEST_DEBUG, 
			     etm_can_slave_debug_data.i2c_bus_error_count, 
			     etm_can_slave_debug_data.spi_bus_error_count,
			     etm_can_slave_debug_data.scale_error_count,
			     *(unsigned int*)&etm_can_slave_debug_data.self_test_results);      
	  
	} else if (slave_data_log_sub_index == 2) {
    	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_STANDARD_DEBUG_TBD_3, 
			     etm_can_slave_debug_data.debugging_TBD_15, 
			     etm_can_slave_debug_data.debugging_TBD_14,
			     etm_can_slave_debug_data.debugging_TBD_13,
			     etm_can_slave_debug_data.debugging_TBD_12);      
	} else {
    	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_STANDARD_DEBUG_TBD_2, 
			     etm_can_slave_debug_data.debugging_TBD_11, 
			     etm_can_slave_debug_data.debugging_TBD_10,
			     etm_can_slave_debug_data.debugging_TBD_9,
			     etm_can_slave_debug_data.debugging_TBD_8);      
	}
	break;
		
	
      case 16:
	// CALIBRATION DATA
	if (slave_data_log_sub_index == 0) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_0,
			     etm_can_slave_debug_data.calibration_0_internal_gain, 
			     etm_can_slave_debug_data.calibration_0_internal_offset,
			     etm_can_slave_debug_data.calibration_0_external_gain,
			     etm_can_slave_debug_data.calibration_0_external_offset);      
			  
	} else if (slave_data_log_sub_index == 1) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_1,
			     etm_can_slave_debug_data.calibration_1_internal_gain, 
			     etm_can_slave_debug_data.calibration_1_internal_offset,
			     etm_can_slave_debug_data.calibration_1_external_gain,
			     etm_can_slave_debug_data.calibration_1_external_offset);      

	} else if (slave_data_log_sub_index == 2) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_2,
			     etm_can_slave_debug_data.calibration_2_internal_gain, 
			     etm_can_slave_debug_data.calibration_2_internal_offset,
			     etm_can_slave_debug_data.calibration_2_external_gain,
			     etm_can_slave_debug_data.calibration_2_external_offset);      
	} else {
    	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_3,
			     etm_can_slave_debug_data.calibration_3_internal_gain, 
			     etm_can_slave_debug_data.calibration_3_internal_offset,
			     etm_can_slave_debug_data.calibration_3_external_gain,
			     etm_can_slave_debug_data.calibration_3_external_offset);      
	}

	break;


      case 17:
	// CALIBRATION DATA
	if (slave_data_log_sub_index == 0) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_4,
			     etm_can_slave_debug_data.calibration_4_internal_gain, 
			     etm_can_slave_debug_data.calibration_4_internal_offset,
			     etm_can_slave_debug_data.calibration_4_external_gain,
			     etm_can_slave_debug_data.calibration_4_external_offset);      
			  
	} else if (slave_data_log_sub_index == 1) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_5,
			     etm_can_slave_debug_data.calibration_5_internal_gain, 
			     etm_can_slave_debug_data.calibration_5_internal_offset,
			     etm_can_slave_debug_data.calibration_5_external_gain,
			     etm_can_slave_debug_data.calibration_5_external_offset);      

	} else if (slave_data_log_sub_index == 2) {
	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_6,
			     etm_can_slave_debug_data.calibration_6_internal_gain, 
			     etm_can_slave_debug_data.calibration_6_internal_offset,
			     etm_can_slave_debug_data.calibration_6_external_gain,
			     etm_can_slave_debug_data.calibration_6_external_offset);      
	} else {
    	  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_CALIBRATION_7,
			     etm_can_slave_debug_data.calibration_7_internal_gain, 
			     etm_can_slave_debug_data.calibration_7_internal_offset,
			     etm_can_slave_debug_data.calibration_7_external_gain,
			     etm_can_slave_debug_data.calibration_7_external_offset);      
	}

	break;
	
      }
  }
}


static void ETMCanSlaveSendScopeData(void) {
  static unsigned long etm_can_scope_50ms_holding_var;
  static unsigned long etm_can_scope_10ms_holding_var;
  static unsigned int  temp_scope_data_a[4];
  static unsigned int  temp_scope_data_b[4];
  static unsigned int  temp_scope_data_location = 0;
  // Transmit pulse log data once every 50 milliseconds


  // If there is pulse scope data to transmit, send it out
  if (ETMTickRunOnceEveryNMilliseconds(50, &etm_can_scope_50ms_holding_var)) {
    if (pulse_data_transmit_index < 10) {
      ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_PULSE_SCOPE_DATA,
			 pulse_data_to_transmit[pulse_data_transmit_index*4 + 3],
			 pulse_data_to_transmit[pulse_data_transmit_index*4 + 2],
			 pulse_data_to_transmit[pulse_data_transmit_index*4 + 1],
			 pulse_data_to_transmit[pulse_data_transmit_index*4 + 0]);
      pulse_data_transmit_index++;
    }
  }
  
  if (ETMTickRunOnceEveryNMilliseconds(10, &etm_can_scope_10ms_holding_var)) {
    // Send out SCOPE DATA
    
    // Add another entry into scope a,
    if (slave_data.scope_a_settings == SCOPE_CHANNEL_ACTIVE) {
      temp_scope_data_a[temp_scope_data_location] = *ptr_scope_a_data_source;
      if (temp_scope_data_location == 3) {
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_SCOPE_A, temp_scope_data_a[3],
			   temp_scope_data_a[2], temp_scope_data_a[1], temp_scope_data_a[0]);
      }
    }

    // Add another entry into scope b,
    if (slave_data.scope_b_settings == SCOPE_CHANNEL_ACTIVE) {
      temp_scope_data_b[temp_scope_data_location] = *ptr_scope_b_data_source;
      if (temp_scope_data_location == 3) {
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_SCOPE_B, temp_scope_data_b[3],
			   temp_scope_data_b[2], temp_scope_data_b[1], temp_scope_data_b[0]);
      }
    }

    temp_scope_data_location++;
    if (temp_scope_data_location == 4) {
      temp_scope_data_location = 0;
    }
  }
}



void ETMCanSlaveSendStatus(void) {
  ETMCanMessage message;
  message.identifier = ETM_CAN_MSG_STATUS_TX | (can_params.address << 2);
  
  message.word0 = _CONTROL_REGISTER;
  message.word1 = _FAULT_REGISTER;
  message.word2 = _WARNING_REGISTER;
  message.word3 = _NOT_LOGGED_REGISTER;

  ETMCanTXMessage(&message, CXTX1CON_ptr);  
  etm_can_slave_debug_data.can_tx_1++;
  
  _NOTICE_0 = 0;
  _NOTICE_1 = 0;
  _NOTICE_2 = 0;
  _NOTICE_3 = 0;
  _NOTICE_4 = 0;
  _NOTICE_5 = 0;
  _NOTICE_6 = 0;
  _NOTICE_7 = 0;

}


void ETMCanSlaveLogPulseData(unsigned int pulse_count, unsigned int log_data_1, unsigned int log_data_0) {
  static unsigned int data_word_3;
  static unsigned int data_word_2;
  static unsigned int previous_pulse_count;
  
  if ((pulse_count & 0x0001) == 0) {
    // this is an even number pulse count, store the data
    data_word_3 = log_data_1;
    data_word_2 = log_data_0;
    previous_pulse_count = pulse_count;
    return;
  }
  // This is an odd pulse count, send out the data

  if (previous_pulse_count != (pulse_count & 0xFFFE)) {
    data_word_3 = LOG_DEFAULT_VALUE_SLAVE_ALT;
    data_word_2 = LOG_DEFAULT_VALUE_SLAVE_ALT;
  }
  
  pulse_count >>= 1;
  pulse_count &= 0x000F;
  pulse_count <<= 4;
  ETMCanSlaveLogData(pulse_count, data_word_3, data_word_2, log_data_1, log_data_0);

  data_word_3 = LOG_DEFAULT_VALUE_SLAVE;
  data_word_2 = LOG_DEFAULT_VALUE_SLAVE;
}




void ETMCanSlaveLogData(unsigned int packet_id, unsigned int word3, unsigned int word2, unsigned int word1, unsigned int word0) {
  ETMCanMessage log_message;
  
  packet_id |= can_params.address; // Add the board address to the packet_id
  packet_id |= 0b0000010000000000; // Add the Data log identifier
  
  // Format the packet Id for the PIC Register
  packet_id <<= 2;
  log_message.identifier = packet_id;
  log_message.identifier &= 0xFF00;
  log_message.identifier <<= 3;
  log_message.identifier |= (packet_id & 0x00FF);
  
  log_message.word0 = word0;
  log_message.word1 = word1;
  log_message.word2 = word2;
  log_message.word3 = word3;
  
  ETMCanAddMessageToBuffer(&etm_can_slave_tx_message_buffer, &log_message);
  MacroETMCanCheckTXBuffer();
}

 
void ETMCanSlaveCheckForTimeOut(void) {
  if (ETMTickGreaterThanNMilliseconds(SLAVE_TIMEOUT_MILLISECONDS, etm_can_slave_communication_timeout_holding_var)) {
    if (ETMTickGreaterThanNMilliseconds(SLAVE_TIMEOUT_MILLISECONDS, etm_can_slave_communication_timeout_holding_var)) {
      if (_LATF8) {
	_LATF8 = 0;
      } else {
	_LATF8 = 1;
      }
      if (etm_can_slave_com_loss == 0) {
	etm_can_slave_debug_data.can_timeout++;
      }
      etm_can_persistent_data.can_timeout_count = etm_can_slave_debug_data.can_timeout;
      etm_can_slave_com_loss = 1;
    }
  }
}


// DPARKER this needs to get fixed -- WHY?? what is the problem???
void ETMCanSlaveSendUpdateIfNewNotReady(void) {
  static unsigned int previous_ready_status;  // DPARKER - Need better name
  
  if ((previous_ready_status == 0) && (_CONTROL_NOT_READY)) {
    // There is new condition that is causing this board to inhibit operation.
    // Send a status update upstream to Master
    ETMCanSlaveSendStatus();
  }
  previous_ready_status = _CONTROL_NOT_READY;
}

void ETMCanSlaveClearDebug(void) {
  unsigned int n;
  unsigned int *reset_data_ptr;

  reset_data_ptr = (unsigned int*)&etm_can_slave_debug_data;

  for (n = 0; n < ((sizeof(etm_can_slave_debug_data)>>1)-32); n++) {
    *reset_data_ptr = 0;
    reset_data_ptr++;
  }

  etm_can_slave_debug_data.can_build_version = P1395_CAN_SLAVE_VERSION;
  etm_can_slave_debug_data.library_build_version = ETM_LIBRARY_VERSION;
}




void ETMCanSlaveSetLogDataRegister(unsigned int log_register, unsigned int data_for_log) {
  if (log_register > 0x0017) {
    return;
  }
  slave_board_data.log_data[log_register] = data_for_log;
}


void ETMCanSlaveSetDebugRegister(unsigned int debug_register, unsigned int debug_value) {
  if (debug_register > 0x000F) {
    return;
  }
  etm_can_slave_debug_data.debug_reg[debug_register] = debug_value;
}


unsigned int ETMCanSlaveGetComFaultStatus(void) {
  return etm_can_slave_com_loss;
}


unsigned int ETMCanSlaveGetSyncMsgHighSpeedLogging(void) {
  if (etm_can_slave_sync_message.sync_0_control_word.sync_1_high_speed_logging_enabled) {
    return 0xFFFF;
  } else {
    return 0;
  }
}


unsigned int ETMCanSlaveGetSyncMsgCoolingFault(void) {
  if (etm_can_slave_sync_message.sync_0_control_word.sync_4_cooling_fault) {
    return 0xFFFF;
  } else {
    return 0;
  }
}


unsigned int ETMCanSlaveGetSyncMsgSystemHVDisable(void) {
  if (etm_can_slave_sync_message.sync_0_control_word.sync_5_system_hv_disable) {
    return 0xFFFF;
  } else {
    return 0;
  }
}


unsigned int ETMCanSlaveGetSyncMsgGunDriverDisableHeater(void) {
  if (etm_can_slave_sync_message.sync_0_control_word.sync_6_gun_driver_disable_heater) {
    return 0xFFFF;
  } else {
    return 0;
  }
}


unsigned int ETMCanSlaveGetSyncMsgScopeHVVmonEnabled(void) {
  if (etm_can_slave_sync_message.sync_0_control_word.sync_D_scope_HV_HVMON_active) {
    return 0xFFFF;
  } else {
    return 0;
  }
}


unsigned int ETMCanSlaveGetSyncMsgEnableFaultIgnore(void) {
  if (etm_can_slave_sync_message.sync_0_control_word.sync_E_ingnore_faults_enabled) {
    return 0xFFFF;
  } else {
    return 0;
  }
}


unsigned int ETMCanSlaveGetSyncMsgResetEnable(void) {
  if (etm_can_slave_sync_message.sync_0_control_word.sync_0_reset_enable) {
    return 0xFFFF;
  } else {
    return 0;
  }
}



unsigned int last_return_value_level_high_word_count_low_word;  // Pulse Level High Word, Pulse Count Low Word

void ETMCanSlaveTriggerRecieved(void) {
  // Does the slave need to send anything out or just update the next energy level
  unsigned char last_pulse_count;
  last_pulse_count = last_return_value_level_high_word_count_low_word;

  // Disable the Can Interrupt
  _C1IE = 0;
  _C2IE = 0;
  
  if (last_pulse_count == etm_can_slave_sync_message.pulse_count) {
    // we have not recieved a new sync message yet
    // Update pulse_count and pulse_level with the next expected value
    // Check to see if we are in interleave mode

    etm_can_slave_sync_message.pulse_count = (last_pulse_count + 1);
    switch (etm_can_slave_sync_message.next_energy_level)
      {
      
      case DOSE_SELECT_INTERLEAVE_0_1_DOSE_LEVEL_0:
	etm_can_slave_sync_message.next_energy_level = DOSE_SELECT_INTERLEAVE_0_1_DOSE_LEVEL_1;
	break;
	
      case DOSE_SELECT_INTERLEAVE_0_1_DOSE_LEVEL_1:
	etm_can_slave_sync_message.next_energy_level = DOSE_SELECT_INTERLEAVE_0_1_DOSE_LEVEL_0;
	break;
	
      case DOSE_SELECT_INTERLEAVE_2_3_DOSE_LEVEL_2:
	etm_can_slave_sync_message.next_energy_level = DOSE_SELECT_INTERLEAVE_2_3_DOSE_LEVEL_3;
	break;
	
      case DOSE_SELECT_INTERLEAVE_2_3_DOSE_LEVEL_3:
	etm_can_slave_sync_message.next_energy_level = DOSE_SELECT_INTERLEAVE_2_3_DOSE_LEVEL_2;
	break;
	
      default:
	// Don't change the pulse level because we aren't in interleaved mode
	break;
      }
  }

  // Reenable the relevant can interrupt
  if (CXEC_ptr == &C1EC) {
    // We are using CAN1.
    _C1IE = 1;
  } else {
    _C2IE = 1;
  }
}


unsigned char ETMCanSlaveGetPulseLevel(void) {
  return (etm_can_slave_sync_message.next_energy_level & 0x0F);
}

unsigned char ETMCanSlaveGetPulseCount(void) {
  return (etm_can_slave_sync_message.pulse_count);
}

unsigned int ETMCanSlaveGetSetting(unsigned char setting_select) {
  unsigned int *data_ptr;
  data_ptr = (unsigned int*)&slave_data;
  data_ptr += setting_select;
  return *data_ptr;
}


 void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _C1Interrupt(void) {
  _C1IF = 0;
  DoCanInterrupt();
}


void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _C2Interrupt(void) {
  _C2IF = 0;
  DoCanInterrupt();
}


void DoCanInterrupt(void) {
  ETMCanMessage can_message;

  etm_can_slave_debug_data.CXINTF_max |= *CXINTF_ptr;
  
  if (*CXRX0CON_ptr & BUFFER_FULL_BIT) {
    // A message has been received in Buffer Zero
    if (!(*CXRX0CON_ptr & FILTER_SELECT_BIT)) {
      // The command was received by Filter 0
      // It is a SYNC / Next Pulse Level Command
      _LATF7 = 1;
      etm_can_slave_debug_data.can_rx_0_filt_0++;
      ETMCanRXMessage(&can_message, CXRX0CON_ptr);
      *(unsigned int*)&etm_can_slave_sync_message.sync_0_control_word = can_message.word0;
      *(unsigned int*)&etm_can_slave_sync_message.pulse_count = can_message.word1;
      etm_can_slave_sync_message.prf_from_ecb = can_message.word2;
      *(unsigned int*)&etm_can_slave_sync_message.scope_A_select = can_message.word3;
      ClrWdt();
      etm_can_slave_com_loss = 0;
      etm_can_slave_communication_timeout_holding_var = ETMTickGet();
      _LATF7 = 0;
    } else {
      // The commmand was received by Filter 1
      // This is a setting/command from the ECB
      ETMCanRXMessageBuffer(&etm_can_slave_rx_message_buffer, CXRX0CON_ptr);
      etm_can_slave_debug_data.can_rx_0_filt_1++;
    }
    *CXINTF_ptr &= RX0_INT_FLAG_BIT; // Clear the RX0 Interrupt Flag
  }
  
  if (*CXRX1CON_ptr & BUFFER_FULL_BIT) {
    /* 
       A message has been recieved in Buffer 1
       This is a RTSP message - no other use for now
    */
    etm_can_slave_debug_data.can_rx_1_filt_2++;
    *CXINTF_ptr &= RX1_INT_FLAG_BIT; // Clear the RX1 Interrupt Flag
  }
  
  if (!(*CXTX0CON_ptr & TX_REQ_BIT) && ((ETMCanBufferNotEmpty(&etm_can_slave_tx_message_buffer)))) {
    /*
      TX0 is empty and there is a message waiting in the transmit message buffer
      Load the next message into TX0
    */
    ETMCanTXMessageBuffer(&etm_can_slave_tx_message_buffer, CXTX0CON_ptr);
    *CXINTF_ptr &= 0xFFFB; // Clear the TX0 Interrupt Flag
    etm_can_slave_debug_data.can_tx_0++;
  }
  
  if (*CXINTF_ptr & ERROR_FLAG_BIT) {
    // There was some sort of CAN Error
    // DPARKER - figure out which error and fix/reset
    etm_can_slave_debug_data.CXINTF_max |= *CXINTF_ptr;
    etm_can_slave_debug_data.can_error_flag++;
    *CXINTF_ptr &= ~ERROR_FLAG_BIT; // Clear the ERR Flag
  } else {
    // FLASH THE CAN LED
    if (ETMReadPinLatch(can_params.led)) {
      ETMClearPin(can_params.led);
    } else {
      ETMSetPin(can_params.led);
    }
  }
}





void ETMCanSlaveLogHVVmonData(unsigned int sp4, unsigned int sp3, unsigned int sp2, unsigned int sp1, unsigned int sp0) {
  unsigned int data3;
  unsigned int data2;
  unsigned int data1;
  unsigned int data0;

  if (slave_data.scope_hv_vmon_settings == SCOPE_CHANNEL_ACTIVE) {
    data0  = sp0 & 0xFFF;
    data0 |= ((sp1 << 12) & 0xF000);
    
    data1  = (sp1 >> 4) & 0xFF;
    data1 |= ((sp2 <<8) & 0xFF00);
    
    data2  = (sp2>>8) & 0xF;
    data2 |= (sp3 << 4) & 0xFFF0;
    
    data3  = sp4 & 0xFFF;

    ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_HV_VMON_DATA, data3, data2, data1, data0);
  }
}

void ETMCanSlaveLogPulseCurrent(TYPE_PULSE_DATA *pulse_data) {
  static unsigned long start_tick;
  unsigned int send_this_pulse;

  send_this_pulse = 0;
  if (ETMTickGreaterThanNMilliseconds(1000, start_tick)) {
    // It has been at least 1 second since the last time a pulse was logged.
    send_this_pulse = 1;
  }
  
  if (pulse_data->arc_this_pulse) {
    //send_this_pulse = 1;
  }
  if (send_this_pulse) {
    pulse_data_transmit_index = 0;
    Compact12BitInto16Bit(&pulse_data_to_transmit[0], &pulse_data->data[0], 0);
    Compact12BitInto16Bit(&pulse_data_to_transmit[4], &pulse_data->data[5], 1);
    Compact12BitInto16Bit(&pulse_data_to_transmit[8], &pulse_data->data[10], 2);
    Compact12BitInto16Bit(&pulse_data_to_transmit[12], &pulse_data->data[15], 3);
    Compact12BitInto16Bit(&pulse_data_to_transmit[16], &pulse_data->data[20], 4);
    Compact12BitInto16Bit(&pulse_data_to_transmit[20], &pulse_data->data[25], 5);
    Compact12BitInto16Bit(&pulse_data_to_transmit[24], &pulse_data->data[30], 6);
    Compact12BitInto16Bit(&pulse_data_to_transmit[28], &pulse_data->data[35], 7);
    Compact12BitInto16Bit(&pulse_data_to_transmit[32], &pulse_data->data[40], 8);
    Compact12BitInto16Bit(&pulse_data_to_transmit[36], &pulse_data->data[45], 9);
    start_tick = ETMTickGet();
  }
}


void ETMCanSlaveSetScopeDataAddress(unsigned int scope_channel, unsigned int *data_address) {
  if (scope_channel >= SCOPE_DATA_SELECT_SIZE) {
    // Not a valid scope channel
    return;
  }
  scope_data_sources[scope_channel] = data_address;
}


void Compact12BitInto16Bit(unsigned int *transmit_data, unsigned int *source_data, unsigned int count) {
  // Compacts 5 16 bit numbers (assuming 12bit) int 4

  transmit_data[0]  = source_data[0] & 0xFFF;
  transmit_data[0] |= ((source_data[1] << 12) & 0xF000);

  transmit_data[1]  = (source_data[1] >> 4) & 0xFF;
  transmit_data[1] |= ((source_data[2] <<8) & 0xFF00);

  transmit_data[2]  = (source_data[2]>>8) & 0xF;
  transmit_data[2] |= (source_data[3] << 4) & 0xFFF0;

  transmit_data[3]  = source_data[4] & 0xFFF;
  transmit_data[3] |= (count & 0x000F) << 12;
}

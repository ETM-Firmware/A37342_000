#include "A37342_000.h"
#include "FIRMWARE_VERSION.h"
//#include "LTC265X.h"
//#include "ETM_EEPROM.h"

#define OFFSET_ZERO 0 // DPARKER should this be in ETM_ANALOG or ETM_SCALE??


unsigned int test_vprog_0_monitor;


// Changes for working through a reset
unsigned int setup_done;

volatile unsigned int adc_mirror[16];
volatile unsigned int adc_mirror_latest_update;


unsigned int post_pulse_process_count;

// This is firmware for the HV Lambda Board

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


LTC265X U1_LTC2654;
LambdaControlData global_data_A37342;



void DoStateMachine(void);
void EnableHVLambda(void);
void DisableHVLambda(void);
void DoA37342(void);
void UpdateFaultsAndStatusBits(void);
void InitializeA37342(void);
void DoPostPulseProcess(void);
unsigned int CheckAtEOC(void);
void UpdateSystemConfiguration(unsigned int system_configuartion);
void FlashLeds(void);

int main(void) {
  global_data_A37342.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  switch (global_data_A37342.control_state) {
    
  case STATE_STARTUP:
    InitializeA37342();
    ETMCanSlaveStatusUpdateBitNotReady(NOT_READY);
    global_data_A37342.control_state = STATE_WAITING_FOR_CONFIG;
    break;

    
  case STATE_WAITING_FOR_CONFIG:
    DisableHVLambda();
    ETMCanSlaveStatusUpdateBitNotReady(NOT_READY);
    global_data_A37342.startup_counter = 0;
    while (global_data_A37342.control_state == STATE_WAITING_FOR_CONFIG) {
      DoA37342();
      FlashLeds();
      if ((ETMCanSlaveStatusCheckNotConfigured() == 0) && (global_data_A37342.startup_counter > 200))  {
	UpdateSystemConfiguration(ETMCanSlaveGetSetting(SYSTEM_CONFIGURATION_SELECT));
	global_data_A37342.control_state = STATE_WAITING_FOR_POWER;
      }
    }
    break;


  case STATE_WAITING_FOR_POWER:
    DisableHVLambda();
    ETMCanSlaveStatusUpdateBitNotReady(NOT_READY);
    global_data_A37342.hv_lambda_power_wait = 0;
    while (global_data_A37342.control_state == STATE_WAITING_FOR_POWER) {
      DoA37342();
      
      if (ETMCanSlaveGetSyncMsgSystemHVDisable()) {
	global_data_A37342.hv_lambda_power_wait = 0;
      }

      if (global_data_A37342.hv_lambda_power_wait >= AC_POWER_UP_DELAY) {
	global_data_A37342.control_state = STATE_POWER_UP;
      }

      if (ETMCanSlaveStatusReadFaultRegister()) {
	global_data_A37342.control_state = STATE_FAULT_WAIT;
      } 
    }
    break;


  case STATE_POWER_UP:
    global_data_A37342.lambda_reached_eoc = 0;
    EnableHVLambda();
    ETMCanSlaveStatusUpdateBitNotReady(NOT_READY);
    global_data_A37342.hv_lambda_power_wait = 0;
    while (global_data_A37342.control_state == STATE_POWER_UP) {
      DoA37342();
      
      if (CheckAtEOC()) {
	global_data_A37342.control_state = STATE_OPERATE;
      }
      
      if (ETMCanSlaveGetSyncMsgSystemHVDisable()) {
	global_data_A37342.control_state = STATE_WAITING_FOR_POWER;
      }

      if (ETMCanSlaveStatusReadFaultRegister()) {
	global_data_A37342.control_state = STATE_FAULT_WAIT;
      }
      
      if ((PIN_LAMBDA_SUM_FLT == ILL_LAMBDA_FAULT_ACTIVE) && (global_data_A37342.hv_lambda_power_wait > AC_POWER_UP_DELAY)) {
	global_data_A37342.control_state = STATE_WAITING_FOR_POWER;
      }
    }
    break;

  case STATE_OPERATE:
    ETMCanSlaveStatusUpdateBitNotReady(READY);
    while (global_data_A37342.control_state == STATE_OPERATE) {
      DoA37342();
      
      if (ETMCanSlaveGetSyncMsgSystemHVDisable()) {
	global_data_A37342.control_state = STATE_WAITING_FOR_POWER;
      }

      if (ETMCanSlaveStatusReadFaultRegister()) {
	global_data_A37342.control_state = STATE_FAULT_WAIT;
      }
    }
    break;


  case STATE_FAULT_WAIT:
    DisableHVLambda();
    ETMCanSlaveStatusUpdateBitNotReady(NOT_READY);
    global_data_A37342.fault_wait_time = 0;
    while (global_data_A37342.control_state == STATE_FAULT_WAIT) {
      DoA37342();

      if (global_data_A37342.fault_wait_time >= TIME_WAIT_FOR_LAMBDA_TO_SET_FAULT_OUTPUTS) {
	global_data_A37342.control_state = STATE_FAULT;
      }
    }
    break;

    
  case STATE_FAULT:
    DisableHVLambda();
    ETMCanSlaveStatusUpdateBitNotReady(NOT_READY);
    while (global_data_A37342.control_state == STATE_FAULT) {
      DoA37342();

      if (ETMCanSlaveStatusReadFaultRegister() == 0) {
	global_data_A37342.control_state = STATE_WAITING_FOR_CONFIG;
      }
    }
    break;
    

  default:
    global_data_A37342.control_state = STATE_FAULT;
    break;
  }
}

void DoPostPulseProcess(void) {
  // Send the pulse data up to the ECB for logging


  global_data_A37342.pulse_id = ETMCanSlaveGetPulseCount();
  global_data_A37342.pulse_level = ETMCanSlaveGetPulseLevel();

  switch (global_data_A37342.pulse_level) {
    
  case DOSE_SELECT_DOSE_LEVEL_0:
    ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_low_energy_vprog, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_0));
    ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_high_energy_vprog, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_1));
    PIN_LAMBDA_VOLTAGE_SELECT = OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY;
    ETMCanSlaveStatusUpdateLoggedBit(_STATUS_LAMBDA_HIGH_ENERGY, 0);
    break;
    
    
  case DOSE_SELECT_DOSE_LEVEL_1:
    ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_low_energy_vprog, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_0));
    ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_high_energy_vprog, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_1));
    PIN_LAMBDA_VOLTAGE_SELECT = !OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY;
    ETMCanSlaveStatusUpdateLoggedBit(_STATUS_LAMBDA_HIGH_ENERGY, 1);
    break;


  case DOSE_SELECT_DOSE_LEVEL_2:
    ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_low_energy_vprog, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_2));
    ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_high_energy_vprog, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_3));
    PIN_LAMBDA_VOLTAGE_SELECT = OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY;
    ETMCanSlaveStatusUpdateLoggedBit(_STATUS_LAMBDA_HIGH_ENERGY, 0);
    break;


  case DOSE_SELECT_DOSE_LEVEL_3:
    ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_low_energy_vprog, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_2));
    ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_high_energy_vprog, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_3));
    PIN_LAMBDA_VOLTAGE_SELECT = !OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY;
    ETMCanSlaveStatusUpdateLoggedBit(_STATUS_LAMBDA_HIGH_ENERGY, 1);
    break;
  }
  
  // Update the HV Lambda Program Values
  WriteLTC265XTwoChannels(&U1_LTC2654,
			  LTC265X_WRITE_AND_UPDATE_DAC_C,
			  ETMAnalogOutputGetDACValue(&global_data_A37342.analog_output_high_energy_vprog),
			  LTC265X_WRITE_AND_UPDATE_DAC_D,
			  ETMAnalogOutputGetDACValue(&global_data_A37342.analog_output_low_energy_vprog));
  
  global_data_A37342.no_pulse_counter = 0;
}




void DoA37342(void) {
  static unsigned long ten_millisecond_holding_var;
  static unsigned long temp_timer_scope_test;
  static unsigned int  temp_scope_test_b = 0;
  
  ETMCanSlaveDoCan();

  test_vprog_0_monitor = ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_0);

  if (ETMTickRunOnceEveryNMilliseconds(1, &temp_timer_scope_test)) {
      ETMCanSlaveLogHVVmonData(temp_scope_test_b+4,temp_scope_test_b+3,temp_scope_test_b+2,temp_scope_test_b+1,temp_scope_test_b);
      temp_scope_test_b += 5;
      temp_scope_test_b &= 0x0FFF;
      ETMCanSlaveLogHVVmonData(temp_scope_test_b+4,temp_scope_test_b+3,temp_scope_test_b+2,temp_scope_test_b+1,temp_scope_test_b);
      temp_scope_test_b += 5;
      temp_scope_test_b &= 0x0FFF;
  }
  
  if (global_data_A37342.run_post_pulse_process) {
    global_data_A37342.run_post_pulse_process = 0;  
    DoPostPulseProcess();
    post_pulse_process_count++;
  }


  if (ETMTickRunOnceEveryNMilliseconds(10, &ten_millisecond_holding_var)) {
    
    global_data_A37342.hv_lambda_power_wait++;
    global_data_A37342.startup_counter++;

    ETMCanSlaveSetDebugRegister(1, 0);

    ETMCanSlaveSetDebugRegister(2, global_data_A37342.pulse_id);
    ETMCanSlaveSetDebugRegister(3, global_data_A37342.pulse_level);
    
    // Update debugging information
    //ETMCanSlaveSetDebugRegister(0, global_data_A37342.analog_input_lambda_vmon.reading_scaled_and_calibrated);
    //ETMCanSlaveSetDebugRegister(1, global_data_A37342.analog_input_lambda_vpeak.reading_scaled_and_calibrated);
    //ETMCanSlaveSetDebugRegister(2, global_data_A37342.analog_input_lambda_imon.reading_scaled_and_calibrated);
    //ETMCanSlaveSetDebugRegister(3, global_data_A37342.analog_input_lambda_heat_sink_temp.reading_scaled_and_calibrated);
    //ETMCanSlaveSetDebugRegister(0x4, slave_board_data.local_data[0]);
    //ETMCanSlaveSetDebugRegister(0x5, slave_board_data.local_data[1]);
    //ETMCanSlaveSetDebugRegister(0x6, slave_board_data.local_data[2]);
    //ETMCanSlaveSetDebugRegister(0x7, slave_board_data.local_data[3]);
    //ETMCanSlaveSetDebugRegister(0x8, ETMCanSlaveGetPulseCount());
    //ETMCanSlaveSetDebugRegister(0x9, post_pulse_process_count);
    //ETMCanSlaveSetDebugRegister(0xA, global_data_A37342.control_state);

    // ETMCanSlaveSetDebugRegister(0xE, RESERVED);
    // ETMCanSlaveSetDebugRegister(0xF, RESERVED);

    
    

    // Update all the logging data
    ETMCanSlaveSetLogDataRegister(0, ETMAnalogInputGetReading(&global_data_A37342.analog_input_lambda_vpeak));
    ETMCanSlaveSetLogDataRegister(1, ETMAnalogInputGetReading(&global_data_A37342.analog_input_lambda_imon));
    ETMCanSlaveSetLogDataRegister(2, ETMAnalogInputGetReading(&global_data_A37342.analog_input_lambda_vmon));
    ETMCanSlaveSetLogDataRegister(3, ETMAnalogInputGetReading(&global_data_A37342.analog_input_lambda_heat_sink_temp));
    ETMCanSlaveSetLogDataRegister(4, ETMAnalogOutputGetSetPoint(&global_data_A37342.analog_output_low_energy_vprog));
    ETMCanSlaveSetLogDataRegister(5, ETMAnalogOutputGetSetPoint(&global_data_A37342.analog_output_high_energy_vprog));
    ETMCanSlaveSetLogDataRegister(7, global_data_A37342.eoc_not_reached_count);
    ETMCanSlaveSetLogDataRegister(8, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_0));
    ETMCanSlaveSetLogDataRegister(9, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_1));
    ETMCanSlaveSetLogDataRegister(10, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_2));
    ETMCanSlaveSetLogDataRegister(11, ETMCanSlaveGetSetting(HVPS_SET_POINT_DOSE_3));
    

    if (global_data_A37342.control_state == STATE_FAULT_WAIT) {
      global_data_A37342.fault_wait_time++;
      if (global_data_A37342.fault_wait_time >= TIME_WAIT_FOR_LAMBDA_TO_SET_FAULT_OUTPUTS) {
	global_data_A37342.fault_wait_time = TIME_WAIT_FOR_LAMBDA_TO_SET_FAULT_OUTPUTS;
      }
    }

    if (global_data_A37342.control_state != STATE_OPERATE) {
      // Update the HV Lambda Program Values
      WriteLTC265XTwoChannels(&U1_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_C,
			      ETMAnalogOutputGetDACValue(&global_data_A37342.analog_output_high_energy_vprog),
			      LTC265X_WRITE_AND_UPDATE_DAC_D,
			      ETMAnalogOutputGetDACValue(&global_data_A37342.analog_output_low_energy_vprog));      
      
    } else {
      global_data_A37342.no_pulse_counter++;
      if (global_data_A37342.no_pulse_counter >= HV_ON_LAMBDA_SET_POINT_REFRESH_RATE_WHEN_NOT_PULSING) {
	// A long time has passed without updating the Lambda Set points
	// Update the HV Lambda Program Values
	global_data_A37342.no_pulse_counter = 0;
	WriteLTC265XTwoChannels(&U1_LTC2654,
				LTC265X_WRITE_AND_UPDATE_DAC_C,
				ETMAnalogOutputGetDACValue(&global_data_A37342.analog_output_high_energy_vprog),
				LTC265X_WRITE_AND_UPDATE_DAC_D,
				ETMAnalogOutputGetDACValue(&global_data_A37342.analog_output_low_energy_vprog));      
      } 
    }
    
    UpdateFaultsAndStatusBits();
  }
}

  
unsigned int CheckAtEOC(void) {
  unsigned int minimum_vmon_at_eoc;
  unsigned int vmon;

  if (PIN_LAMBDA_VOLTAGE_SELECT == OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY) {
    minimum_vmon_at_eoc = ETMScaleFactor2(ETMAnalogOutputGetSetPoint(&global_data_A37342.analog_output_low_energy_vprog), MACRO_DEC_TO_CAL_FACTOR_2(.90), 0);
  } else {
    minimum_vmon_at_eoc = ETMScaleFactor2(ETMAnalogOutputGetSetPoint(&global_data_A37342.analog_output_high_energy_vprog), MACRO_DEC_TO_CAL_FACTOR_2(.90), 0);
  }
  
  vmon = ETMScaleFactor16(ADCBUF1 << 4, MACRO_DEC_TO_SCALE_FACTOR_16(VMON_SCALE_FACTOR), OFFSET_ZERO);

  ETMCanSlaveSetDebugRegister(0xE, vmon);
  ETMCanSlaveSetDebugRegister(0xF, minimum_vmon_at_eoc);

  if (vmon > minimum_vmon_at_eoc) {
    return 1;
  }
  return 0;
}


void UpdateFaultsAndStatusBits(void) {

  // Update the digital input status pins
  if (PIN_LAMBDA_EOC == ILL_LAMBDA_AT_EOC) {
    ETMCanSlaveStatusUpdateNotLoggedBit(_STATUS_LAMBDA_AT_EOC, 1);
  } else {
    ETMCanSlaveStatusUpdateNotLoggedBit(_STATUS_LAMBDA_AT_EOC, 0);
  }
  
  // Update the logged and not logged status bits
  ETMDigitalUpdateInput(&global_data_A37342.digital_hv_not_powered, PIN_LAMBDA_NOT_POWERED);
  if (ETMDigitalFilteredOutput(&global_data_A37342.digital_hv_not_powered) == ILL_LAMBDA_NOT_POWERED) {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_NOT_POWERED, 1);
  } else {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_NOT_POWERED, 0);
  }

  ETMDigitalUpdateInput(&global_data_A37342.digital_sum_flt, PIN_LAMBDA_SUM_FLT);
  if (ETMDigitalFilteredOutput(&global_data_A37342.digital_sum_flt) == ILL_LAMBDA_FAULT_ACTIVE) {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_SUM_FAULT, 1);
  } else {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_SUM_FAULT, 0);
  }
       
  ETMDigitalUpdateInput(&global_data_A37342.digital_hv_off,  PIN_LAMBDA_HV_ON_READBACK);
  if (ETMDigitalFilteredOutput(&global_data_A37342.digital_hv_off) != ILL_LAMBDA_HV_ON) {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_READBACK_HV_OFF, 1);
  } else {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_READBACK_HV_OFF, 0);
  }


  if (ETMCanSlaveGetSetting(SYSTEM_CONFIGURATION_SELECT) == SYSTEM_CONFIGURATION_2_5_R) {
    // This pin is not valid on the 2.5 ignore phase loss
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_PHASE_LOSS, 0);
  } else {
    ETMDigitalUpdateInput(&global_data_A37342.digital_phase_loss,  PIN_LAMBDA_PHASE_LOSS_FLT);
    if (ETMDigitalFilteredOutput(&global_data_A37342.digital_phase_loss) == ILL_LAMBDA_FAULT_ACTIVE) {
      ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_PHASE_LOSS, 1);
    } else {
      ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_PHASE_LOSS, 0);
    }
  }

  ETMDigitalUpdateInput(&global_data_A37342.digital_over_temp,  PIN_LAMBDA_OVER_TEMP_FLT);
  if (ETMDigitalFilteredOutput(&global_data_A37342.digital_over_temp) == ILL_LAMBDA_FAULT_ACTIVE) {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_OVER_TEMP, 1);
  } else {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_OVER_TEMP, 0);
  }
  
  ETMDigitalUpdateInput(&global_data_A37342.digital_interlock,  PIN_LAMBDA_INTERLOCK_FLT);
  if (ETMDigitalFilteredOutput(&global_data_A37342.digital_interlock) == ILL_LAMBDA_FAULT_ACTIVE) {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_INTERLOCK, 1);
  } else {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_INTERLOCK, 0);
  }
  
  ETMDigitalUpdateInput(&global_data_A37342.digital_load_flt,  PIN_LAMBDA_LOAD_FLT);
  if (ETMDigitalFilteredOutput(&global_data_A37342.digital_load_flt) == ILL_LAMBDA_FAULT_ACTIVE) {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_LOAD_FLT, 1);
  } else {
    ETMCanSlaveStatusUpdateLoggedBit(_LOGGED_LAMBDA_LOAD_FLT, 0);
  }
}




void InitializeA37342(void) {

  // Configure Inhibit Interrupt
  _INT1IP = 7; // This must be the highest priority interrupt
  _INT1EP = 1; // Negative Transition
  
  
  // Initialize all I/O Registers
  TRISA = A37342_TRISA_VALUE;
  TRISB = A37342_TRISB_VALUE;
  TRISC = A37342_TRISC_VALUE;
  TRISD = A37342_TRISD_VALUE;
  TRISF = A37342_TRISF_VALUE;
  TRISG = A37342_TRISG_VALUE;


  // Configure ADC Interrupt
  _ADIP   = 5; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)


  // Initialize TMR1 & T1 Interrupt
  TMR1  = 0;
  PR1   = TMR1_LAMBDA_CHARGE_PERIOD;
  _T1IE = 0;
  _T1IF = 0;
  T1CON = T1CON_VALUE;
  _T1IP = 6;

  // Initialize ETM Tick using TMR5
  ETMTickInitialize(FCY_CLK, ETM_TICK_USE_TIMER_5);
  

  /*
  ETMEEPromUseI2C();
  ETMEEPromConfigureI2CDevice(EEPROM_SIZE_64K_BITS,
			      FCY_CLK,
			      ETM_I2C_400K_BAUD,
			      EEPROM_I2C_ADDRESS_0,
			      I2C_PORT);
  */

  ETMEEPromUseInternal();
  
  // Initialize the Can module
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_HV_LAMBDA_BOARD,
			_PIN_RG13, 4,
			_PIN_RA7, _PIN_RG12);

  ETMCanSlaveLoadConfiguration(37342, SOFTWARE_DASH_NUMBER, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV);

  ETMCanSlaveSetScopeDataAddress(0, &global_data_A37342.pulse_id);
  ETMCanSlaveSetScopeDataAddress(1, &global_data_A37342.eoc_not_reached_count);
  ETMCanSlaveSetScopeDataAddress(2, &global_data_A37342.no_pulse_counter);
  ETMCanSlaveSetScopeDataAddress(3, &global_data_A37342.vmon_at_eoc_period);
  ETMCanSlaveSetScopeDataAddress(4, &test_vprog_0_monitor);
  

  // Initialize LTC DAC
  SetupLTC265X(&U1_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);
  DisableHVLambda();

   // Initialize Digital Input Filters
  ETMDigitalInitializeInput(&global_data_A37342.digital_hv_not_powered , ILL_LAMBDA_NOT_POWERED, 10);
  ETMDigitalInitializeInput(&global_data_A37342.digital_sum_flt        , !ILL_LAMBDA_FAULT_ACTIVE, 10);
  ETMDigitalInitializeInput(&global_data_A37342.digital_hv_off         , ILL_LAMBDA_HV_ON, 10);
  ETMDigitalInitializeInput(&global_data_A37342.digital_phase_loss     , !ILL_LAMBDA_FAULT_ACTIVE, 10);
  ETMDigitalInitializeInput(&global_data_A37342.digital_over_temp      , !ILL_LAMBDA_FAULT_ACTIVE, 10);
  ETMDigitalInitializeInput(&global_data_A37342.digital_interlock      , !ILL_LAMBDA_FAULT_ACTIVE, 10);
  ETMDigitalInitializeInput(&global_data_A37342.digital_load_flt       , !ILL_LAMBDA_FAULT_ACTIVE, 10);



  //Initialize the internal ADC for Startup Power Checks
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCSSL = ADCSSL_SETTING;

  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;

  PIN_LAMBDA_VOLTAGE_SELECT = OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY;  
}

void UpdateSystemConfiguration(unsigned int system_configuartion) {
  switch (system_configuartion) {

  case SYSTEM_CONFIGURATION_6_4_R:
    // This used 20KV Lambda
    global_data_A37342.vprog_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(2.66667);
    global_data_A37342.vmon_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(.31250);
    global_data_A37342.hvps_max_program = 20000;
    global_data_A37342.software_config_number = 495;
    break;

  case SYSTEM_CONFIGURATION_6_4_M:
    // This uses 25KV Lambda
    global_data_A37342.vprog_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(2.13125);
    global_data_A37342.vmon_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(.3906);
    global_data_A37342.hvps_max_program = 22100;
    global_data_A37342.software_config_number = 100;
    break;

  case SYSTEM_CONFIGURATION_6_4_S:
    // This uses 20KV Lambda
    global_data_A37342.vprog_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(2.66667);
    global_data_A37342.vmon_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(.31250);
    global_data_A37342.hvps_max_program = 20000;
    global_data_A37342.software_config_number = 495;
    break;

  case SYSTEM_CONFIGURATION_2_5_R:
    // This uses 18KV Lambda
    global_data_A37342.vprog_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(2.96296);
    global_data_A37342.vmon_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(.28125);
    global_data_A37342.hvps_max_program = 18000;
    global_data_A37342.software_config_number = 0;
    break;

  default:
    // Assume 25KV Lambda so that output stays low
    global_data_A37342.vprog_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(2.13125);
    global_data_A37342.vmon_scale_factor = MACRO_DEC_TO_SCALE_FACTOR_16(.3906);
    global_data_A37342.hvps_max_program = 22100;
    global_data_A37342.software_config_number = 100;


  // Initialize the Analog input/output data structures
  
  ETMAnalogInputInitialize(&global_data_A37342.analog_input_lambda_vmon,
			   global_data_A37342.vmon_scale_factor,
			   OFFSET_ZERO,	   
			   ETM_ANALOG_AVERAGE_64_SAMPLES);

  
  ETMAnalogInputInitialize(&global_data_A37342.analog_input_lambda_vpeak,
			   global_data_A37342.vmon_scale_factor,
			   OFFSET_ZERO,
			   ETM_ANALOG_AVERAGE_64_SAMPLES);
  
  ETMAnalogInputInitialize(&global_data_A37342.analog_input_lambda_imon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.40179),
			   OFFSET_ZERO,
			   ETM_ANALOG_AVERAGE_64_SAMPLES);

  ETMAnalogInputInitialize(&global_data_A37342.analog_input_lambda_heat_sink_temp,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.78125),
			   10000,
			   ETM_ANALOG_AVERAGE_64_SAMPLES);

  ETMAnalogOutputInitialize(&global_data_A37342.analog_output_low_energy_vprog,
			    global_data_A37342.vprog_scale_factor,
			    OFFSET_ZERO,
			    global_data_A37342.hvps_max_program,
			    HV_LAMBDA_MIN_VPROG,
			    HV_LAMBDA_DAC_ZERO_OUTPUT);

  ETMAnalogOutputInitialize(&global_data_A37342.analog_output_high_energy_vprog,
			    global_data_A37342.vprog_scale_factor,
			    OFFSET_ZERO,
			    global_data_A37342.hvps_max_program,
			    HV_LAMBDA_MIN_VPROG,
			    HV_LAMBDA_DAC_ZERO_OUTPUT);

  ETMAnalogOutputInitialize(&global_data_A37342.analog_output_spare,
			    MACRO_DEC_TO_SCALE_FACTOR_16(5.33333),
			    OFFSET_ZERO,
			    10000,
			    0,
			    0);

  ETMAnalogOutputInitialize(&global_data_A37342.analog_output_adc_test,
			    MACRO_DEC_TO_SCALE_FACTOR_16(1),
			    OFFSET_ZERO,
			    0xFFFF,
			    0,
			    0);

  
  ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_spare, 3000);
  ETMAnalogOutputSetPoint(&global_data_A37342.analog_output_adc_test, ADC_DAC_TEST_VALUE);

  ETMAnalogOutputEnable(&global_data_A37342.analog_output_spare);
  ETMAnalogOutputEnable(&global_data_A37342.analog_output_adc_test);

  // Update the spare analog output and the DAC test output
  WriteLTC265XTwoChannels(&U1_LTC2654,
			  LTC265X_WRITE_AND_UPDATE_DAC_A,
			  ETMAnalogOutputGetDACValue(&global_data_A37342.analog_output_spare),
			  LTC265X_WRITE_AND_UPDATE_DAC_B,
			  ETMAnalogOutputGetDACValue(&global_data_A37342.analog_output_adc_test));

  }
}


void FlashLeds(void) {
  switch (((global_data_A37342.startup_counter >> 4) & 0b11)) {
    
  case 0:
    _LATA7 = !OLL_LED_ON;
    _LATG12 = !OLL_LED_ON;
    _LATG13 = !OLL_LED_ON;
    break;
    
  case 1:
    _LATA7 = OLL_LED_ON;
    _LATG12 = !OLL_LED_ON;
    _LATG13 = !OLL_LED_ON;
    break;
    
  case 2:
    _LATA7 = OLL_LED_ON;
    _LATG12 = OLL_LED_ON;
    _LATG13 = !OLL_LED_ON;
    break;
    
  case 3:
    _LATA7 = OLL_LED_ON;
    _LATG12 = OLL_LED_ON;
    _LATG13 = OLL_LED_ON;
    break;
  }
}


void EnableHVLambda(void) {
  // Set the enable register in the DAC structure
  // The next time the DAC is updated it will be updated with the most recent high/low energy values
  ETMAnalogOutputEnable(&global_data_A37342.analog_output_high_energy_vprog);
  ETMAnalogOutputEnable(&global_data_A37342.analog_output_low_energy_vprog);
  
  // Set digital output to enable HV_ON of the lambda
  PIN_LAMBDA_ENABLE = OLL_ENABLE_LAMBDA;
  
  // Set digital output to inhibit the lambda
  PIN_LAMBDA_INHIBIT = !OLL_INHIBIT_LAMBDA;
  
  _INT1IF = 0;
  _INT1IE = 1;  
}




void DisableHVLambda(void) {
  // Clear the enable register in the DAC structure
  // The next time the DAC is updated it will be updated with the "disabled" value
  ETMAnalogOutputDisable(&global_data_A37342.analog_output_high_energy_vprog);
  ETMAnalogOutputDisable(&global_data_A37342.analog_output_low_energy_vprog);

  //_INT1IE = 0;  DPARKER FORCE ENABLE
  _INT1IE = 1;

 
  // Set digital output to inhibit the lambda
  PIN_LAMBDA_INHIBIT = OLL_INHIBIT_LAMBDA;
  
  // Set digital output to disable HV_ON of the lambda
  PIN_LAMBDA_ENABLE = !OLL_ENABLE_LAMBDA;
}






void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  /*
    Buffer Data
    BUF0/BUF8 = Vpeak
    BUF1/BUF9 = Vmon
    BUF2/BUFA = Imon
    BUF3/BUFB = Vmon
    BUF4/BUFC = Vprog
    BUF5/BUFD = Vmon
  */

  _ADIF = 0;

  // Copy Data From Buffer to RAM
  if (_BUFS) {
    // read ADCBUF 0-7
    ETMAnalogInputUpdate(&global_data_A37342.analog_input_lambda_vpeak, ADCBUF0);
    ETMAnalogInputUpdate(&global_data_A37342.analog_input_lambda_vmon, ADCBUF1);
    ETMAnalogInputUpdate(&global_data_A37342.analog_input_lambda_imon, ADCBUF2);
    ETMAnalogInputUpdate(&global_data_A37342.analog_input_lambda_heat_sink_temp, ADCBUF4);
    
    adc_mirror[0x1] = ADCBUF1;
    adc_mirror[0x3] = ADCBUF3;
    adc_mirror[0x4] = ADCBUF4;
    adc_mirror[0x5] = ADCBUF5;
    adc_mirror_latest_update = 1;
  } else {
    // read ADCBUF 8-15
    ETMAnalogInputUpdate(&global_data_A37342.analog_input_lambda_vpeak, ADCBUF8);
    ETMAnalogInputUpdate(&global_data_A37342.analog_input_lambda_vmon, ADCBUF9);
    ETMAnalogInputUpdate(&global_data_A37342.analog_input_lambda_imon, ADCBUFA);
    ETMAnalogInputUpdate(&global_data_A37342.analog_input_lambda_heat_sink_temp, ADCBUFC);
    
    adc_mirror[0x9] = ADCBUF9;
    adc_mirror[0xB] = ADCBUFB;
    adc_mirror[0xC] = ADCBUFC;
    adc_mirror[0xD] = ADCBUFD;
    adc_mirror_latest_update = 0;
  }
}


/*
  A trigger on INT1 (High to Low Tansition) Starts the charging process
  After Charge time period (2.1 mS for 400Hz System) the HVPS is inhibited


 */

void __attribute__((interrupt, shadow, no_auto_psv)) _INT1Interrupt(void) {
  T1CONbits.TON = 1; // DPARKER MOVE THIS TO PREPROLOGUE
  PIN_LAMBDA_INHIBIT = !OLL_INHIBIT_LAMBDA; // DPARKER MOVE THIS TO PREPROLOGUE
  
  _INT1IF = 0;
  _T1IF = 0;
  _T1IE = 1;
  
  global_data_A37342.run_post_pulse_process = 1;
  global_data_A37342.pulse_id = ETMCanSlaveGetPulseCount();
  global_data_A37342.pulse_level = ETMCanSlaveGetPulseLevel();
  
}


void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
  unsigned int adc_mirror_19;
  unsigned int adc_mirror_3B;
  unsigned int adc_mirror_5D;

  unsigned int latest_adc_reading_B3;
  unsigned int latest_adc_reading_D5;
  unsigned int latest_adc_reading_19;
  unsigned int latest_adc_reading_3B;
  unsigned int latest_adc_reading_5D;

  unsigned int vmon;
  unsigned int minimum_vmon_at_eoc;

  /*
    This interrupt indicates that the cap charger should have finished charging
    This interrupt is called X us after the HVPS is enabled
    It samples the HVPS voltage and Disables the HVPS so that it will not cause the thyratron to latch
    If the HVPS is too low, the lambda eoc not reached bit is set
  */


  PIN_LAMBDA_INHIBIT = OLL_INHIBIT_LAMBDA;  // DPARKER - MOVE THIS TO THE PREPROLOGUE

  // Prepare for the next Charge Period
  T1CONbits.TON = 0; 
  TMR1 = 0;
  _T1IF = 0;
  _T1IE = 0;
  
  
  // Save the most recent vmon adc reading
  //_ADIE = 0;  // THIS Interrupt is higher Priority than the ADC interrupt so no need to disable the ADC interrupt
  // NEED to prevent updates to adc_mirror or it could overwrite calculation
  if (adc_mirror_latest_update) {
    // The ADC is currently filling 0x8 - 0xF
    latest_adc_reading_B3 = adc_mirror[0x3];
    latest_adc_reading_D5 = adc_mirror[0x5];
    latest_adc_reading_19 = ADCBUF9;
    latest_adc_reading_3B = ADCBUFB;
    latest_adc_reading_5D = ADCBUFD;
    
    adc_mirror_19 = adc_mirror[0x9];
    adc_mirror_3B = adc_mirror[0xB];
    adc_mirror_5D = adc_mirror[0xD];
    global_data_A37342.vprog_at_eoc_period = ETMScaleFactor16(ADCBUF4 << 4,
							      MACRO_DEC_TO_SCALE_FACTOR_16(VMON_SCALE_FACTOR), 0);
    
    
  } else {
    // The ADC is currently filling 0x0 - 0x7
    latest_adc_reading_B3 = adc_mirror[0xB];
    latest_adc_reading_D5 = adc_mirror[0xD];
    latest_adc_reading_19 = ADCBUF1;
    latest_adc_reading_3B = ADCBUF3;
    latest_adc_reading_5D = ADCBUF5;

    adc_mirror_19 = adc_mirror[0x1];
    adc_mirror_3B = adc_mirror[0x3];
    adc_mirror_5D = adc_mirror[0x5];
    global_data_A37342.vprog_at_eoc_period = ETMScaleFactor16(ADCBUFC << 4,
							      MACRO_DEC_TO_SCALE_FACTOR_16(VMON_SCALE_FACTOR), 0);
  }
  //_ADIE = 1;
  
  if (latest_adc_reading_3B == adc_mirror_3B) {
    // Either ADCBUF3/B has not been updated yet (or the value did not change over the previous 12 samples in which case we can use any value)
    if (latest_adc_reading_19 == adc_mirror_19) {
      // We have not updated ADCBUF1/9 yet (or again the value did not change in which case any value is valid)
      vmon = latest_adc_reading_D5; // This sample is (32 TAD -> 50 TAD) Old
    } else {
      // We have updated ADCBUF1/9 
      vmon = latest_adc_reading_19; // This sample is (14 TAD -> 50 TAD) OLD
    }
  } else {
    // ADCBUF3/B has been updated
    if (latest_adc_reading_5D == adc_mirror_5D) {
      // We have not updated ADCBUF5/D yet (or the value did not change in which case any value is valid)
      vmon = latest_adc_reading_3B; // This sample is (32 TAD -> 50 TAD) Old
    } else {
      // We have updated ADCBUF5/D
      vmon = latest_adc_reading_5D; // This sample is (14 TAD -> 50 TAD) OLD
    } 
  }


  if (PIN_LAMBDA_VOLTAGE_SELECT == OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY) {
    minimum_vmon_at_eoc = ETMScaleFactor2(ETMAnalogOutputGetSetPoint(&global_data_A37342.analog_output_low_energy_vprog), MACRO_DEC_TO_CAL_FACTOR_2(.90), 0);
  } else {
    minimum_vmon_at_eoc = ETMScaleFactor2(ETMAnalogOutputGetSetPoint(&global_data_A37342.analog_output_high_energy_vprog), MACRO_DEC_TO_CAL_FACTOR_2(.90), 0);
  }
  
  // Check that we reached EOC
  if (global_data_A37342.vmon_at_eoc_period > minimum_vmon_at_eoc) {
    global_data_A37342.lambda_reached_eoc = 1;
  } else {
    global_data_A37342.eoc_not_reached_count++;
    global_data_A37342.lambda_reached_eoc = 0;
  }

  // DPARKER, consider adding delay so that the logging message isn't scheduled durring pulse


  if (TEST_POINT_B == 1) {
    TEST_POINT_B = 0;
  } else {
    TEST_POINT_B = 1;
  }
  if (ETMCanSlaveGetSyncMsgHighSpeedLogging()) {
    /*
    ETMCanSlaveLogPulseData(global_data_A37342.pulse_id,
			    global_data_A37342.vmon_at_eoc_period,
			    global_data_A37342.vprog_at_eoc_period);
    */
    // DPARKER TESTING
    ETMCanSlaveLogPulseData(global_data_A37342.pulse_id,
			    222,
			    global_data_A37342.pulse_id);
  }

  ETMCanSlaveTriggerRecieved();

}
  

void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}




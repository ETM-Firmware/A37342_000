#include "A36926_000.h"
#include "FIRMWARE_VERSION.h"
//#include "LTC265X.h"
//#include "ETM_EEPROM.h"


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
LambdaControlData global_data_A36926;



void DoStateMachine(void);
void EnableHVLambda(void);
void DisableHVLambda(void);
void DoA36926(void);
void UpdateFaultsAndStatusBits(void);
void InitializeA36926(void);
void DoPostPulseProcess(void);

int main(void) {
  global_data_A36926.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}



void DoStateMachine(void) {
  switch (global_data_A36926.control_state) {
    
  case STATE_STARTUP:
    InitializeA36926();
    DisableHVLambda();
    _CONTROL_NOT_CONFIGURED = 1;
    _CONTROL_NOT_READY = 1;
    global_data_A36926.control_state = STATE_WAITING_FOR_CONFIG;
    break;

    
  case STATE_WAITING_FOR_CONFIG:
    DisableHVLambda();
    _CONTROL_NOT_READY = 1;
    while (global_data_A36926.control_state == STATE_WAITING_FOR_CONFIG) {
      DoA36926();
      if (_CONTROL_NOT_CONFIGURED == 0) {
	global_data_A36926.control_state = STATE_WAITING_FOR_POWER;
      }
    }
    break;


  case STATE_WAITING_FOR_POWER:
    DisableHVLambda();
    _CONTROL_NOT_READY = 1;
    _FAULT_REGISTER = 0;
    while (global_data_A36926.control_state == STATE_WAITING_FOR_POWER) {
      DoA36926();
      
      if (!ETMCanSlaveGetSyncMsgSystemHVDisable()) {
	global_data_A36926.control_state = STATE_POWER_UP;
      }

      if (_FAULT_REGISTER != 0) {
	global_data_A36926.control_state = STATE_FAULT_WAIT;
      } 
    }
    break;



  case STATE_POWER_UP:
    EnableHVLambda();
    _CONTROL_NOT_READY = 1;
    global_data_A36926.power_up_delay_counter = 0;
    while (global_data_A36926.control_state == STATE_POWER_UP) {
      DoA36926();
      
      if ((global_data_A36926.power_up_delay_counter >= POWER_UP_DELAY)) {
	global_data_A36926.control_state = STATE_OPERATE;
      }      

      if (ETMCanSlaveGetSyncMsgSystemHVDisable()) {
	global_data_A36926.control_state = STATE_WAITING_FOR_POWER;
      }

      if (_FAULT_REGISTER != 0) {
	global_data_A36926.control_state = STATE_FAULT_WAIT;
      }
    }
    break;

  case STATE_OPERATE:
    _CONTROL_NOT_READY = 0;
    while (global_data_A36926.control_state == STATE_OPERATE) {
      DoA36926();
      
      if (ETMCanSlaveGetSyncMsgSystemHVDisable()) {
	global_data_A36926.control_state = STATE_WAITING_FOR_POWER;
      }
      
      if (_FAULT_REGISTER != 0) {
	global_data_A36926.control_state = STATE_FAULT_WAIT;
      }
    }
    break;


  case STATE_FAULT_WAIT:
    DisableHVLambda();
    _CONTROL_NOT_READY = 1;
    global_data_A36926.fault_wait_time = 0;
    while (global_data_A36926.control_state == STATE_FAULT_WAIT) {
      DoA36926();

      if (global_data_A36926.fault_wait_time >= TIME_WAIT_FOR_LAMBDA_TO_SET_FAULT_OUTPUTS) {
	global_data_A36926.control_state = STATE_FAULT;
      }
    }
    break;

    
  case STATE_FAULT:
    DisableHVLambda();
    _CONTROL_NOT_READY = 1;
    while (global_data_A36926.control_state == STATE_FAULT) {
      DoA36926();

      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	global_data_A36926.control_state = STATE_WAITING_FOR_CONFIG;
      }
    }
    break;
    

  default:
    global_data_A36926.control_state = STATE_FAULT;
    break;
  }
}


void DoPostPulseProcess(void) {
  // Send the pulse data up to the ECB for logging
  if (ETMCanSlaveGetSyncMsgHighSpeedLogging()) {
    ETMCanSlaveLogPulseData(ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_FAST_LOG_0,
			    global_data_A36926.pulse_id,
			    ETMScaleFactor16(global_data_A36926.vmon_store_1 << 4, MACRO_DEC_TO_SCALE_FACTOR_16(VMON_SCALE_FACTOR), OFFSET_ZERO), // This is the most recent lambda voltage ADC reading at the end of the chrage period
			    ETMScaleFactor16(global_data_A36926.vmon_store_3 << 4, MACRO_DEC_TO_SCALE_FACTOR_16(VMON_SCALE_FACTOR), OFFSET_ZERO), // This is the most recent lambda voltage ADC reading when a pulse is triggered
			    ETMScaleFactor16(global_data_A36926.vmon_store_2 << 4, MACRO_DEC_TO_SCALE_FACTOR_16(VMON_SCALE_FACTOR), OFFSET_ZERO)); // This is the most recent lambda vpeak ADC reading at the end of the chrage period
    // DPARKER also get better names than vmon_store_X
  }
  
  

  // Update the HV Lambda Program Values
  ETMAnalogScaleCalibrateDACSetting(&global_data_A36926.analog_output_high_energy_vprog);
  ETMAnalogScaleCalibrateDACSetting(&global_data_A36926.analog_output_low_energy_vprog);
  WriteLTC265XTwoChannels(&U1_LTC2654,
			  LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36926.analog_output_high_energy_vprog.dac_setting_scaled_and_calibrated,
			  LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36926.analog_output_low_energy_vprog.dac_setting_scaled_and_calibrated);
  
  global_data_A36926.no_pulse_counter = 0;
}




void DoA36926(void) {
  ETMCanSlaveDoCan();
  
  if (ETMCanSlaveIsNextPulseLevelHigh()) {
    PIN_LAMBDA_VOLTAGE_SELECT = !OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY;
    _STATUS_LAMBDA_HIGH_ENERGY = 1;
  } else {
    PIN_LAMBDA_VOLTAGE_SELECT = OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY;
    _STATUS_LAMBDA_HIGH_ENERGY = 0;
  }

  if (global_data_A36926.run_post_pulse_process) {
    global_data_A36926.run_post_pulse_process = 0;  
    DoPostPulseProcess();
    post_pulse_process_count++;
  }
  
  if (_T3IF) {
    // Timer has expired so execute the scheduled code (should be once every 10ms unless the configuration file is changes
    _T3IF = 0;
    

    // Update debugging information
    ETMCanSlaveSetDebugRegister(0, global_data_A36926.analog_input_lambda_vmon.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(1, global_data_A36926.analog_input_lambda_vpeak.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(2, global_data_A36926.analog_input_lambda_imon.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(3, global_data_A36926.analog_input_lambda_heat_sink_temp.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(0x4, slave_board_data.local_data[0]);
    ETMCanSlaveSetDebugRegister(0x5, slave_board_data.local_data[1]);
    ETMCanSlaveSetDebugRegister(0x6, slave_board_data.local_data[2]);
    ETMCanSlaveSetDebugRegister(0x7, slave_board_data.local_data[3]);
    ETMCanSlaveSetDebugRegister(0x8, ETMCanSlaveGetPulseCount());
    ETMCanSlaveSetDebugRegister(0x9, post_pulse_process_count);
    ETMCanSlaveSetDebugRegister(0xF, global_data_A36926.control_state);


    // Update all the logging data
    slave_board_data.log_data[0] = global_data_A36926.analog_input_lambda_vpeak.reading_scaled_and_calibrated;
    slave_board_data.log_data[1] = global_data_A36926.analog_output_low_energy_vprog.set_point;
    slave_board_data.log_data[2] = global_data_A36926.analog_output_high_energy_vprog.set_point;
    slave_board_data.log_data[3] = global_data_A36926.false_trigger_total_count;
    slave_board_data.log_data[4] = global_data_A36926.analog_input_lambda_heat_sink_temp.reading_scaled_and_calibrated;
    slave_board_data.log_data[5] = global_data_A36926.analog_input_lambda_imon.reading_scaled_and_calibrated;
    slave_board_data.log_data[6] = global_data_A36926.analog_input_lambda_vmon.reading_scaled_and_calibrated;
    slave_board_data.log_data[7] = global_data_A36926.eoc_not_reached_count;
  
    
    if (global_data_A36926.control_state == STATE_POWER_UP) {
      global_data_A36926.power_up_delay_counter++;
      if (global_data_A36926.power_up_delay_counter >= POWER_UP_DELAY) {
	global_data_A36926.power_up_delay_counter = POWER_UP_DELAY;
      }
    }

    if (global_data_A36926.control_state == STATE_FAULT_WAIT) {
      global_data_A36926.fault_wait_time++;
      if (global_data_A36926.fault_wait_time >= TIME_WAIT_FOR_LAMBDA_TO_SET_FAULT_OUTPUTS) {
	global_data_A36926.fault_wait_time = TIME_WAIT_FOR_LAMBDA_TO_SET_FAULT_OUTPUTS;
      }
    }

    // Do Math on the ADC inputs
    ETMAnalogScaleCalibrateADCReading(&global_data_A36926.analog_input_lambda_vmon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36926.analog_input_lambda_vpeak);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36926.analog_input_lambda_imon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36926.analog_input_lambda_heat_sink_temp);
    
    
    if (global_data_A36926.control_state != STATE_OPERATE) {
      // Update the HV Lambda Program Values
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36926.analog_output_high_energy_vprog);
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36926.analog_output_low_energy_vprog);
      WriteLTC265XTwoChannels(&U1_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36926.analog_output_high_energy_vprog.dac_setting_scaled_and_calibrated,
			      LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36926.analog_output_low_energy_vprog.dac_setting_scaled_and_calibrated);
      
      
    } else {
      global_data_A36926.no_pulse_counter++;
      if (global_data_A36926.no_pulse_counter >= HV_ON_LAMBDA_SET_POINT_REFRESH_RATE_WHEN_NOT_PULSING) {
	// A long time has passed without updating the Lambda Set points
	// Update the HV Lambda Program Values
	global_data_A36926.no_pulse_counter = 0;
	
	ETMAnalogScaleCalibrateDACSetting(&global_data_A36926.analog_output_high_energy_vprog);
	ETMAnalogScaleCalibrateDACSetting(&global_data_A36926.analog_output_low_energy_vprog);
	WriteLTC265XTwoChannels(&U1_LTC2654,
				LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36926.analog_output_high_energy_vprog.dac_setting_scaled_and_calibrated,
				LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36926.analog_output_low_energy_vprog.dac_setting_scaled_and_calibrated);
      } 
    }
    
  UpdateFaultsAndStatusBits();
  }
}



void UpdateFaultsAndStatusBits(void) {

  // Update the digital input status pins
  if (PIN_LAMBDA_EOC == ILL_LAMBDA_AT_EOC) {
    _STATUS_LAMBDA_AT_EOC = 1;
  } else {
    _STATUS_LAMBDA_AT_EOC = 0;
  }
  
  if (ETMCanSlaveGetComFaultStatus()) {
    _FAULT_CAN_COMMUNICATION_LATCHED = 1;
  }

  // Update the logged and not logged status bits
  ETMDigitalUpdateInput(&global_data_A36926.digital_hv_not_powered, PIN_LAMBDA_NOT_POWERED);
  if (ETMDigitalFilteredOutput(&global_data_A36926.digital_hv_not_powered) == ILL_LAMBDA_NOT_POWERED) {
    _LOGGED_LAMBDA_NOT_POWERED = 1;
  } else {
    _LOGGED_LAMBDA_NOT_POWERED = 0;
  }

  ETMDigitalUpdateInput(&global_data_A36926.digital_sum_flt, PIN_LAMBDA_SUM_FLT);
  if (ETMDigitalFilteredOutput(&global_data_A36926.digital_sum_flt) == ILL_LAMBDA_FAULT_ACTIVE) {
    _NOT_LOGGED_LAMBDA_SUM_FAULT = 1;
  } else {
    _NOT_LOGGED_LAMBDA_SUM_FAULT = 0;
  }
    
    
  ETMDigitalUpdateInput(&global_data_A36926.digital_hv_off,  PIN_LAMBDA_HV_ON_READBACK);
  if (ETMDigitalFilteredOutput(&global_data_A36926.digital_hv_off) != ILL_LAMBDA_HV_ON) {
    _LOGGED_LAMBDA_READBACK_HV_OFF = 1;
  } else {
    _LOGGED_LAMBDA_READBACK_HV_OFF = 0;
  }

#ifdef __LCS1202
  ETMDigitalUpdateInput(&global_data_A36926.digital_phase_loss,  PIN_LAMBDA_PHASE_LOSS_FLT);
  if (ETMDigitalFilteredOutput(&global_data_A36926.digital_phase_loss) == ILL_LAMBDA_FAULT_ACTIVE) {
    _LOGGED_LAMBDA_PHASE_LOSS = 1;
  } else {
    _LOGGED_LAMBDA_PHASE_LOSS = 0;
  }
#else
  _LOGGED_LAMBDA_PHASE_LOSS = 0;
#endif      

  ETMDigitalUpdateInput(&global_data_A36926.digital_over_temp,  PIN_LAMBDA_OVER_TEMP_FLT);
  if (ETMDigitalFilteredOutput(&global_data_A36926.digital_over_temp) == ILL_LAMBDA_FAULT_ACTIVE) {
    _LOGGED_LAMBDA_OVER_TEMP = 1;
  } else {
    _LOGGED_LAMBDA_OVER_TEMP = 0;
  }
    
  ETMDigitalUpdateInput(&global_data_A36926.digital_interlock,  PIN_LAMBDA_INTERLOCK_FLT);
  if (ETMDigitalFilteredOutput(&global_data_A36926.digital_interlock) == ILL_LAMBDA_FAULT_ACTIVE) {
    _LOGGED_LAMBDA_INTERLOCK = 1;
  } else {
    _LOGGED_LAMBDA_INTERLOCK = 0;
  }
  
  ETMDigitalUpdateInput(&global_data_A36926.digital_load_flt,  PIN_LAMBDA_LOAD_FLT);
  if (ETMDigitalFilteredOutput(&global_data_A36926.digital_load_flt) == ILL_LAMBDA_FAULT_ACTIVE) {
    _LOGGED_LAMBDA_LOAD_FLT = 1;
  } else {
    _LOGGED_LAMBDA_LOAD_FLT = 0;
  }

  
  // Update fault bits if the lambda should be running
  if ((global_data_A36926.control_state == STATE_OPERATE) || (global_data_A36926.control_state == STATE_FAULT_WAIT)) {
    if (_NOT_LOGGED_LAMBDA_SUM_FAULT) {
      _FAULT_LAMBDA_SUM_FAULT = 1;
    }

    if (_LOGGED_LAMBDA_NOT_POWERED) {
      _FAULT_LAMBDA_NOT_POWERED = 1;
    }
  }
    
  // Look for too Many False Triggers
#define FALSE_TRIGGER_DECREMENT_TIME   100  // 1 Second
#define FALSE_TRIGGER_TRIP_POINT       50
  global_data_A36926.false_trigger_timer++;
  if (global_data_A36926.false_trigger_timer >= FALSE_TRIGGER_DECREMENT_TIME) {
    if (global_data_A36926.false_trigger_counter) {
      global_data_A36926.false_trigger_counter--;
    }
  }
  
  if (global_data_A36926.false_trigger_counter >= FALSE_TRIGGER_TRIP_POINT) {
    //_FAULT_FALSE_TRIGGER = 1;
    // DPARKER - Fix the false trigger detection
  } 
}
    



void InitializeA36926(void) {
  unsigned int startup_counter;

  // Initialize the status register and load the inhibit and fault masks
  _CONTROL_REGISTER = 0;
  _FAULT_REGISTER = 0;
  _WARNING_REGISTER = 0;
  _NOT_LOGGED_REGISTER = 0;



  // Configure Inhibit Interrupt
  _INT3IP = 7; // This must be the highest priority interrupt
  _INT3EP = 0; // Positive Transition
  
  
  // Initialize all I/O Registers
  TRISA = A36926_TRISA_VALUE;
  TRISB = A36926_TRISB_VALUE;
  TRISC = A36926_TRISC_VALUE;
  TRISD = A36926_TRISD_VALUE;
  TRISF = A36926_TRISF_VALUE;
  TRISG = A36926_TRISG_VALUE;


  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)


  // Configure T1 Inetrrupt
  _T1IP   = 5;


  // Initialize TMR1
  TMR1  = 0;
  _T1IF = 0;
  T1CON = T1CON_VALUE;



  // Initialize TMR2
  // TMR2 is unused but it must be configured for 16 bit mode
  T2CON = 0;

  // Initialize TMR3
  PR3   = PR3_VALUE_10_MILLISECONDS;
  TMR3  = 0;
  _T3IF = 0;
  T3CON = T3CON_VALUE;

  // Initialize the External EEprom
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);

  // Initialize the Can module
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_HV_LAMBDA_BOARD, _PIN_RG13, 4, _PIN_RA7, _PIN_RG12);
  ETMCanSlaveLoadConfiguration(36926, 0, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV);


  // Initialize LTC DAC
  SetupLTC265X(&U1_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);


  // Initialize Digital Input Filters
  ETMDigitalInitializeInput(&global_data_A36926.digital_hv_not_powered , ILL_LAMBDA_NOT_POWERED, 30);
  ETMDigitalInitializeInput(&global_data_A36926.digital_sum_flt        , !ILL_LAMBDA_FAULT_ACTIVE, 30);
  ETMDigitalInitializeInput(&global_data_A36926.digital_hv_off         , ILL_LAMBDA_HV_ON, 30);
  ETMDigitalInitializeInput(&global_data_A36926.digital_phase_loss     , !ILL_LAMBDA_FAULT_ACTIVE, 30);
  ETMDigitalInitializeInput(&global_data_A36926.digital_over_temp      , !ILL_LAMBDA_FAULT_ACTIVE, 30);
  ETMDigitalInitializeInput(&global_data_A36926.digital_interlock      , !ILL_LAMBDA_FAULT_ACTIVE, 30);
  ETMDigitalInitializeInput(&global_data_A36926.digital_load_flt       , !ILL_LAMBDA_FAULT_ACTIVE, 30);


  // Initialize the Analog input data structures
  ETMAnalogInitializeInput(&global_data_A36926.analog_input_lambda_vmon,
    MACRO_DEC_TO_SCALE_FACTOR_16(VMON_SCALE_FACTOR),
    OFFSET_ZERO,
    ANALOG_INPUT_1,
    NO_OVER_TRIP,
    NO_UNDER_TRIP,
    NO_TRIP_SCALE,
    NO_FLOOR,
    NO_COUNTER,
    NO_COUNTER
    );
  
  ETMAnalogInitializeInput(&global_data_A36926.analog_input_lambda_vpeak,
    MACRO_DEC_TO_SCALE_FACTOR_16(VMON_SCALE_FACTOR),
    OFFSET_ZERO,
    ANALOG_INPUT_2,
    NO_OVER_TRIP,
    NO_UNDER_TRIP,
    NO_TRIP_SCALE,
    NO_FLOOR,
    NO_COUNTER,
    NO_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36926.analog_input_lambda_imon,
    MACRO_DEC_TO_SCALE_FACTOR_16(.40179),
    OFFSET_ZERO,
    ANALOG_INPUT_3,
    NO_OVER_TRIP,
    NO_UNDER_TRIP,
    NO_TRIP_SCALE,
    NO_FLOOR,
    NO_COUNTER,
    NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36926.analog_input_lambda_heat_sink_temp,
    MACRO_DEC_TO_SCALE_FACTOR_16(.78125),
    10000,
    ANALOG_INPUT_4,
    LAMBDA_HEATSINK_OVER_TEMP,
    NO_UNDER_TRIP,
    NO_TRIP_SCALE,
    NO_FLOOR,
    TRIP_COUNTER_1Sec,
    TRIP_COUNTER_1Sec);
  
  ETMAnalogInitializeInput(&global_data_A36926.analog_input_5v_mon,
    MACRO_DEC_TO_SCALE_FACTOR_16(.12500),
    OFFSET_ZERO,
    ANALOG_INPUT_5,
    PWR_5V_OVER_FLT,
    PWR_5V_UNDER_FLT,
    NO_TRIP_SCALE,
    NO_FLOOR,
    NO_COUNTER,
    NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36926.analog_input_15v_mon,
    MACRO_DEC_TO_SCALE_FACTOR_16(.25063),
    OFFSET_ZERO,
    ANALOG_INPUT_6,
    PWR_15V_OVER_FLT,
    PWR_15V_UNDER_FLT,
    NO_TRIP_SCALE,
    NO_FLOOR,
    NO_COUNTER,
    NO_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36926.analog_input_neg_15v_mon,
    MACRO_DEC_TO_SCALE_FACTOR_16(.06250),
    OFFSET_ZERO,
    ANALOG_INPUT_7,
    PWR_NEG_15V_OVER_FLT,
    PWR_NEG_15V_UNDER_FLT,
    NO_TRIP_SCALE,
    NO_FLOOR,
    NO_COUNTER,
    NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36926.analog_input_pic_adc_test_dac,
    MACRO_DEC_TO_SCALE_FACTOR_16(1),
    OFFSET_ZERO,
    ANALOG_INPUT_8,
    ADC_DAC_TEST_OVER_FLT,
    ADC_DAC_TEST_UNDER_FLT,
    NO_TRIP_SCALE,
    NO_FLOOR,
    NO_COUNTER,
    NO_COUNTER);


    // Initialize the Analog Output Data Structures
  ETMAnalogInitializeOutput(&global_data_A36926.analog_output_high_energy_vprog,
    MACRO_DEC_TO_SCALE_FACTOR_16(VPROG_SCALE_FACTOR),
    OFFSET_ZERO,
    ANALOG_OUTPUT_2,
    HV_LAMBDA_MAX_VPROG,
    HV_LAMBDA_MIN_VPROG,
    HV_LAMBDA_DAC_ZERO_OUTPUT);
  
  ETMAnalogInitializeOutput(&global_data_A36926.analog_output_low_energy_vprog,
			    MACRO_DEC_TO_SCALE_FACTOR_16(VPROG_SCALE_FACTOR),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_3,
			    HV_LAMBDA_MAX_VPROG,
			    HV_LAMBDA_MIN_VPROG,
			    HV_LAMBDA_DAC_ZERO_OUTPUT);

  ETMAnalogInitializeOutput(&global_data_A36926.analog_output_spare,
			    MACRO_DEC_TO_SCALE_FACTOR_16(5.33333),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_0,
			    10000,
			    0,
			    0);

  ETMAnalogInitializeOutput(&global_data_A36926.analog_output_adc_test,
			    MACRO_DEC_TO_SCALE_FACTOR_16(1),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_NO_CALIBRATION,
			    0xFFFF,
			    0,
			    0);


  ETMAnalogSetOutput(&global_data_A36926.analog_output_spare, 3000);
  ETMAnalogSetOutput(&global_data_A36926.analog_output_adc_test, ADC_DAC_TEST_VALUE);

  global_data_A36926.analog_output_spare.enabled      = 1;
  global_data_A36926.analog_output_adc_test.enabled   = 1;

  ETMAnalogScaleCalibrateDACSetting(&global_data_A36926.analog_output_spare);
  ETMAnalogScaleCalibrateDACSetting(&global_data_A36926.analog_output_adc_test);

  // Update the spare analog output and the DAC test output
  WriteLTC265XTwoChannels(&U1_LTC2654,
			  LTC265X_WRITE_AND_UPDATE_DAC_A,
			  global_data_A36926.analog_output_spare.dac_setting_scaled_and_calibrated,
			  LTC265X_WRITE_AND_UPDATE_DAC_B,
			  global_data_A36926.analog_output_adc_test.dac_setting_scaled_and_calibrated);
  

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


  // Flash LEDs at Startup
  startup_counter = 0;
  while (startup_counter <= 400) {  // 4 Seconds total
    ETMCanSlaveDoCan();
    if (_T3IF) {
      _T3IF =0;
      startup_counter++;
    } 
    switch (((startup_counter >> 4) & 0b11)) {
      
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
  

  PIN_LAMBDA_VOLTAGE_SELECT = OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY;  
}




void EnableHVLambda(void) {
  // Set the enable register in the DAC structure
  // The next time the DAC is updated it will be updated with the most recent high/low energy values
  global_data_A36926.analog_output_high_energy_vprog.enabled = 1;
  global_data_A36926.analog_output_low_energy_vprog.enabled = 1;
  
  // Set digital output to inhibit the lambda
  PIN_LAMBDA_INHIBIT = !OLL_INHIBIT_LAMBDA;
  
  // Set digital output to enable HV_ON of the lambda
  PIN_LAMBDA_ENABLE = OLL_ENABLE_LAMBDA;

  T1CONbits.TON = 0;
  PR1 = (TMR1_LAMBDA_CHARGE_PERIOD - TMR1_DELAY_HOLDOFF);
  TMR1 = 0;
  _T1IF = 0;
  _T1IE = 1;
  T1CONbits.TON = 1;
  
  _INT3IF = 0;
  _INT3IE = 1;  
}




void DisableHVLambda(void) {
  // Clear the enable register in the DAC structure
  // The next time the DAC is updated it will be updated with the "disabled" value
  global_data_A36926.analog_output_high_energy_vprog.enabled = 0;
  global_data_A36926.analog_output_low_energy_vprog.enabled = 0;
  
  _INT3IE = 0; 

  // Set digital output to inhibit the lambda
  PIN_LAMBDA_INHIBIT = OLL_INHIBIT_LAMBDA;
  
  // Set digital output to disable HV_ON of the lambda
  PIN_LAMBDA_ENABLE = !OLL_ENABLE_LAMBDA;
}






void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;
  
  if (global_data_A36926.adc_ignore_current_sample) {
    // There was a pulse durring the sample sequence.  Throw the data away!!!
    global_data_A36926.adc_ignore_current_sample = 0;
  } else {
    // Copy Data From Buffer to RAM
    if (_BUFS) {
      // read ADCBUF 0-7
      global_data_A36926.analog_input_lambda_vmon.adc_accumulator           += ADCBUF0;
      global_data_A36926.analog_input_lambda_vpeak.adc_accumulator          += ADCBUF1;
      global_data_A36926.analog_input_lambda_imon.adc_accumulator           += ADCBUF2;
      global_data_A36926.analog_input_lambda_heat_sink_temp.adc_accumulator += ADCBUF3;
      
      global_data_A36926.analog_input_15v_mon.adc_accumulator               += ADCBUF4;
      global_data_A36926.analog_input_neg_15v_mon.adc_accumulator           += ADCBUF5;
      global_data_A36926.analog_input_5v_mon.adc_accumulator                += ADCBUF6;
      global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator      += ADCBUF7;

      global_data_A36926.accumulator_counter++ ;
      
      if (global_data_A36926.store_lambda_voltage) {
	global_data_A36926.vmon_store_1 = ADCBUF0;
	// DPARKER global_data_A36926.vmon_store_2 = ADCBUF1;
	global_data_A36926.store_lambda_voltage = 0;
      }
    } else {
      // read ADCBUF 8-15
      global_data_A36926.analog_input_lambda_vmon.adc_accumulator           += ADCBUF8;
      global_data_A36926.analog_input_lambda_vpeak.adc_accumulator          += ADCBUF9;
      global_data_A36926.analog_input_lambda_imon.adc_accumulator           += ADCBUFA;
      global_data_A36926.analog_input_lambda_heat_sink_temp.adc_accumulator += ADCBUFB;
      
      global_data_A36926.analog_input_15v_mon.adc_accumulator               += ADCBUFC;
      global_data_A36926.analog_input_neg_15v_mon.adc_accumulator           += ADCBUFD;
      global_data_A36926.analog_input_5v_mon.adc_accumulator                += ADCBUFE;
      global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator      += ADCBUFF;

      global_data_A36926.accumulator_counter++ ;

      if (global_data_A36926.store_lambda_voltage) {
	global_data_A36926.vmon_store_1 = ADCBUF8;
	// DPARKER global_data_A36926.vmon_store_2 = ADCBUF9;
	global_data_A36926.store_lambda_voltage = 0;
      }
    }

    
    if (global_data_A36926.accumulator_counter >= 128) {

      global_data_A36926.analog_input_lambda_vmon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36926.analog_input_lambda_vmon.filtered_adc_reading = global_data_A36926.analog_input_lambda_vmon.adc_accumulator;
      global_data_A36926.analog_input_lambda_vmon.adc_accumulator = 0;
      
      global_data_A36926.analog_input_lambda_vpeak.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36926.analog_input_lambda_vpeak.filtered_adc_reading = global_data_A36926.analog_input_lambda_vpeak.adc_accumulator;
      global_data_A36926.analog_input_lambda_vpeak.adc_accumulator = 0;
      
      global_data_A36926.analog_input_lambda_imon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36926.analog_input_lambda_imon.filtered_adc_reading = global_data_A36926.analog_input_lambda_imon.adc_accumulator;
      global_data_A36926.analog_input_lambda_imon.adc_accumulator = 0;


      global_data_A36926.analog_input_lambda_heat_sink_temp.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36926.analog_input_lambda_heat_sink_temp.filtered_adc_reading = global_data_A36926.analog_input_lambda_heat_sink_temp.adc_accumulator;
      global_data_A36926.analog_input_lambda_heat_sink_temp.adc_accumulator = 0;
      
      global_data_A36926.analog_input_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36926.analog_input_15v_mon.filtered_adc_reading = global_data_A36926.analog_input_15v_mon.adc_accumulator;
      global_data_A36926.analog_input_15v_mon.adc_accumulator = 0;
      
      global_data_A36926.analog_input_neg_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36926.analog_input_neg_15v_mon.filtered_adc_reading = global_data_A36926.analog_input_neg_15v_mon.adc_accumulator;
      global_data_A36926.analog_input_neg_15v_mon.adc_accumulator = 0;            

      global_data_A36926.analog_input_5v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36926.analog_input_5v_mon.filtered_adc_reading = global_data_A36926.analog_input_5v_mon.adc_accumulator;
      global_data_A36926.analog_input_5v_mon.adc_accumulator = 0;

      global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36926.analog_input_pic_adc_test_dac.filtered_adc_reading = global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator;
      global_data_A36926.analog_input_pic_adc_test_dac.adc_accumulator = 0;
      
      
      global_data_A36926.accumulator_counter = 0;
    }
  }
}


void __attribute__((interrupt, shadow, no_auto_psv)) _INT3Interrupt(void) {
  /*    
    Assuming a sucessful trigger, this interupt will do the following conecptual steps
    (1) Inhibit the HV supply a set amount of time.
    (2) After the inhibt time has passed, !Inhibit the power supply, and start the EOC timer
    (3) Set the status bit that indicates a pulse occured
  */ 
  
  PIN_LAMBDA_INHIBIT = OLL_INHIBIT_LAMBDA;
  
  if (_BUFS) {
    global_data_A36926.vmon_store_3 = ADCBUF0;
    global_data_A36926.vmon_store_2 = ADCBUF3;
  } else {
    global_data_A36926.vmon_store_3 = ADCBUF8;
    global_data_A36926.vmon_store_2 = ADCBUFB;
  }

  // Setup timer1 to time the inhibit period
  T1CONbits.TON = 0;
  TMR1 = 0;
  PR1 = TMR1_DELAY_HOLDOFF;
  T1CONbits.TON = 1;
  _T1IF = 0;
  
  if (global_data_A36926.run_post_pulse_process) {
    // We never completed the post pulse process.  
    global_data_A36926.post_pulse_did_not_run_counter++;
  }
  
  global_data_A36926.previous_pulse_id = global_data_A36926.pulse_id;
  global_data_A36926.pulse_id = ETMCanSlaveGetPulseCount();
  if (global_data_A36926.previous_pulse_id == global_data_A36926.pulse_id) {
    // We did not receive a message from the PULSE SYNC board between pulses
    // There should not have been a trigger
    global_data_A36926.false_trigger_total_count++;
    global_data_A36926.false_trigger_counter++;
  }

  if (_T1IE) {
    // If timer one is enabled then we did not complete the Charge period before the next pulse
    global_data_A36926.charge_period_error_counter++;
  }
  
  _T1IE = 0;
  
  while(!_T1IF);                                                   // what for the holdoff time to pass

  // We need to guarantee that we start charging to the low energy mode voltage until we get a command to charge to high energ
  ETMCanSlaveSetPulseLevelLow();
  PIN_LAMBDA_VOLTAGE_SELECT = OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY;
  
  PIN_LAMBDA_INHIBIT = !OLL_INHIBIT_LAMBDA;
  
  // Set up Timer1 to produce interupt at end of charge period
  T1CONbits.TON = 0;
  TMR1 = 0;
  _T1IF = 0;
  _T1IE = 1;
  PR1 = (TMR1_LAMBDA_CHARGE_PERIOD - TMR1_DELAY_HOLDOFF);
  T1CONbits.TON = 1;
  
  
  
  
  global_data_A36926.run_post_pulse_process = 1;     // This tells the main control loop that a pulse has occured and that it should run the post pulse process once (and only once) 
  global_data_A36926.adc_ignore_current_sample = 1;  // This allows the internal ADC ISR to know that there was a pulse and to discard all the data from the sequence where the pulse occured
  
  global_data_A36926.pulse_counter++;
  
  _INT3IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
  /*
    This interrupt indicates that the cap charger should have finished charging and it is time to enable the trigger pulse.

    This interrupt is called X us after charging starts
    If the lambda is not at EOC, it does not enable the trigger and sets the Lambda EOC Timeout Fault bit
    If the lambda is at EOC, It enables the trigger & sets status bits to show that the lambda is not charging and that the system is ready to fire.    
  */
  
  _T1IF = 0;         // Clear the interrupt flag
  _T1IE = 0;         // Disable the interrupt (This will be enabled the next time that a capacitor charging sequence starts)
  T1CONbits.TON = 0;   // Stop the timer from incrementing (Again this will be restarted with the next time the capacitor charge sequence starts)
  global_data_A36926.store_lambda_voltage = 1;

  if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
    __delay32(DELAY_TCY_5US);
    if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
      __delay32(DELAY_TCY_5US);
      if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
	__delay32(DELAY_TCY_5US);
	if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
	  __delay32(DELAY_TCY_5US);
	  if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
	    global_data_A36926.eoc_not_reached_count++;
	  }
	} 
      }
    }
  }
}  


void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;

  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
    case ETM_CAN_REGISTER_HV_LAMBDA_SET_1_LAMBDA_SET_POINT:
      ETMAnalogSetOutput(&global_data_A36926.analog_output_high_energy_vprog, message_ptr->word1); 
      ETMAnalogSetOutput(&global_data_A36926.analog_output_low_energy_vprog,message_ptr->word2);
      _CONTROL_NOT_CONFIGURED = 0;
      break;
      
    default:
      //local_can_errors.invalid_index++;
      break;
    }
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}




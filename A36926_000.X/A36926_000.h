#ifndef __A36926_000_H
#define __A36926_000_H

#include <xc.h>
#include <adc12.h>
#include <timer.h>
#include <libpic30.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"



#define FCY_CLK     10000000
#define FCY_CLK_MHZ 10



// DPARKER move timer 5 to timer 3

/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN 
  Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by LTC265X Module
  I2C    - Used/Configured by EEPROM Module


  Timer1 - Used to time to Lambda Charge Time and the Lambda Inhibit Time and the startup timer
  Timer3 - Used for 10msTicToc

  ADC Module - See Below For Specifics

*/



// ---------- BASE A36926 I/O CONFIGURATION ----------------- //
 
#define PIC_DIG_IN_1      _RD8
#define PIC_DIG_IN_2      _RD9
#define PIC_DIG_IN_3      _RD10
#define PIC_DIG_IN_4      _RD11
#define PIC_DIG_IN_5      _RD12
#define PIC_DIG_IN_6      _RD13
#define PIC_DIG_IN_7      _RD14
#define PIC_DIG_IN_8      _RD15

#define AUTO_INHIBIT_DETECT          _RA14   // This is INT3
#define RESET_DETECT                 _RG14



#define PIC_RELAY_OUT                _LATD3
#define PIC_OUTPUT_LAMBDA_SELECT     _LATD2
#define PIC_DIGITAL_OUT_2_NOT        _LATD1
#define PIC_DIGITAL_OUT_1_NOT        _LATD0
#define PIC_15V_SUPPLY_ENABLE        _LATA6

#define TEST_POINT_A                 _LATF6
#define TEST_POINT_B                 _LATF7
#define TEST_POINT_C                 _LATF8
#define TEST_POINT_D                 _LATF2
#define TEST_POINT_E                 _LATF3
#define TEST_POINT_F                 _LATB14


// These are all managed by the CAN Module
//#define LED_OPERATIONAL              _LATA7
//#define LED_A_RED                    _LATG12
//#define LED_B_GREEN                  _LATG13



/*
  BASE ANALOG CONFIGURATION
  PIC_ADC_AN1  is on AN2
  PIC_ADC_AN2  is on AN3
  PIC_ADC_AN3  is on AN4
  PIC_ADC_AN4  is on AN5
  
  PIC_ADC_+15V_MON is on AN6
  PIC_ADC_-15V_MON is on AN7
  PIC_ADC_5V_MON   is on AN8
  PIC_ADC_TEST_DAC is on AN9
*/


// Pins that must be configured as outputs
/*
  A6,A7
  B14
  C
  D0,D1,D2,D3
  F2,F3,F6,F7,F8
  G12,G13
*/

#define A36926_TRISA_VALUE 0b1111111110111111
#define A36926_TRISB_VALUE 0b1011111111111111
#define A36926_TRISC_VALUE 0b1111111111111111
#define A36926_TRISD_VALUE 0b1111111111110000
#define A36926_TRISF_VALUE 0b1111111100110011
#define A36926_TRISG_VALUE 0b1100111111111111



// ----------------------- CONFIGURE PINS FROM ALAL A36926-000 Code ----------------- //


// -------- Digital Input Pins ----------//
//#define PIN_FIBER_TRIGGER_IN                  _RA12
//#define PIN_FIBER_ENERGY_SELECT               _RA13
#define PIN_AUTO_INHIBIT                      AUTO_INHIBIT_DETECT

#define PIN_LAMBDA_EOC                        PIC_DIG_IN_4
#define PIN_LAMBDA_HV_ON_READBACK             PIC_DIG_IN_8
#define PIN_LAMBDA_NOT_POWERED                PIC_DIG_IN_1
#define PIN_LAMBDA_SUM_FLT                    PIC_DIG_IN_6
#define PIN_LAMBDA_PHASE_LOSS_FLT             PIC_DIG_IN_7
#define PIN_LAMBDA_OVER_TEMP_FLT              PIC_DIG_IN_2
#define PIN_LAMBDA_INTERLOCK_FLT              PIC_DIG_IN_3
#define PIN_LAMBDA_LOAD_FLT                   PIC_DIG_IN_5

#define PIN_RESET_DETECT                      RESET_DETECT


#define ILL_HIGH_ENERGY_SELECTED              1
#define ILL_LAMBDA_AT_EOC                     1
#define ILL_LAMBDA_NOT_POWERED                0
#define ILL_LAMBDA_HV_ON                      1
#define ILL_LAMBDA_FAULT_ACTIVE               1




// ------- Digital Output Pins ---------//

#define PIN_LAMBDA_VOLTAGE_SELECT             PIC_OUTPUT_LAMBDA_SELECT
#define OLL_LAMBDA_VOLTAGE_SELECT_LOW_ENERGY  0

#define PIN_LAMBDA_INHIBIT                    PIC_DIGITAL_OUT_1_NOT
#define PIN_LAMBDA_ENABLE                     PIC_DIGITAL_OUT_2_NOT

#define PIN_LED_OPERATIONAL_GREEN             LED_OPERATIONAL
#define PIN_LED_A_RED                         LED_A_RED
#define PIN_LED_B_GREEN                       LED_B_GREEN  // This is is configured by the CAN module to flash on CAN Bus activity

#define PIN_OUT_TP_A                          TEST_POINT_A
#define PIN_OUT_TP_B                          TEST_POINT_B
#define PIN_OUT_TP_C                          TEST_POINT_C
#define PIN_OUT_TP_D                          TEST_POINT_D
#define PIN_OUT_TP_E                          TEST_POINT_E
#define PIN_OUT_TP_F                          TEST_POINT_F

#define OLL_LED_ON                            0
#define OLL_SELECT_HIGH_ENERGY_MODE           1
#define OLL_INHIBIT_LAMBDA                    0
#define OLL_ENABLE_LAMBDA                     0










// ------------------------ CONFIGURE ADC MODULE ------------------- //

// ----------------- ANALOG INPUT PINS ---------------- //
/* 
   AN2 - Lambda Vmon
   AN3 - Lambda VPeak
   AN4 - Lambda Imon
   AN5 - Lambda Temp
   AN6 - +15V Mon
   AN7 - -15V Mon   
   AN8 - 5V Mon
   AN9 - ADC Test Input
   

   
*/

/*
  This sets up the ADC to work as following
  AUTO Sampeling
  External Vref+/Vref-
  With 10MHz System Clock, ADC Clock is 450ns (4.5 clocks per ADC clock), Sample Time is 6 ADC Clock so total sample time is 9.0uS
  Conversion rate of 111KHz (13.888 Khz per Channel), 138 Samples per 10mS interrupt
  8 Samples per Interrupt, use alternating buffers
  Scan Through Selected Inputs
*/

#define ADCON1_SETTING  (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING  (ADC_VREF_EXT_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCHS_SETTING   (ADC_CH0_POS_SAMPLEA_AN2 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN2 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING  (ENABLE_AN2_ANA & ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA & ENABLE_AN8_ANA & ENABLE_AN9_ANA)
#define ADCSSL_SETTING  (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 & SKIP_SCAN_AN15)
#define ADCON3_SETTING  (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)

/* 
   ---------- TMR1 Configuration -----------
   Timer1 - Used to time to Lambda Charge Time and the Lambda Inhibit Time
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/
    
#define T1CON_VALUE                    (T1_OFF & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_8 & T1_SOURCE_INT)
#define TMR1_DELAY_HOLDOFF_US          LAMBDA_HOLDOFF_TIME_US
#define TMR1_LAMBDA_CHARGE_TIME_US     LAMBDA_MAX_CHARGE_TIME_US
#define TMR1_DELAY_HOLDOFF             (FCY_CLK_MHZ*TMR1_DELAY_HOLDOFF_US/8)    
#define TMR1_LAMBDA_CHARGE_PERIOD      (FCY_CLK_MHZ*TMR1_LAMBDA_CHARGE_TIME_US/8)
#define TMR1_RETRIGGER_BLANK           (FCY_CLK_MHZ*RETRIGGER_BLANKING_US/8)



/* 
   TMR5 Configuration
   Timer5 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T3CON_VALUE                    (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_8 & T3_SOURCE_INT)
#define PR3_VALUE_10_MILLISECONDS      12500


typedef struct {
  AnalogInput analog_input_lambda_vmon;               // 1V per LSB
  AnalogInput analog_input_lambda_vpeak;              // 1V per LSB
  AnalogInput analog_input_lambda_imon;               // 100uA per LSB
  AnalogInput analog_input_lambda_heat_sink_temp;     // 1 mili Deg C per LSB

  AnalogInput analog_input_5v_mon;                    // 1mV per LSB
  AnalogInput analog_input_15v_mon;                   // 1mV per LSB
  AnalogInput analog_input_neg_15v_mon;               // 1mV per LSB
  AnalogInput analog_input_pic_adc_test_dac;          // 62.5uV per LSB


  AnalogOutput analog_output_high_energy_vprog;       // 1V per LSB
  AnalogOutput analog_output_low_energy_vprog;        // 1V per LSB

  AnalogOutput analog_output_spare;                   // 1mV per LSB
  AnalogOutput analog_output_adc_test;                // 62.5uV per LSB

  unsigned int accumulator_counter;
  unsigned int adc_ignore_current_sample;
  unsigned int eoc_not_reached_count;
  unsigned int control_state;
  unsigned int led_divider;
  unsigned int run_post_pulse_process;
  unsigned int no_pulse_counter;
  unsigned int pulse_counter;
  unsigned int post_pulse_did_not_run_counter;
  unsigned int charge_period_error_counter;
  unsigned int power_up_timer;
  //unsigned int fault_active;
  unsigned int power_up_delay_counter;
  unsigned int fault_wait_time;
  unsigned int pulse_id;
  unsigned int previous_pulse_id;
  unsigned int false_trigger_total_count;
  unsigned int false_trigger_counter;
  unsigned int false_trigger_timer;

  TYPE_DIGITAL_INPUT digital_sum_flt;
  TYPE_DIGITAL_INPUT digital_hv_off;
  TYPE_DIGITAL_INPUT digital_phase_loss;
  TYPE_DIGITAL_INPUT digital_hv_not_powered;
  TYPE_DIGITAL_INPUT digital_over_temp;
  TYPE_DIGITAL_INPUT digital_interlock;
  TYPE_DIGITAL_INPUT digital_load_flt;


  //unsigned int sum_flt_counter;
  //unsigned int hv_off_counter;
  //unsigned int phase_loss_counter;
  //unsigned int lambda_not_powered_counter;

  unsigned int vmon_store_1;
  unsigned int vmon_store_2;
  unsigned int vmon_store_3;
  unsigned int store_lambda_voltage;
} LambdaControlData;

extern LambdaControlData global_data_A36926;


// State Definitions
#define STATE_STARTUP                10
#define STATE_WAITING_FOR_CONFIG     20
#define STATE_WAITING_FOR_POWER      30
#define STATE_POWER_UP               40
#define STATE_OPERATE                50
#define STATE_FAULT_WAIT             55
#define STATE_FAULT                  60



#define DELAY_TCY_5US                FCY_CLK_MHZ*5


#define _LOGGED_LAMBDA_NOT_POWERED                      _WARNING_0
#define _LOGGED_LAMBDA_READBACK_HV_OFF                  _WARNING_1
#define _LOGGED_LAMBDA_PHASE_LOSS                       _WARNING_2
#define _LOGGED_LAMBDA_OVER_TEMP                        _WARNING_3
#define _LOGGED_LAMBDA_INTERLOCK                        _WARNING_4
#define _LOGGED_LAMBDA_LOAD_FLT                         _WARNING_5


#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_0
#define _FAULT_LAMBDA_SUM_FAULT                         _FAULT_1
#define _FAULT_LAMBDA_NOT_POWERED                       _FAULT_2
#define _FAULT_POWER_UP_TIMEOUT                         _FAULT_3
#define _FAULT_FALSE_TRIGGER                            _FAULT_4

#define _NOT_LOGGED_LAMBDA_SUM_FAULT                    _NOT_LOGGED_0
#define _STATUS_LAMBDA_AT_EOC                           _NOT_LOGGED_1
#define _STATUS_LAMBDA_HIGH_ENERGY                      _NOT_LOGGED_2














// MOVE TO SETTINGS FILE






#ifdef __LCS1202
#define VPROG_SCALE_FACTOR 2.66667
#define VMON_SCALE_FACTOR  .31250
#else
#define VPROG_SCALE_FACTOR 2.96296
#define VMON_SCALE_FACTOR  .28125
#endif


// Settings for the HV Lambda 
#define HV_LAMBDA_MAX_VPROG            19000
#define HV_LAMBDA_MIN_VPROG            3000
#define HV_LAMBDA_DAC_ZERO_OUTPUT      0x0000    // This will set the DAC output to Zero (program of Zero as well)



#define LAMBDA_HEATSINK_OVER_TEMP      57000     // 57 Deg C
#define TRIP_COUNTER_100mS             10
#define TRIP_COUNTER_1Sec              100




#define LAMBDA_HOLDOFF_TIME_US         700        // 400 uS
#define LAMBDA_MAX_CHARGE_TIME_US      2400      // 2.4mS
#define RETRIGGER_BLANKING_US          1100       // 500us


#define HV_ON_LAMBDA_SET_POINT_REFRESH_RATE_WHEN_NOT_PULSING            200             // 2 seconds


#define POWER_UP_DELAY   500                            // 5 Seconds
#define TIME_WAIT_FOR_LAMBDA_TO_SET_FAULT_OUTPUTS   300  // 1 Seconds



#define PWR_5V_OVER_FLT        5200                   // 5.2 V
#define PWR_5V_UNDER_FLT       4800                   // 4.8 V

#define PWR_15V_OVER_FLT       15500                  // 15.5 V
#define PWR_15V_UNDER_FLT      14500                  // 14.5 V

#define PWR_NEG_15V_OVER_FLT   15500                  // -15.5 V
#define PWR_NEG_15V_UNDER_FLT  14500                  // -14.5 V

#define ADC_DAC_TEST_VALUE     0x8000                 // Default Test Value
#define ADC_DAC_TEST_OVER_FLT  0x8200                 // 1.01562 of test value 
#define ADC_DAC_TEST_UNDER_FLT 0x7E00                 // .984375 of test value






#endif

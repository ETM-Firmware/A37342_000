#ifndef __A37342_000_CONFIG_H
#define __A37342_000_CONFIG_H



// SELECT ONE POWER SUPPLY
//#define __HVPS_LC1202_20KV
//#define __HVPS_LC1202_20KV_ALLOW_PROGRAM_OVER_25KV
//#define __HVPS_LC1202_25KV
#define __HVPS_LC802_18KV

// SELECT ONE MAX REP-RATE
//#define __PRF_MAX_800
#define __PRF_MAX_400


#ifdef __HVPS_LC1202_20KV
#define VPROG_SCALE_FACTOR      2.66667
#define VMON_SCALE_FACTOR       .31250
#define HV_LAMBDA_MAX_VPROG     20000
#define SOFTWARE_DASH_NUMBER    495
#define __HVPS_LC1202
#endif


#ifdef __HVPS_LC1202_20KV_ALLOW_PROGRAM_OVER_25KV
#define VPROG_SCALE_FACTOR      2.66667
#define VMON_SCALE_FACTOR       .31250
#define HV_LAMBDA_MAX_VPROG     20500
#define SOFTWARE_DASH_NUMBER    100
#define __HVPS_LC1202
#endif


#ifdef __HVPS_LC1202_25KV
#define VPROG_SCALE_FACTOR      2.13125
#define VMON_SCALE_FACTOR       .3906
#define HV_LAMBDA_MAX_VPROG     22100
#define SOFTWARE_DASH_NUMBER    100
#define __HVPS_LC1202
#endif



#ifdef __HVPS_LC802_18KV
#define VPROG_SCALE_FACTOR      2.96296
#define VMON_SCALE_FACTOR       .28125
#define HV_LAMBDA_MAX_VPROG     18000
#define SOFTWARE_DASH_NUMBER    0
#define __HVPS_802
#endif


#ifdef __PRF_MAX_400
#define LAMBDA_HOLDOFF_TIME_US         150        // 150 uS
#define LAMBDA_MAX_CHARGE_TIME_US      2300       // 2.3mS
#endif

#ifdef __PRF_MAX_800
#define LAMBDA_HOLDOFF_TIME_US         150        // 150 uS
#define LAMBDA_MAX_CHARGE_TIME_US      1150       // 1.15mS
#endif


#define LAMBDA_STARTUP_CHARGE_TIME_US  50000      // 50ms


#endif

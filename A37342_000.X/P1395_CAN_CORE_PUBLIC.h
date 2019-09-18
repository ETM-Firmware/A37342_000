#ifndef __P1395_CAN_CORE_PUBLIC_H
#define __P1395_CAN_CORE_PUBLIC_H


//#define ETM_CAN_HIGH_ENERGY           1
//#define ETM_CAN_LOW_ENERGY            0


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


#define SYSTEM_CONFIGURATION_6_4_R                                      0xA64A
#define SYSTEM_CONFIGURATION_6_4_M                                      0xA64B
#define SYSTEM_CONFIGURATION_6_4_S                                      0xA64C
#define SYSTEM_CONFIGURATION_2_5_R                                      0xA25A






#endif









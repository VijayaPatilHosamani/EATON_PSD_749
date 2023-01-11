
/* ==============================================================================
System Name: DC-DC Converter

File Name:	 converter_settings.h

Target:      F28004x

Author:

Description:    define MACROS and variables for different configurations

=================================================================================  */
#ifndef CONVERTER_SETTING_H
#define CONVERTER_SETTING_H



//*****************************************************************************
//defines
//*****************************************************************************

/*-----------------Device Related Defines-----------------*/
// CPU system clock frequency = 100MHz
#define CPU_SYS_CLOCK      (100*1000000)
// PWM module clock frequency = 100MHz
#define PWMSYSCLOCK_FREQ   (100*1000000)

/*---------------System Interrupt settings----------------*/

#define C28x_HS_MONITORING_INTERRUPT_PIE_GROUP_NO INTERRUPT_ACK_GROUP3
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (modify HSloop interrupt setting, 2 lines) //
#define C28x_HS_MONITORING_INTERRUPT_TRIG_PWM_BASE EPWM6_BASE // BK Original: EPWM4_BASE
#define C28x_HS_MONITORING_INTERRUPT INT_EPWM6 // BK Original: INT_EPWM4
#define CTRL_HS_MONITORING_ISR_FREQUENCY 10000
#define C28x_CTRL_LS_MONITORING_INTERRUPT_TRIG_CPUTIMER_BASE CPUTIMER2_BASE
#define C28x_CTRL_LS_MONITORING_INTERRUPT INT_TIMER2


/*----------------------Control Loop Design---------------------*/
#define HS_MONITORING_ISR_FREQUENCY (DC_CONVERTER_PWM_SWITCHING_FREQUENCY/HS_MONITORING_ISR_FREQ_RATIO)
#define HS_MONITORING_ISR_FREQ_RATIO    1


/*----------------------PI Control Loop Tuning Parameters---------------------*/
// Proportional gain
//#define  PI_KP  (float)0.1
// Integral gain
//#define PI_KI  (float)0.0
// Upper control saturation limit
#define  PI_OUT_UP_SAT   (float)1.0
// Lower control saturation limit
#define   PI_OUT_LOW_SAT  (float)-1.0
// Upper Integrator saturation limit
#define PI_INTG_MAX  (float)1.0
// Lower Integrator saturation limit
#define  PI_INTG_MIN  (float)-1.0

/*---------------------Closed loop design settings-------------*/

// Enable(1)- for enabling user settings ( current command set point decides duty cycle)
// Disable(0) - for closed loop operation (current command set point is decided by Vout set point and Feedback
//#define ENABLE_USER_DEFINED_CURRENT 1

//#define DUTY_CYCLE_DEBUG 0         // Use a specific duty cycle set in converter.h


// set desired current command (range from 0.0A to MAX_SENSE_OUTPUT_CURRENT)

//#define CURRENT_COMMAND_SET_VALUE (float)20.0
//#define CURRENT_COMMAND_SET_VALUE_Slope (float) (0.4/70.0)
//#define CURRENT_COMMAND_SET_VALUE_INIT (float)  (10.0/70.0)
//#define CURRENT_COMMAND_SET_FACTOR (CURRENT_COMMAND_SET_VALUE * CURRENT_COMMAND_SET_VALUE_Slope + CURRENT_COMMAND_SET_VALUE_INIT)


// set desired output reference voltage (range from 0.0V to MAX_SENSE_OUTPUT_VOLTAGE)
#define  OUT_REF_VOLT       (float)24.0
#define  OUT_REF_VOLT_FACTOR  (OUT_REF_VOLT/ MAX_SENSE_OUTPUT_VOLTAGE)
#define AVG_CTRL_WINDOW_SIZE  10

/*----------------System Related Values---------------------*/
// PWM switching frequency = 100kHz
#define DC_CONVERTER_PWM_SWITCHING_FREQUENCY ((float)100*1000)
// PWM Triangle period = PWM module clock frequency / PWM switching frequency
#define DC_CONVERTER_PWM_PERIOD (PWMSYSCLOCK_FREQ)/(DC_CONVERTER_PWM_SWITCHING_FREQUENCY)


/*------------PWM pin, ADC Selection related variables---------------*/

// PWM base for DC converter module-- current command PWM. CMD1 uses PWM5, CMD2 uses PWM2, CMD3 uses PWM3
#define DC_CONVERTER_PWM_CMD1               EPWM5_BASE
#define DC_CONVERTER_PWM_CMD2               EPWM2_BASE
#define DC_CONVERTER_PWM_CMD3               EPWM3_BASE
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 2 lines) //
#define DC_CONVERTER_PWM_CMD4               EPWM1_BASE
#define DC_CONVERTER_PWM_CMD5               EPWM4_BASE
//Triangle generation for interrupt trigger source -- High Speed Monitoring ISR
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (modify HSloop setting from ePWM4 to ePWM6) //
#define HS_MONITORING_INTR_PWM_BASE         EPWM6_BASE // Origin: EPWM4_BASE


// Set GPIO16 as PWM5A
#define DC_CONVERTER_CMD1_H_GPIO                 16
#define DC_CONVERTER_CMD1_H_GPIO_PIN_CONFIG      GPIO_16_EPWM5A

// Set GPIO2 as PWM2A
#define DC_CONVERTER_CMD2_H_GPIO                 2
#define DC_CONVERTER_CMD2_H_GPIO_PIN_CONFIG      GPIO_2_EPWM2A

// Set GPIO4 as PWM3A
#define DC_CONVERTER_CMD3_H_GPIO                 4
#define DC_CONVERTER_CMD3_H_GPIO_PIN_CONFIG      GPIO_4_EPWM3A

// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 6 lines) //
// Set GPIO0 as PWM1A
#define DC_CONVERTER_CMD4_H_GPIO                 0
#define DC_CONVERTER_CMD4_H_GPIO_PIN_CONFIG      GPIO_0_EPWM1A

// Set GPIO6 as PWM4A
#define DC_CONVERTER_CMD5_H_GPIO                 6
#define DC_CONVERTER_CMD5_H_GPIO_PIN_CONFIG      GPIO_6_EPWM4A


// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (modify GPIO settings for OC fault) //
/*------------GPIO DEFINES for DC module Input Current Exceeded Fault signal (From Module)---------------*/
#define DC_Module1_OC_FAULT_IN_GPIO                40
#define DC_Module1_OC_FAULT_IN_GPIO_PIN_CONFIG     GPIO_40_GPIO40
#define DC_Module2_OC_FAULT_IN_GPIO                39
#define DC_Module2_OC_FAULT_IN_GPIO_PIN_CONFIG     GPIO_39_GPIO39
#define DC_Module3_OC_FAULT_IN_GPIO                34
#define DC_Module3_OC_FAULT_IN_GPIO_PIN_CONFIG     GPIO_34_GPIO34
#define DC_Module4_OC_FAULT_IN_GPIO                1
#define DC_Module4_OC_FAULT_IN_GPIO_PIN_CONFIG     GPIO_1_GPIO1
#define DC_Module5_OC_FAULT_IN_GPIO                58
#define DC_Module5_OC_FAULT_IN_GPIO_PIN_CONFIG     GPIO_58_GPIO58
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (modify GPIO settings for Polarity fault) //
/*------------GPIO DEFINES for DC module Reversed Polarity Fault signal (From Module)---------------*/
#define DC_Module1_POLARITY_FAULT_IN_GPIO                7
#define DC_Module1_POLARITY_FAULT_IN_GPIO_PIN_CONFIG     GPIO_7_GPIO7
#define DC_Module2_POLARITY_FAULT_IN_GPIO                5
#define DC_Module2_POLARITY_FAULT_IN_GPIO_PIN_CONFIG     GPIO_5_GPIO5
#define DC_Module3_POLARITY_FAULT_IN_GPIO                59
#define DC_Module3_POLARITY_FAULT_IN_GPIO_PIN_CONFIG     GPIO_59_GPIO59
#define DC_Module4_POLARITY_FAULT_IN_GPIO                3
#define DC_Module4_POLARITY_FAULT_IN_GPIO_PIN_CONFIG     GPIO_3_GPIO3
#define DC_Module5_POLARITY_FAULT_IN_GPIO                56
#define DC_Module5_POLARITY_FAULT_IN_GPIO_PIN_CONFIG     GPIO_56_GPIO56

/*------------GPIO DEFINES for DC module Enable/Disable---------------*/
#define DC_Modulex_En_Active_Low          0 // 1: active low, 0: active high // ------ BK: Jan/16/2019 ----- //
// ---------------------------------------------------------------------------------------------------- BK: Jan/16/2019 ----- (add #if - #else - #endif) //
#if DC_Modulex_En_Active_Low

#define DC_Module1_En_GPIO                23
#define DC_Module1_En_GPIO_PIN_CONFIG     GPIO_23_GPIO23
#define GPIO_ENABLE_DC_MODULE_1           GPIO_GPACLEAR_GPIO23
#define GPIO_DISABLE_DC_MODULE_1          GPIO_GPASET_GPIO23

#define DC_Module2_En_GPIO                9
#define DC_Module2_En_GPIO_PIN_CONFIG     GPIO_9_GPIO9
#define GPIO_ENABLE_DC_MODULE_2           GPIO_GPACLEAR_GPIO9 // ------ BK: Jan/09/2019 ----- (change enable to clear, disable to set) //
#define GPIO_DISABLE_DC_MODULE_2          GPIO_GPASET_GPIO9

#define DC_Module3_En_GPIO                10
#define DC_Module3_En_GPIO_PIN_CONFIG     GPIO_10_GPIO10
#define GPIO_ENABLE_DC_MODULE_3           GPIO_GPACLEAR_GPIO10
#define GPIO_DISABLE_DC_MODULE_3          GPIO_GPASET_GPIO10

// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 8 lines) //
#define DC_Module4_En_GPIO                8
#define DC_Module4_En_GPIO_PIN_CONFIG     GPIO_8_GPIO8
#define GPIO_ENABLE_DC_MODULE_4           GPIO_GPACLEAR_GPIO8
#define GPIO_DISABLE_DC_MODULE_4          GPIO_GPASET_GPIO8

#define DC_Module5_En_GPIO                57
#define DC_Module5_En_GPIO_PIN_CONFIG     GPIO_57_GPIO57
#define GPIO_ENABLE_DC_MODULE_5           GPIO_GPBCLEAR_GPIO57
#define GPIO_DISABLE_DC_MODULE_5          GPIO_GPBSET_GPIO57

#else

#define DC_Module1_En_GPIO                23
#define DC_Module1_En_GPIO_PIN_CONFIG     GPIO_23_GPIO23
#define GPIO_ENABLE_DC_MODULE_1           GPIO_GPASET_GPIO23
#define GPIO_DISABLE_DC_MODULE_1          GPIO_GPACLEAR_GPIO23

#define DC_Module2_En_GPIO                9
#define DC_Module2_En_GPIO_PIN_CONFIG     GPIO_9_GPIO9
#define GPIO_ENABLE_DC_MODULE_2           GPIO_GPASET_GPIO9
#define GPIO_DISABLE_DC_MODULE_2          GPIO_GPACLEAR_GPIO9

#define DC_Module3_En_GPIO                10
#define DC_Module3_En_GPIO_PIN_CONFIG     GPIO_10_GPIO10
#define GPIO_ENABLE_DC_MODULE_3           GPIO_GPASET_GPIO10
#define GPIO_DISABLE_DC_MODULE_3          GPIO_GPACLEAR_GPIO10

#define DC_Module4_En_GPIO                8
#define DC_Module4_En_GPIO_PIN_CONFIG     GPIO_8_GPIO8
#define GPIO_ENABLE_DC_MODULE_4           GPIO_GPASET_GPIO8
#define GPIO_DISABLE_DC_MODULE_4          GPIO_GPACLEAR_GPIO8

#define DC_Module5_En_GPIO                57
#define DC_Module5_En_GPIO_PIN_CONFIG     GPIO_57_GPIO57
#define GPIO_ENABLE_DC_MODULE_5           GPIO_GPBSET_GPIO57
#define GPIO_DISABLE_DC_MODULE_5          GPIO_GPBCLEAR_GPIO57

#endif

// Host Terminal
#define GPIO_HOST_SCIRX 28
#define GPIO_HOST_SCITX 29
#define GPIO_HOST_SCIRX_PIN_CONFIG GPIO_28_SCIRXDA
#define GPIO_HOST_SCITX_PIN_CONFIG GPIO_29_SCITXDA

/*------------BOARD PROTECTION DEFINES--------*/
//#define  OUTPUT_OC_PROTECTION_ENABLE  1
//#define  OUTPUT_OV_PROTECTION_ENABLE  0
//#define  TEMP_OT_PROTECTION_ENABLE    0
//#define  INPUT_UV_PROTECTION_ENABLE   0
//#define  INPUT_OV_PROTECTION_ENABLE   0
//#define  REVERSE_PROTECTION_ENABLE    0  // no fault - 1; fault - 0
//#define  INPUT_OC_PROTECTION_ENABLE   0 // Digital input, no fault - 1; fault - 0
//extern int OUTPUT_OC_PROTECTION_ENABLE;
//extern int OUTPUT_OV_PROTECTION_ENABLE;
//extern int TEMP_OT_PROTECTION_ENABLE;
//extern int INPUT_UV_PROTECTION_ENABLE;
//extern int INPUT_OV_PROTECTION_ENABLE;
//extern int REVERSE_PROTECTION_ENABLE;
//extern int INPUT_OC_PROTECTION_ENABLE;

// USE_MODULE_ANALOG_AVERAGE
// 1 ==> Analog input values from each Module are summed and averaged. That average value is used to determine if a Fault condition is occurring.
// 0 ==> Analog input values from any Module, on it's own, is capable of generating a Fault condition.
#define  USE_MODULE_ANALOG_AVERAGE    0

// Debug testing defines
#define FAKE_VALID_VIN 0 //1    // Define this to '1' to make system think we have a Vin >= IP_MIN_RUNNING_VOL

#define BOARD_PROTECTION_OUT_CUR1_CMPSS_BASE               CMPSS1_BASE
#define BOARD_PROTECTION_OUT_CUR1_CMPSS_ASYSCTRL_CMPHPMUX  ASYSCTL_CMPHPMUX_SELECT_1  // Select the Chip Mux for Analog Group 1 (A2/B6/PGA1_OF)
#define BOARD_PROTECTION_OUT_CUR1_CMPSS_ASYSCTRL_MUX_VALUE 0                          // Per Table 12-1 HPMUXSEL=0
#define BOARD_PROTECTION_OUT_CUR1_XBAR_MUX                 XBAR_MUX00                 // CMPSS1.CTRIPH
#define BOARD_PROTECTION_OUT_CUR1_XBAR_MUX_VAL             XBAR_EPWM_MUX00_CMPSS1_CTRIPH

#define BOARD_PROTECTION_OUT_CUR2_CMPSS_BASE               CMPSS2_BASE
#define BOARD_PROTECTION_OUT_CUR2_CMPSS_ASYSCTRL_CMPHPMUX  ASYSCTL_CMPHPMUX_SELECT_2  // Select the Chip Mux for Analog Group 2 (A4/B8/PGA2_OF)
#define BOARD_PROTECTION_OUT_CUR2_CMPSS_ASYSCTRL_MUX_VALUE 0                          // Per Table 12-1 HPMUXSEL=0
#define BOARD_PROTECTION_OUT_CUR2_XBAR_MUX                 XBAR_MUX02                 // CMPSS2.CTRIPH
#define BOARD_PROTECTION_OUT_CUR2_XBAR_MUX_VAL             XBAR_EPWM_MUX02_CMPSS2_CTRIPH

#define BOARD_PROTECTION_OUT_CUR3_CMPSS_BASE               CMPSS7_BASE
#define BOARD_PROTECTION_OUT_CUR3_CMPSS_ASYSCTRL_CMPHPMUX  ASYSCTL_CMPHPMUX_SELECT_7  // Select the Chip Mux for Analog Group 7 (A10/B1/C10/PGA7_OF)
#define BOARD_PROTECTION_OUT_CUR3_CMPSS_ASYSCTRL_MUX_VALUE 0                          // Per Table 12-1 HPMUXSEL=0
#define BOARD_PROTECTION_OUT_CUR3_XBAR_MUX                 XBAR_MUX12                 // CMPSS7.CTRIPH
#define BOARD_PROTECTION_OUT_CUR3_XBAR_MUX_VAL             XBAR_EPWM_MUX12_CMPSS7_CTRIPH

// ---------------------------------------------------------------------------------------------------- BK: Jan/15/2019 ----- (add 10 lines) //
// Refer to Section 18.3.10, Fig. 18-56 and Table. 18-13.
// CMPSSx output
//   CTRIPOUTH, CTRIPOUTL go to output X-BAR only
//   CTRIPH, CTRIPL go to ePWM X-BAR only
#define BOARD_PROTECTION_OUT_CUR4_CMPSS_BASE               CMPSS2_BASE
#define BOARD_PROTECTION_OUT_CUR4_CMPSS_ASYSCTRL_CMPLPMUX  ASYSCTL_CMPLPMUX_SELECT_2  // Select the Chip Mux for Analog Group 2
#define BOARD_PROTECTION_OUT_CUR4_CMPSS_ASYSCTRL_MUX_VALUE 1                          // Per Table 12-1 LPMUXSEL=1
#define BOARD_PROTECTION_OUT_CUR4_XBAR_MUX                 XBAR_MUX03                 // CMPSS2.CTRIPL,
#define BOARD_PROTECTION_OUT_CUR4_XBAR_MUX_VAL             XBAR_EPWM_MUX03_CMPSS2_CTRIPL

#define BOARD_PROTECTION_OUT_CUR5_CMPSS_BASE               CMPSS1_BASE
#define BOARD_PROTECTION_OUT_CUR5_CMPSS_ASYSCTRL_CMPLPMUX  ASYSCTL_CMPLPMUX_SELECT_1  // Select the Chip Mux for Analog Group 1
#define BOARD_PROTECTION_OUT_CUR5_CMPSS_ASYSCTRL_MUX_VALUE 1                          // Per Table 12-1 LPMUXSEL=1
#define BOARD_PROTECTION_OUT_CUR5_XBAR_MUX                 XBAR_MUX01                 // CMPSS1.CTRIPL
#define BOARD_PROTECTION_OUT_CUR5_XBAR_MUX_VAL             XBAR_EPWM_MUX01_CMPSS1_CTRIPL

// ---------------------------------------------------------------------------------------------------- BK: Jan/15/2019 ----- (commented 5 lines) //
/*
#define BOARD_PROTECTION_IP_VOL_CMPSS_BASE               CMPSS7_BASE
#define BOARD_PROTECTION_IP_VOL_CMPSS_ASYSCTRL_CMPHPMUX  ASYSCTL_CMPHPMUX_SELECT_7   // TODO - match this to the correct A/D pin when it's known
#define BOARD_PROTECTION_IP_VOL_CMPSS_ASYSCTRL_MUX_VALUE 0

#define BOARD_PROTECTION_IP_VOL_XBAR_MUX                 XBAR_MUX12
#define BOARD_PROTECTION_IP_VOL_XBAR_MUX_VAL             XBAR_EPWM_MUX12_CMPSS7_CTRIPH
*/

// Currently there is support for up to 3 connected Modules. Vout, Iout and Temp data from these modules is averaged together.
// Support for > 3 Modules will require adding code to do so (ADC SOC configuration, etc)
// To run with < 3 Modules simply change the below number.
#define NUM_SUPPORTED_MODULES  1
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (modify numbers) //
#define MAX_SUPPORTED_MODULES  5



#define SENSED_READING_AVERAGE (float)(1.0 / NUM_SUPPORTED_MODULES)

// When in a Fault state, we will attempt recovery when an UV drop and recover occurs, but only up to the following # of times.
// Once this count is reached we will require a system restart.
#define MAX_FAULT_RETRIES      3
#define FAULT_RECOVERY_COUNTDOWN 1000           // Number of High Speed ISR interrupts that must occur before we will try Fault Recovery. 1000 => ~10ms



/*-----------------------ADC DEFINES-----------------------*/
// ADC defines for output current sense - DC module-->1  Range: 0 - 3.3V (PWM)
#define OUT_CUR_1_ADC_MODULE  ADCA_BASE
#define OUT_CUR_1_ADC_SOC_NO1   ADC_SOC_NUMBER0           // Module 1 output current sense sample 1
#define OUT_CUR_1_ADC_SOC_NO2   ADC_SOC_NUMBER7           // Module 1 output current sense sample 2
#define OUT_CUR_1_ADC_PIN     ADC_CH_ADCIN2
#define OUT_CUR_1_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_CUR_1_ADC_ACQPS_SYS_CLKS 20
#define OUT_CUR_1_ADCRESULTREGBASE ADCARESULT_BASE

// ADC defines for output current sense - DC module-->2  Range: 0 - 3.3V (PWM)
#define OUT_CUR_2_ADC_MODULE  ADCA_BASE
#define OUT_CUR_2_ADC_SOC_NO1  ADC_SOC_NUMBER3            // Module 2 output current sense sample 1
#define OUT_CUR_2_ADC_SOC_NO2  ADC_SOC_NUMBER9            // Module 2 output current sense sample 2
#define OUT_CUR_2_ADC_PIN     ADC_CH_ADCIN4
#define OUT_CUR_2_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_CUR_2_ADC_ACQPS_SYS_CLKS 20
#define OUT_CUR_2_ADCRESULTREGBASE ADCARESULT_BASE

// ADC defines for output current sense - DC module-->3  Range: 0 - 3.3V (PWM)
#define OUT_CUR_3_ADC_MODULE  ADCB_BASE
#define OUT_CUR_3_ADC_SOC_NO1  ADC_SOC_NUMBER6            // Module 3 output current sense sample 1
#define OUT_CUR_3_ADC_SOC_NO2  ADC_SOC_NUMBER10           // Module 3 output current sense sample 2
#define OUT_CUR_3_ADC_PIN     ADC_CH_ADCIN1
#define OUT_CUR_3_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_CUR_3_ADC_ACQPS_SYS_CLKS 20
#define OUT_CUR_3_ADCRESULTREGBASE ADCBRESULT_BASE
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 16 lines) //
// ADC defines for output current sense - DC module-->4  Range: 0 - 3.3V (PWM)
#define OUT_CUR_4_ADC_MODULE  ADCC_BASE
#define OUT_CUR_4_ADC_SOC_NO1   ADC_SOC_NUMBER0           // Module 1 output current sense sample 1
#define OUT_CUR_4_ADC_SOC_NO2   ADC_SOC_NUMBER3           // Module 1 output current sense sample 2
#define OUT_CUR_4_ADC_PIN     ADC_CH_ADCIN1
#define OUT_CUR_4_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_CUR_4_ADC_ACQPS_SYS_CLKS 20
#define OUT_CUR_4_ADCRESULTREGBASE ADCCRESULT_BASE

// ADC defines for output current sense - DC module-->5  Range: 0 - 3.3V (PWM)
#define OUT_CUR_5_ADC_MODULE  ADCC_BASE
#define OUT_CUR_5_ADC_SOC_NO1  ADC_SOC_NUMBER6            // Module 2 output current sense sample 1
#define OUT_CUR_5_ADC_SOC_NO2  ADC_SOC_NUMBER9            // Module 2 output current sense sample 2
#define OUT_CUR_5_ADC_PIN     ADC_CH_ADCIN0
#define OUT_CUR_5_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_CUR_5_ADC_ACQPS_SYS_CLKS 20
#define OUT_CUR_5_ADCRESULTREGBASE ADCCRESULT_BASE


// ADC defines for output voltage sense - DC module-->1  Range: 0 - 28V
#define OUT_VOL_1_ADC_MODULE  ADCA_BASE
#define OUT_VOL_1_ADC_SOC_NO1  ADC_SOC_NUMBER1           // Module 1 output voltage sample 1
#define OUT_VOL_1_ADC_SOC_NO2  ADC_SOC_NUMBER8           // Module 1 output voltage sample 2
#define OUT_VOL_1_ADC_PIN     ADC_CH_ADCIN3
#define OUT_VOL_1_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_VOL_1_ADC_ACQPS_SYS_CLKS 20
#define OUT_VOL_1_ADCRESULTREGBASE ADCARESULT_BASE

// ADC defines for output voltage sense - DC module-->2  Range: 0 - 28V
#define OUT_VOL_2_ADC_MODULE  ADCA_BASE
#define OUT_VOL_2_ADC_SOC_NO1  ADC_SOC_NUMBER4           // Module 2 output voltage sample 1
#define OUT_VOL_2_ADC_SOC_NO2  ADC_SOC_NUMBER11          // Module 2 output voltage sample 2
#define OUT_VOL_2_ADC_PIN     ADC_CH_ADCIN5
#define OUT_VOL_2_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_VOL_2_ADC_ACQPS_SYS_CLKS 20
#define OUT_VOL_2_ADCRESULTREGBASE ADCARESULT_BASE

// ADC defines for output voltage sense - DC module-->3  Range: 0 - 28V
#define OUT_VOL_3_ADC_MODULE  ADCB_BASE
#define OUT_VOL_3_ADC_SOC_NO1  ADC_SOC_NUMBER0           // Module 3 output voltage sample 1
#define OUT_VOL_3_ADC_SOC_NO2  ADC_SOC_NUMBER2           // Module 3 output voltage sample 2
#define OUT_VOL_3_ADC_PIN     ADC_CH_ADCIN0
#define OUT_VOL_3_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_VOL_3_ADC_ACQPS_SYS_CLKS 20
#define OUT_VOL_3_ADCRESULTREGBASE ADCBRESULT_BASE
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 16 lines) //
// ADC defines for output voltage sense - DC module-->4  Range: 0 - 28V
#define OUT_VOL_4_ADC_MODULE  ADCC_BASE
#define OUT_VOL_4_ADC_SOC_NO1  ADC_SOC_NUMBER1           // Module 1 output voltage sample 1
#define OUT_VOL_4_ADC_SOC_NO2  ADC_SOC_NUMBER4           // Module 1 output voltage sample 2
#define OUT_VOL_4_ADC_PIN     ADC_CH_ADCIN3
#define OUT_VOL_4_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_VOL_4_ADC_ACQPS_SYS_CLKS 20
#define OUT_VOL_4_ADCRESULTREGBASE ADCCRESULT_BASE

// ADC defines for output voltage sense - DC module-->5  Range: 0 - 28V
#define OUT_VOL_5_ADC_MODULE  ADCC_BASE
#define OUT_VOL_5_ADC_SOC_NO1  ADC_SOC_NUMBER7           // Module 2 output voltage sample 1
#define OUT_VOL_5_ADC_SOC_NO2  ADC_SOC_NUMBER10          // Module 2 output voltage sample 2
#define OUT_VOL_5_ADC_PIN     ADC_CH_ADCIN4
#define OUT_VOL_5_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define OUT_VOL_5_ADC_ACQPS_SYS_CLKS 20
#define OUT_VOL_5_ADCRESULTREGBASE ADCCRESULT_BASE


// ADC defines for temperature sense - DC module-->1  Range: 0 - 3.3V (PWM)
#define TEMP_1_ADC_MODULE  ADCA_BASE
#define TEMP_1_ADC_SOC_NO1     ADC_SOC_NUMBER2           // Module 1 temperature sense sample 1
#define TEMP_1_ADC_SOC_NO2     ADC_SOC_NUMBER12          // Module 1 temperature sense sample 2
#define TEMP_1_ADC_PIN     ADC_CH_ADCIN6
#define TEMP_1_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define TEMP_1_ADC_ACQPS_SYS_CLKS 20
#define TEMP_1_ADCRESULTREGBASE ADCARESULT_BASE

// ADC defines for temperature sense - DC module-->2  Range: 0 - 3.3V (PWM)
#define TEMP_2_ADC_MODULE  ADCA_BASE
#define TEMP_2_ADC_SOC_NO1     ADC_SOC_NUMBER5           // Module 2 temperature sense sample 1
#define TEMP_2_ADC_SOC_NO2     ADC_SOC_NUMBER13          // Module 2 temperature sense sample 2
#define TEMP_2_ADC_PIN     ADC_CH_ADCIN8
#define TEMP_2_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define TEMP_2_ADC_ACQPS_SYS_CLKS 20
#define TEMP_2_ADCRESULTREGBASE ADCARESULT_BASE

// ADC defines for temperature sense - DC module-->3  Range: 0 - 3.3V (PWM)
#define TEMP_3_ADC_MODULE  ADCA_BASE
#define TEMP_3_ADC_SOC_NO1     ADC_SOC_NUMBER14           // Module 3 temperature sense sample 1
#define TEMP_3_ADC_SOC_NO2     ADC_SOC_NUMBER15           // Module 3 temperature sense sample 2
#define TEMP_3_ADC_PIN     ADC_CH_ADCIN9
#define TEMP_3_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define TEMP_3_ADC_ACQPS_SYS_CLKS 20
#define TEMP_3_ADCRESULTREGBASE ADCARESULT_BASE
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 16 lines) //
// ADC defines for temperature sense - DC module-->4  Range: 0 - 3.3V (PWM)
#define TEMP_4_ADC_MODULE  ADCC_BASE
#define TEMP_4_ADC_SOC_NO1     ADC_SOC_NUMBER2           // Module 1 temperature sense sample 1
#define TEMP_4_ADC_SOC_NO2     ADC_SOC_NUMBER5          // Module 1 temperature sense sample 2
#define TEMP_4_ADC_PIN     ADC_CH_ADCIN5
#define TEMP_4_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define TEMP_4_ADC_ACQPS_SYS_CLKS 20
#define TEMP_4_ADCRESULTREGBASE ADCCRESULT_BASE

// ADC defines for temperature sense - DC module-->5  Range: 0 - 3.3V (PWM)
#define TEMP_5_ADC_MODULE  ADCC_BASE
#define TEMP_5_ADC_SOC_NO1     ADC_SOC_NUMBER8           // Module 2 temperature sense sample 1
#define TEMP_5_ADC_SOC_NO2     ADC_SOC_NUMBER11          // Module 2 temperature sense sample 2
#define TEMP_5_ADC_PIN     ADC_CH_ADCIN2
#define TEMP_5_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define TEMP_5_ADC_ACQPS_SYS_CLKS 20
#define TEMP_5_ADCRESULTREGBASE ADCCRESULT_BASE

// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (commented 16 lines) //
// ADC defines for input current sense (Note - current settings here are for test only. These signals do not yet exist on the schematic.)
//#define IP_CUR_ADC_MODULE  ADCC_BASE
//#define IP_CUR_ADC_SOC_NO1  ADC_SOC_NUMBER0
//#define IP_CUR_ADC_SOC_NO2  ADC_SOC_NUMBER1
//#define IP_CUR_ADC_PIN     ADC_CH_ADCIN0
//#define IP_CUR_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
//#define IP_CUR_ADC_ACQPS_SYS_CLKS 20
//#define IP_CUR_ADCRESULTREGBASE ADCCRESULT_BASE
//
// ADC defines for input voltage sense. (Note - current settings here are for test only. These signals do not yet exist on the schematic.)
//#define IP_VOL_ADC_MODULE  ADCC_BASE
//#define IP_VOL_ADC_SOC_NO1  ADC_SOC_NUMBER2           // Input voltage sample 1
//#define IP_VOL_ADC_SOC_NO2  ADC_SOC_NUMBER3           // Input voltage sample 2
//#define IP_VOL_ADC_PIN     ADC_CH_ADCIN1
//#define IP_VOL_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
//#define IP_VOL_ADC_ACQPS_SYS_CLKS 20
//#define IP_VOL_ADCRESULTREGBASE ADCCRESULT_BASE

// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 40 lines, 5 Paragraph) //
// ADC defines for output voltage sense - DC module-->1  Range: 0 - 28V
#define IN_VOL_1_ADC_MODULE  ADCB_BASE
#define IN_VOL_1_ADC_SOC_NO1  ADC_SOC_NUMBER1           // Module 1 output voltage sample 1
#define IN_VOL_1_ADC_SOC_NO2  ADC_SOC_NUMBER7           // Module 1 output voltage sample 2
#define IN_VOL_1_ADC_PIN     ADC_CH_ADCIN2
#define IN_VOL_1_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define IN_VOL_1_ADC_ACQPS_SYS_CLKS 20
#define IN_VOL_1_ADCRESULTREGBASE ADCBRESULT_BASE

// ADC defines for output voltage sense - DC module-->2  Range: 0 - 28V
#define IN_VOL_2_ADC_MODULE  ADCB_BASE
#define IN_VOL_2_ADC_SOC_NO1  ADC_SOC_NUMBER3          // Module 2 output voltage sample 1
#define IN_VOL_2_ADC_SOC_NO2  ADC_SOC_NUMBER11          // Module 2 output voltage sample 2
#define IN_VOL_2_ADC_PIN     ADC_CH_ADCIN4
#define IN_VOL_2_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define IN_VOL_2_ADC_ACQPS_SYS_CLKS 20
#define IN_VOL_2_ADCRESULTREGBASE ADCBRESULT_BASE

// ADC defines for output voltage sense - DC module-->3  Range: 0 - 28V
#define IN_VOL_3_ADC_MODULE  ADCC_BASE
#define IN_VOL_3_ADC_SOC_NO1  ADC_SOC_NUMBER12           // Module 3 output voltage sample 1
#define IN_VOL_3_ADC_SOC_NO2  ADC_SOC_NUMBER15           // Module 3 output voltage sample 2
#define IN_VOL_3_ADC_PIN     ADC_CH_ADCIN14
#define IN_VOL_3_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define IN_VOL_3_ADC_ACQPS_SYS_CLKS 20
#define IN_VOL_3_ADCRESULTREGBASE ADCCRESULT_BASE

// ADC defines for output voltage sense - DC module-->4  Range: 0 - 28V
#define IN_VOL_4_ADC_MODULE  ADCB_BASE
#define IN_VOL_4_ADC_SOC_NO1  ADC_SOC_NUMBER4           // Module 1 output voltage sample 1
#define IN_VOL_4_ADC_SOC_NO2  ADC_SOC_NUMBER12           // Module 1 output voltage sample 2
#define IN_VOL_4_ADC_PIN     ADC_CH_ADCIN15
#define IN_VOL_4_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define IN_VOL_4_ADC_ACQPS_SYS_CLKS 20
#define IN_VOL_4_ADCRESULTREGBASE ADCBRESULT_BASE

// ADC defines for output voltage sense - DC module-->5  Range: 0 - 28V
#define IN_VOL_5_ADC_MODULE  ADCA_BASE
#define IN_VOL_5_ADC_SOC_NO1  ADC_SOC_NUMBER6           // Module 2 output voltage sample 1
#define IN_VOL_5_ADC_SOC_NO2  ADC_SOC_NUMBER10          // Module 2 output voltage sample 2
#define IN_VOL_5_ADC_PIN     ADC_CH_ADCIN1
#define IN_VOL_5_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define IN_VOL_5_ADC_ACQPS_SYS_CLKS 20
#define IN_VOL_5_ADCRESULTREGBASE ADCARESULT_BASE

// ADC defines for ADC offset sense
#define VOFFSET_ADC_MODULE  ADCC_BASE
#define VOFFSET_ADC_SOC_NO1  ADC_SOC_NUMBER9
#define VOFFSET_ADC_SOC_NO2  ADC_SOC_NUMBER10
#define VOFFSET_ADC_SOC_NO3  ADC_SOC_NUMBER11
#define VOFFSET_ADC_PIN     ADC_CH_ADCIN4
#define VOFFSET_ADC_TRIG_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define VOFFSET_ADC_ACQPS_SYS_CLKS 20
#define VOFFSET_ADCRESULTREGBASE ADCCRESULT_BASE

// Enable (1) or disable (0) calibration routine to get offset for sensing signals
#define ENABLE_CALIBRATE_OFFSET 0

/*-----------------COMPARATOR Defines-----------------*/

#define CMPSS_FILTER_SAMPLE_PRESCALE  2
#define CMPSS_FILTER_SAMPLE_WINDOW    10
#define CMPSS_FILTER_THRESHOLD        7




/*-----------------Profiling GPIO Defines-----------------*/

#define GPIO_PROFILING1 0
#define GPIO_PROFILING1_SET GPIO_GPASET_GPIO0
#define GPIO_PROFILING1_CLEAR GPIO_GPACLEAR_GPIO0
#define GPIO_PROFILING1_PIN_CONFIG GPIO_0_GPIO0

#define GPIO_PROFILING2 17
#define GPIO_PROFILING2_SET GPIO_GPASET_GPIO17
#define GPIO_PROFILING2_CLEAR GPIO_GPACLEAR_GPIO17
#define GPIO_PROFILING2_PIN_CONFIG GPIO_17_GPIO17

#define GPIO_LED1 31
#define GPIO_LED1_SET GPIO_GPASET_GPIO31
#define GPIO_LED1_CLEAR GPIO_GPACLEAR_GPIO31
#define GPIO_LED1_PIN_CONFIG GPIO_31_GPIO31


#define GPIO_LED2 34
#define GPIO_LED2_SET GPIO_GPASET_GPIO34
#define GPIO_LED2_CLEAR GPIO_GPACLEAR_GPIO34
#define GPIO_LED2_PIN_CONFIG GPIO_34_GPIO34

/*----------------- Instrumentation code Defines-----------------*/

#define MAX_ADC_VOLTAGE             (float)3.3      // Max voltage ADC input will see. Corresponds to a 0xFFF reading.
#define AVG_MON_WINDOW_SIZE             100         // The size of the averaging buffer and hence the number of samples.
#define MAX_SENSE_INPUT_CURRENT     (float)20.0
#define MAX_SENSE_INPUT_VOL         (float)900.0
#define MAX_SENSE_OUTPUT_CURRENT    (float)155.5323  // Iout_max= 48.626 * 3.3- 4.9335 = 155.5323
#define MAX_SENSE_OUTPUT_VOLTAGE    (float)30.0    // 28V is the maximum expected voltage on VBat
//#define MAX_SENSE_OUTPUT_VOLTAGE    (float) 28.0/3.035    // 28*(6.2/(6.2+51))=3.035,
                                                   // added by JL 10/25/2018
#define MAX_SENSE_TEMPERATURE       (float)120.0   // 195 degrees would be a 'zero' reading, based on interpolated data. We would never see that.

// Linear equations for Temperature and Iout, based on 0.85
//#define TEMPERATURE_SLOPE1 (float) (-12.381)
//#define TEMPERATURE_SLOPE2 (float) (70.516)
//#define TEMPERATURE_SLOPE3 (float) (-168.92)
//#define TEMPERATURE_YINT  (float) (241.95)
// ---------------------------------------------------------------------------------------------------- JL: Feb/13/2019 ----- (update coefficiency based on actual test 0.8) //
//#define TEMPERATURE_SLOPE1 (float) (-9.7049)
//#define TEMPERATURE_SLOPE2 (float) (50.758)
//#define TEMPERATURE_SLOPE3 (float) (-130.57)
//#define TEMPERATURE_YINT  (float) (218.29)
// ---------------------------------------------------------------------------------------------------- JL: Feb/13/2019 ----- (update coefficiency based on actual test 0.825) //
//#define TEMPERATURE_SLOPE1 (float) (-8.916)
//#define TEMPERATURE_SLOPE2 (float) (47.581)
//#define TEMPERATURE_SLOPE3 (float) (-125.35)
//#define TEMPERATURE_YINT  (float) (217.26)
// ---------------------------------------------------------------------------------------------------- JL: update based on Miro's new simulation table //
#define TEMPERATURE_SLOPE1 (float) (-10.893)
#define TEMPERATURE_SLOPE2 (float) (55.746)
#define TEMPERATURE_SLOPE3 (float) (-135.57)
#define TEMPERATURE_YINT  (float) (219.19)



//#define Iout_SLOPE     (float) (48.626)
//#define Iout_YINT      (float) (-4.9335)
// ---------------------------------------------------------------------------------------------------- JL: Feb/12/2019 ----- (update coefficiency based on actual test) //
#define Iout_SLOPE     (float) (50.51)
#define Iout_YINT      (float) (-7.7516)
//#define Vout_SLOPE     (float) (8.7753)
//#define Vout_YINT      (float) (3.7033)
// ---------------------------------------------------------------------------------------------------- JL: Feb/12/2019 ----- (update coefficiency based on actual test) //
#define Vout_SLOPE     (float) (8.7976)
#define Vout_YINT      (float) (3.3875)
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 2 lines, and dummy value) //
//#define Vin_SLOPE     (float) (242.95)
//#define Vin_YINT      (float) (94.633)
// ---------------------------------------------------------------------------------------------------- JL: Feb/12/2019 ----- (update coefficiency based on actual test) //
#define Vin_SLOPE     (float) (243.19)
#define Vin_YINT      (float) (91.529)

// sensor correction, added by JL 10/31/2018
#define  op_cur_sensed_offset0 (float) (0.0)
#define  op_cur_sensed_offset1 (float) (0.0)
#define  op_cur_sensed_offset2 (float) (0.0)
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 2 lines) //
#define  op_cur_sensed_offset3 (float) (0.0)
#define  op_cur_sensed_offset4 (float) (0.0)

/*----------------- TRIP LIMITS -----------------*/

#define OUTPUT_OV_TRIP_LIMIT        (float)32.5
#define TEMP_OT_TRIP_LIMIT          (float)150.0
#define OUTPUT_OC_TRIP_LIMIT        (float)100.0 //150.0
// ---------------------------------------------------------------------------------------------------- BK: Feb/03/2019 ----- (add 1 line) //
#define OUTPUT_OC_TRIP_LIMIT_LOOP   (float)30.0 //70.0//100.0 // recommand <= OUTPUT_OC_TRIP_LIMIT

// Use a fault and a running minimum to prevent oscillating between states if Vin is just around IP_UV_TRIP_LIMIT.
#define IP_MIN_RUNNING_VOL          (float)280.0 //320.0 //480.0 //10 //750 //10 //460.0    // Input Voltage must rise across this for System to be running
#define IP_UV_TRIP_LIMIT            (float)425.0 //420.0    // Input Voltage  below this is a Fault
#define IP_OV_TRIP_LIMIT            (float)870.0    // Power supply maximum allowable supplied Voltage 880V -> ADC = 3.23

#ifdef _DEBUG_ROM
#define DISPLAY_DATA_SIZE   400       // HOST display buffer
#else
#define DISPLAY_DATA_SIZE   1         // Easier to just make it smaller when not in use.
#endif


#endif //_PROJSETTINGS_H

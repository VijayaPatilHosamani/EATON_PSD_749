/* ============================================================================
System Name:

File Name:	  	converter_board.h

Target:         F28004x

Author:

Description:	This file consists of common variables and functions
                for a particular hardware board, like functions to read current
                and voltage signals on the board and functions to setup the
                basic peripherals of the board
		        This file must be settings independent, all settings dependent
				code should reside in the parent solution project.
===========================================================================  */

#ifndef CONVERTER_BOARD_H
#define CONVERTER_BOARD_H


//*****************************************************************************
// the includes
//*****************************************************************************
#define MATH_TYPE 1
#include "IQmathLib.h"
// Include files for device support, F2806x in this case
#include "driverlib.h"
#include "device.h"
#include "converter_settings.h"

//*****************************************************************************
//defines
//*****************************************************************************

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PWM_TRIP_STATUS EPWM_getTripZoneFlagStatus
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (commented 2 lines) //
//#define IP_CUR_FB       ADC_readResult(IP_CUR_ADCRESULTREGBASE,IP_CUR_ADC_SOC_NO1)
//#define IP_CUR_FB_2     ADC_readResult(IP_CUR_ADCRESULTREGBASE,IP_CUR_ADC_SOC_NO2)

// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (commented 2 lines and add 10 lines) //
//#define IP_VOL_FB       ADC_readResult(IP_VOL_ADCRESULTREGBASE,IP_VOL_ADC_SOC_NO1)
//#define IP_VOL_FB_2     ADC_readResult(IP_VOL_ADCRESULTREGBASE,IP_VOL_ADC_SOC_NO2)
#define IN_VOL_1_FB    ADC_readResult(IN_VOL_1_ADCRESULTREGBASE,IN_VOL_1_ADC_SOC_NO1)
#define IN_VOL_1_FB_2  ADC_readResult(IN_VOL_1_ADCRESULTREGBASE,IN_VOL_1_ADC_SOC_NO2)
#define IN_VOL_2_FB    ADC_readResult(IN_VOL_2_ADCRESULTREGBASE,IN_VOL_2_ADC_SOC_NO1)
#define IN_VOL_2_FB_2  ADC_readResult(IN_VOL_2_ADCRESULTREGBASE,IN_VOL_2_ADC_SOC_NO2)
#define IN_VOL_3_FB    ADC_readResult(IN_VOL_3_ADCRESULTREGBASE,IN_VOL_3_ADC_SOC_NO1)
#define IN_VOL_3_FB_2  ADC_readResult(IN_VOL_3_ADCRESULTREGBASE,IN_VOL_3_ADC_SOC_NO2)
#define IN_VOL_4_FB    ADC_readResult(IN_VOL_4_ADCRESULTREGBASE,IN_VOL_4_ADC_SOC_NO1)
#define IN_VOL_4_FB_2  ADC_readResult(IN_VOL_4_ADCRESULTREGBASE,IN_VOL_4_ADC_SOC_NO2)
#define IN_VOL_5_FB    ADC_readResult(IN_VOL_5_ADCRESULTREGBASE,IN_VOL_5_ADC_SOC_NO1)
#define IN_VOL_5_FB_2  ADC_readResult(IN_VOL_5_ADCRESULTREGBASE,IN_VOL_5_ADC_SOC_NO2)

#define TEMP_1_FB	    ADC_readResult(TEMP_1_ADCRESULTREGBASE,TEMP_1_ADC_SOC_NO1)
#define TEMP_1_FB_2	    ADC_readResult(TEMP_1_ADCRESULTREGBASE,TEMP_1_ADC_SOC_NO2)
#define TEMP_2_FB       ADC_readResult(TEMP_2_ADCRESULTREGBASE,TEMP_2_ADC_SOC_NO1)
#define TEMP_2_FB_2     ADC_readResult(TEMP_2_ADCRESULTREGBASE,TEMP_2_ADC_SOC_NO2)
#define TEMP_3_FB       ADC_readResult(TEMP_3_ADCRESULTREGBASE,TEMP_3_ADC_SOC_NO1)
#define TEMP_3_FB_2     ADC_readResult(TEMP_3_ADCRESULTREGBASE,TEMP_3_ADC_SOC_NO2)
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 4 lines) //
#define TEMP_4_FB       ADC_readResult(TEMP_4_ADCRESULTREGBASE,TEMP_4_ADC_SOC_NO1)
#define TEMP_4_FB_2     ADC_readResult(TEMP_4_ADCRESULTREGBASE,TEMP_4_ADC_SOC_NO2)
#define TEMP_5_FB       ADC_readResult(TEMP_5_ADCRESULTREGBASE,TEMP_5_ADC_SOC_NO1)
#define TEMP_5_FB_2     ADC_readResult(TEMP_5_ADCRESULTREGBASE,TEMP_5_ADC_SOC_NO2)

#define OUT_VOL_1_FB    ADC_readResult(OUT_VOL_1_ADCRESULTREGBASE,OUT_VOL_1_ADC_SOC_NO1)
#define OUT_VOL_1_FB_2  ADC_readResult(OUT_VOL_1_ADCRESULTREGBASE,OUT_VOL_1_ADC_SOC_NO2)
#define OUT_VOL_2_FB    ADC_readResult(OUT_VOL_2_ADCRESULTREGBASE,OUT_VOL_2_ADC_SOC_NO1)
#define OUT_VOL_2_FB_2  ADC_readResult(OUT_VOL_2_ADCRESULTREGBASE,OUT_VOL_2_ADC_SOC_NO2)
#define OUT_VOL_3_FB    ADC_readResult(OUT_VOL_3_ADCRESULTREGBASE,OUT_VOL_3_ADC_SOC_NO1)
#define OUT_VOL_3_FB_2  ADC_readResult(OUT_VOL_3_ADCRESULTREGBASE,OUT_VOL_3_ADC_SOC_NO2)
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 4 lines) //
#define OUT_VOL_4_FB    ADC_readResult(OUT_VOL_4_ADCRESULTREGBASE,OUT_VOL_4_ADC_SOC_NO1)
#define OUT_VOL_4_FB_2  ADC_readResult(OUT_VOL_4_ADCRESULTREGBASE,OUT_VOL_4_ADC_SOC_NO2)
#define OUT_VOL_5_FB    ADC_readResult(OUT_VOL_5_ADCRESULTREGBASE,OUT_VOL_5_ADC_SOC_NO1)
#define OUT_VOL_5_FB_2  ADC_readResult(OUT_VOL_5_ADCRESULTREGBASE,OUT_VOL_5_ADC_SOC_NO2)

#define OUT_CUR_1_FB       ADC_readResult(OUT_CUR_1_ADCRESULTREGBASE,OUT_CUR_1_ADC_SOC_NO1)
#define OUT_CUR_1_FB_2     ADC_readResult(OUT_CUR_1_ADCRESULTREGBASE,OUT_CUR_1_ADC_SOC_NO2)
#define OUT_CUR_2_FB       ADC_readResult(OUT_CUR_2_ADCRESULTREGBASE,OUT_CUR_2_ADC_SOC_NO1)
#define OUT_CUR_2_FB_2     ADC_readResult(OUT_CUR_2_ADCRESULTREGBASE,OUT_CUR_2_ADC_SOC_NO2)
#define OUT_CUR_3_FB       ADC_readResult(OUT_CUR_3_ADCRESULTREGBASE,OUT_CUR_3_ADC_SOC_NO1)
#define OUT_CUR_3_FB_2     ADC_readResult(OUT_CUR_3_ADCRESULTREGBASE,OUT_CUR_3_ADC_SOC_NO2)
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 4 lines) //
#define OUT_CUR_4_FB       ADC_readResult(OUT_CUR_4_ADCRESULTREGBASE,OUT_CUR_4_ADC_SOC_NO1)
#define OUT_CUR_4_FB_2     ADC_readResult(OUT_CUR_4_ADCRESULTREGBASE,OUT_CUR_4_ADC_SOC_NO2)
#define OUT_CUR_5_FB       ADC_readResult(OUT_CUR_5_ADCRESULTREGBASE,OUT_CUR_5_ADC_SOC_NO1)
#define OUT_CUR_5_FB_2     ADC_readResult(OUT_CUR_5_ADCRESULTREGBASE,OUT_CUR_5_ADC_SOC_NO2)

#define VOFFSET_FB       ADC_readResult(VOFFSET_ADCRESULTREGBASE,VOFFSET_ADC_SOC_NO1)
#define VOFFSET_FB_2     ADC_readResult(VOFFSET_ADCRESULTREGBASE,VOFFSET_ADC_SOC_NO2)



//#define ADC_PU_SCALE_FACTOR  (float)(0.000244140625)   // 1/4096 (2^12)
#define ADC_PU_SCALE_FACTOR  (float)(0.0008056640625)   // 3.3/4096 (2^12)  added by JL 10/23/2018
// sensor correction, added by JL 10/31/2018
//#define  op_cur_sensed_offset0 (float) (-0.09)
//#define  op_cur_sensed_offset1 (float) (0.0)
//#define  op_cur_sensed_offset2 (float) (0.0)

//definitions for selecting DACH reference
#define REFERENCE_VDDA     0

// ADC Configuration
#define REFERENCE_INTERNAL 0 //internal reference (12-bit only)
#define REFERENCE_EXTERNAL 1 //external reference

#define REFERENCE_VDAC     0
#define REFERENCE_VREF     1


#define DISABLE_AQ_SW_FRC(m)				HWREGH(m + EPWM_O_AQCSFRC) = 0x00;
#define AQ_SW_FORCE_PWMxA_HIGH_PWMxB_LOW(m)	HWREGH(m + EPWM_O_AQCSFRC) = 0x06;
#define AQ_SW_FORCE_PWMxA_LOW_PWMxB_HIGH(m)	HWREGH(m + EPWM_O_AQCSFRC) = 0x09;
#define AQ_SW_FORCE_PWMxA_LOW_PWMxB_LOW(m)	HWREGH(m + EPWM_O_AQCSFRC) = 0x05;

#define GET_TASK_A_TIMER_OVERFLOW_STATUS CPUTimer_getTimerOverflowStatus(CPUTIMER0_BASE)
#define CLEAR_TASK_A_TIMER_OVERFLOW_FLAG CPUTimer_clearOverflowFlag(CPUTIMER0_BASE)


//*****************************************************************************
// the function prototypes
//*****************************************************************************

void setupDevice(void);
void setupADCconversion(uint32_t adc_base, uint16_t adc_soc_number,
                        uint16_t adc_trig_source, uint16_t adc_pin,
                        uint16_t acqps);
void setupADC(void);
void setupProfilingGPIO(void);
void disablePWMCLKCounting(void);
void enablePWMCLKCounting(void);
void DeviceInit(void);
void MemCopy(void);
void InitFlash(void);
void setupEPWM5toTriggerADCSOC();
void setPinsAsPWM();
void setupDC_DCconverterPWM(uint32_t base1,uint32_t base2,uint32_t base3,uint32_t base4,uint32_t base5,uint32_t base6,uint16_t pwm_period_ticks);
void globalvariablesinit(void);
void setDCModuleEnablePins(void);
void setupCMPSS(uint32_t base1, float current_limit, float current_max_sense );
void setupCMPSS_L(uint32_t base3, float current_limit, float current_max_sense ); // ----- BK: Jan/15/2019 ----- (add new function) //
void setupBoardProtection(uint32_t base1, uint32_t base2, uint32_t base3, uint32_t base4, uint32_t base5);
void setInputPROTPins(void); // ----- BK: Jan/16/2019 ----- (add new function) //

interrupt void IpVoltageAndOpCurrentMonitoringISR(void);
interrupt void OpVoltageAndTempISR(void);
void SCIA_Init(long SCI_VBUS_CLOCKRATE, long SCI_BAUDRATE);
void setupSCIconnectionForHOST(void);

#ifdef __cplusplus
extern "C"{
#endif
void EPWM_setCounterCompareValueOptimized(uint32_t base, EPWM_CounterCompareModule compModule, uint16_t compCount);
#ifdef __cplusplus
}
#endif

// Inline functions

// setProfilingGPIO1
inline void setProfilingGPIO1(void)
{
    HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_PROFILING1_SET;
}

// resetProfilingGPIO1
inline void resetProfilingGPIO1(void)
{
    HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_PROFILING1_CLEAR;
}
// setProfilingGPIO2
inline void setProfilingGPIO2(void)
{
    HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_PROFILING2_SET;
}

// resetProfilingGPIO2
inline void resetProfilingGPIO2(void)
{
    HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_PROFILING2_CLEAR;
}
// EnableDcModule1
/*
inline void EnableDcModule(volatile uint32_t module)
{
    if (module == 0)
    {
        HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_DISABLE_DC_MODULE_1;
    }
    else
        if (module == 1)
        {
            HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_DISABLE_DC_MODULE_2;
        }
        else
            if (module == 2)
            {
                HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_DISABLE_DC_MODULE_3;
            }
    // ---------------------------------------------------------------------------------------------------- BK: Jan/16/2019 ----- (add 4 and 5) //
            else
                if (module == 3)
                {
                    HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_DISABLE_DC_MODULE_4;
                }
                else
                    if (module == 4)
                    {
                        HWREG(GPIODATA_BASE  + GPIO_O_GPBCLEAR ) = GPIO_DISABLE_DC_MODULE_5;
                    }
}

inline void DisableDcModule(volatile uint32_t module)
{

    if (module == 0)
    {
        HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_ENABLE_DC_MODULE_1;
    }
    else
        if (module == 1)
        {
            HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_ENABLE_DC_MODULE_2;
        }
        else
            if (module == 2)
            {
                HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_ENABLE_DC_MODULE_3;
            }
    // ---------------------------------------------------------------------------------------------------- BK: Jan/16/2019 ----- (add 4 and 5) //
            else
                if (module == 3)
                {
                    HWREG(GPIODATA_BASE + GPIO_O_GPASET) = GPIO_ENABLE_DC_MODULE_4;
                }
                else
                    if (module == 4)
                    {
                        HWREG(GPIODATA_BASE + GPIO_O_GPBSET) = GPIO_ENABLE_DC_MODULE_5;
                    }
}*/
// ---------------------------------------------------------------------------------------------------- BK: Jan/16/2019 ----- (Active low Enable) //

#if DC_Modulex_En_Active_Low

inline void EnableDcModule(volatile uint32_t module)
{
    if (module == 0)
    {
        HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_ENABLE_DC_MODULE_1;
    }
    else
        if (module == 1)
        {
            HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_ENABLE_DC_MODULE_2;
        }
        else
            if (module == 2)
            {
                HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_ENABLE_DC_MODULE_3;
            }
            else
                if (module == 3)
                {
                    HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_ENABLE_DC_MODULE_4;
                }
                else
                    if (module == 4)
                    {
                        HWREG(GPIODATA_BASE  + GPIO_O_GPBCLEAR ) = GPIO_ENABLE_DC_MODULE_5;
                    }
}

inline void DisableDcModule(volatile uint32_t module)
{

    if (module == 0)
    {
        HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_DISABLE_DC_MODULE_1;
    }
    else
        if (module == 1)
        {
            HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_DISABLE_DC_MODULE_2;
        }
        else
            if (module == 2)
            {
                HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_DISABLE_DC_MODULE_3;
            }
            else
                if (module == 3)
                {
                    HWREG(GPIODATA_BASE + GPIO_O_GPASET) = GPIO_DISABLE_DC_MODULE_4;
                }
                else
                    if (module == 4)
                    {
                        HWREG(GPIODATA_BASE + GPIO_O_GPBSET) = GPIO_DISABLE_DC_MODULE_5;
                    }
}


#else

inline void EnableDcModule(volatile uint32_t module)
{
    if (module == 0)
    {
        HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_ENABLE_DC_MODULE_1;
    }
    else
        if (module == 1)
        {
            HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_ENABLE_DC_MODULE_2;
        }
        else
            if (module == 2)
            {
                HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_ENABLE_DC_MODULE_3;
            }
            else
                if (module == 3)
                {
                    HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_ENABLE_DC_MODULE_4;
                }
                else
                    if (module == 4)
                    {
                        HWREG(GPIODATA_BASE  + GPIO_O_GPBSET ) = GPIO_ENABLE_DC_MODULE_5;
                    }
}

inline void DisableDcModule(volatile uint32_t module)
{

    if (module == 0)
    {
        HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_DISABLE_DC_MODULE_1;
    }
    else
        if (module == 1)
        {
            HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_DISABLE_DC_MODULE_2;
        }
        else
            if (module == 2)
            {
                HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_DISABLE_DC_MODULE_3;
            }
            else
                if (module == 3)
                {
                    HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR) = GPIO_DISABLE_DC_MODULE_4;
                }
                else
                    if (module == 4)
                    {
                        HWREG(GPIODATA_BASE + GPIO_O_GPBCLEAR) = GPIO_DISABLE_DC_MODULE_5;
                    }
}

#endif
// ---------------------------------------------------------------------------------------------------- BK: Jan/16/2019 ----- (End additional part #if) //


// ClearInterrupt
inline void clearInterrupt(uint16_t pie_group_no)
{
    Interrupt_clearACKGroup(pie_group_no);
}


/*******************************************************************************************************************//**
 * enablePWMInterruptGeneration()
 *
 * Input parameters:
 *  uint32_t base                        - base address of the PWM to configure.
 *
 *    Called by setupInterrupt() to enable the generation of the PWM4 Compare B interrupt which is used for
 *    IpVoltageAndOpCurrentMonitoringISR().
 *
 **********************************************************************************************************************/
inline void enablePWMInterruptGeneration(uint32_t base)
{
    EPWM_setInterruptSource(base,EPWM_INT_TBCTR_D_CMPB);
    EPWM_setInterruptEventCount(base,HS_MONITORING_ISR_FREQ_RATIO);
    EPWM_enableInterrupt(base);
    EPWM_clearEventTriggerInterruptFlag(base);
}

/*******************************************************************************************************************//**
 * setupInterrupt()
 *
 *    Called by main() at the end of startup sequence to configure the PWM4 and CPU Timer interrupts.
 *
 **********************************************************************************************************************/
inline void setupInterrupt(void)
{
    // setup interrupt configurations for IpVoltageAndOpCurrentMonitoring
    Interrupt_register(C28x_HS_MONITORING_INTERRUPT, &IpVoltageAndOpCurrentMonitoringISR);
    enablePWMInterruptGeneration(C28x_HS_MONITORING_INTERRUPT_TRIG_PWM_BASE);
    clearInterrupt(C28x_HS_MONITORING_INTERRUPT_PIE_GROUP_NO);
    Interrupt_enable(C28x_HS_MONITORING_INTERRUPT);

    // setup interrupt configurations for Monitoring and protection ISR
    CPUTimer_enableInterrupt(
            C28x_CTRL_LS_MONITORING_INTERRUPT_TRIG_CPUTIMER_BASE
            );
    CPUTimer_clearOverflowFlag(
            C28x_CTRL_LS_MONITORING_INTERRUPT_TRIG_CPUTIMER_BASE
            );
    Interrupt_register(C28x_CTRL_LS_MONITORING_INTERRUPT, &OpVoltageAndTempISR);
    Interrupt_enable(C28x_CTRL_LS_MONITORING_INTERRUPT);

    EALLOW;
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global real time interrupt DBGM
    EDIS;
}

/*******************************************************************************************************************//**
 * clearPWMTripFlags()
 *
 *    Called by IpVoltageAndOpCurrentMonitoring() when recovery from a Fault is in process to clear any existing PWM Trip conditions.
 *
 **********************************************************************************************************************/
inline void clearPWMTripFlags(uint32_t base)
{
    // clear all the configured trip sources for the PWM module
    EPWM_clearTripZoneFlag(base,
                           ( EPWM_TZ_INTERRUPT_OST |
                             EPWM_TZ_INTERRUPT_CBC |
                             EPWM_TZ_INTERRUPT_DCAEVT1 )
                           );
}


// TODO calibration for ADC data

#if ENABLE_CALIBRATE_OFFSET
inline void read_Voffset(volatile float *voffset)
{

    *voffset = ((float) (VOFFSET_FB + VOFFSET_FB_2)*0.5);

}
#endif

/*******************************************************************************************************************//**
 * read_sensed_values_HS()
 *
 * Input parameters:
 * volatile float * ip_cur_sensed   - pointer to location to store input current.
 * volatile float * ip_vol_sensed   - pointer to location to store input voltage.
 * volatile float temp_sensed[] - pointer to an array of floats used to store Module output currents.
 *
 *
 *    Called by IpVoltageAndOpCurrentMonitoring() to read the Module Output current and Input curr and voltage sampled A/D values.
 *    For each value there are 2 SOCs defined
 *    so that on every A/D conversion cycle we get 2 samples of each Analog value being converted. These two values are
 *    summed and averaged and scaled appropriately (0 - 4095).
 *    Note - parameters are volatile to insure compiler actually re-reads each of the sampled values.
 *
 **********************************************************************************************************************/
inline void read_sensed_values_HS(volatile float * ip_cur_sensed, volatile float ip_vol_sensed[], volatile float op_cur_sensed[])
{
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (commented 2 lines) //
//    *ip_cur_sensed = ((float)(IP_CUR_FB  + IP_CUR_FB_2)*0.5 *ADC_PU_SCALE_FACTOR);
//    *ip_vol_sensed = ((float)(IP_VOL_FB  + IP_VOL_FB_2)*0.5 *ADC_PU_SCALE_FACTOR);

    op_cur_sensed[0] = ((float)((OUT_CUR_1_FB  + OUT_CUR_1_FB_2)*0.5 *ADC_PU_SCALE_FACTOR + op_cur_sensed_offset0));
    ip_vol_sensed[0] = ((float)((IN_VOL_1_FB  + IN_VOL_1_FB_2)*0.5 *ADC_PU_SCALE_FACTOR));

#if (NUM_SUPPORTED_MODULES >= 2)
    op_cur_sensed[1] = ((float)((OUT_CUR_2_FB  + OUT_CUR_2_FB_2)*0.5 *ADC_PU_SCALE_FACTOR + op_cur_sensed_offset1));
    ip_vol_sensed[1] = ((float)((IN_VOL_2_FB  + IN_VOL_2_FB_2)*0.5 *ADC_PU_SCALE_FACTOR));
#endif

#if (NUM_SUPPORTED_MODULES >= 3)
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (delete ')' in statement) //
    op_cur_sensed[2] = ((float)((OUT_CUR_3_FB  + OUT_CUR_3_FB_2)*0.5 *ADC_PU_SCALE_FACTOR + op_cur_sensed_offset2));
    ip_vol_sensed[2] = ((float)((IN_VOL_3_FB  + IN_VOL_3_FB_2)*0.5 *ADC_PU_SCALE_FACTOR));
#endif
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 6 lines) //
#if (NUM_SUPPORTED_MODULES >= 4)
    op_cur_sensed[3] = ((float)((OUT_CUR_4_FB  + OUT_CUR_4_FB_2)*0.5 *ADC_PU_SCALE_FACTOR + op_cur_sensed_offset3));
    ip_vol_sensed[3] = ((float)((IN_VOL_4_FB  + IN_VOL_4_FB_2)*0.5 *ADC_PU_SCALE_FACTOR));
#endif

#if (NUM_SUPPORTED_MODULES >= 5)
    op_cur_sensed[4] = ((float)((OUT_CUR_5_FB  + OUT_CUR_5_FB_2)*0.5 *ADC_PU_SCALE_FACTOR + op_cur_sensed_offset4));
    ip_vol_sensed[4] = ((float)((IN_VOL_5_FB  + IN_VOL_5_FB_2)*0.5 *ADC_PU_SCALE_FACTOR));
#endif
}


/*******************************************************************************************************************//**
 * read_sensed_values_LS()
 *
 * Input parameters:
 * volatile float op_vol_sensed[] - pointer to an array of floats used to store Module output voltages.
 * volatile float temp_sensed[] - pointer to an array of floats used to store Module output temperatures.
 *
 *
 *    Called by OpVoltageAndTemp() to read the sampled A/D temperature values. For each value there
 *    are 2 SOCs defined so that on every A/D conversion cycle we get 2 samples of each Analog temperature value
 *    being converted. These two values are summed and averaged and scaled appropriately (0 - 4095).
 *    Note - parameters are volatile to insure compiler actually re-reads each of the sampled values.
 *
 **********************************************************************************************************************/
inline void read_sensed_values_LS(volatile float op_vol_sensed[], volatile float temp_sensed[])
{
    op_vol_sensed[0] =  ((float)(OUT_VOL_1_FB +  OUT_VOL_1_FB_2)*0.5*ADC_PU_SCALE_FACTOR);
    temp_sensed[0] = ((float)(TEMP_1_FB  + TEMP_1_FB_2)*0.5 * ADC_PU_SCALE_FACTOR);

#if (NUM_SUPPORTED_MODULES >= 2)
    op_vol_sensed[1] =  ((float)(OUT_VOL_2_FB +  OUT_VOL_2_FB_2)*0.5*ADC_PU_SCALE_FACTOR);
    temp_sensed[1] = ((float)(TEMP_2_FB  + TEMP_2_FB_2)*0.5 * ADC_PU_SCALE_FACTOR);
#endif

#if (NUM_SUPPORTED_MODULES >= 3)
    op_vol_sensed[2] =  ((float)(OUT_VOL_3_FB +  OUT_VOL_3_FB_2)*0.5*ADC_PU_SCALE_FACTOR);
    temp_sensed[2] = ((float)(TEMP_3_FB  + TEMP_3_FB_2)*0.5 * ADC_PU_SCALE_FACTOR);
#endif

    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 8 lines) //
#if (NUM_SUPPORTED_MODULES >= 4)
    op_vol_sensed[3] =  ((float)(OUT_VOL_4_FB +  OUT_VOL_4_FB_2)*0.5*ADC_PU_SCALE_FACTOR);
    temp_sensed[3] = ((float)(TEMP_4_FB  + TEMP_4_FB_2)*0.5 * ADC_PU_SCALE_FACTOR);
#endif

#if (NUM_SUPPORTED_MODULES >= 5)
    op_vol_sensed[4] =  ((float)(OUT_VOL_5_FB +  OUT_VOL_5_FB_2)*0.5*ADC_PU_SCALE_FACTOR);
    temp_sensed[4] = ((float)(TEMP_5_FB  + TEMP_5_FB_2)*0.5 * ADC_PU_SCALE_FACTOR);
#endif

}

#endif





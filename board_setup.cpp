/* ==============================================================================
System Name:    DC-DC converter

File Name:      converter_board.c

Target:         F28004x

Author:

Description:    This file consists of board related initialization,
                this file is used to make the main file more readable
===========================================================================  */


#include "converter_board.h"
#include "InterruptHandlers.hpp"


/*******************************************************************************************************************//**
 * setupDevice()
 *
 *    This function is the first function called by main() during the startup process and carries out the following:
 *    - Disables the watchdog.
 *    - If running from Flash, copies time critical code and flash setup code to RAM and calls Flash Initialization
 *    - to setup flash wait states.
 *    - Set up PLL control and clock dividers for setting up System clock.
 *    - Set the system clock to run off the internal oscillator.
 *    - Turn on all peripherals.
 *    - Disables pin locks and enable internal pull-ups.
 *    - Initializes PIE and clears PIE registers. Disables CPU interrupts.
 *    - Initializes the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
 *    - Configures the CPU timer to run at 10KHz.
 *
 **********************************************************************************************************************/
void setupDevice(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // initialize CPU timers
    //

    // Initialize timer period to maximum
    CPUTimer_setPeriod(CPUTIMER2_BASE, DEVICE_SYSCLK_FREQ/10000 ); // 10KHz

    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);

    // Make sure timer is stopped
    CPUTimer_stopTimer(CPUTIMER2_BASE);

    CPUTimer_setEmulationMode(CPUTIMER2_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
     // Reload all counter register with period value
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);
    CPUTimer_resumeTimer(CPUTIMER2_BASE);

}



/*******************************************************************************************************************//**
 * setupADC()
 *
 *    Setup the ADC modules:
 *    - Specify a 3.3V reference voltage.
 *    - ADC clock is set to 50 MHz and the ADC module is powered up.
 *    - Configure the SOC (Start of conversion) for each of the Analog inputs to be sampled. Each SOC
 *      uses ADC_TRIGGER_EPWM5_SOCA as it's start of conversion trigger as EPWM5 has been configured to generate
 *      this signal when it's decrementing Time-base counter is equal to CMPB.
 *
 **********************************************************************************************************************/
void setupADC(void)
{

    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    //set ADC clock as 50MHz
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_2_0);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_2_0);


    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    ADC_enableConverter(ADCC_BASE);

    DEVICE_DELAY_US(1000);


    /*--------------------setup ADC conversions for input analog signals---------------*/
    // Parameter description of function ADC_setupSOC:
    // First: Base of ADC module
    // Second: SOC Number of ADC module
    // Third: Trigger source for ADC to trigger start-of-conversion
    // Fourth: Channel number of ADC module
    // Fifth: Time for Sampling the input sample at ADC input Pin


    // MODULE OUTPUT CURRENT
    //SOC settings :OUTPUT CURRENT- DC MODULE-->1
    ADC_setupSOC(OUT_CUR_1_ADC_MODULE, OUT_CUR_1_ADC_SOC_NO1, OUT_CUR_1_ADC_TRIG_SOURCE,
                 OUT_CUR_1_ADC_PIN, OUT_CUR_1_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_CUR_1_ADC_MODULE, OUT_CUR_1_ADC_SOC_NO2, OUT_CUR_1_ADC_TRIG_SOURCE,
                 OUT_CUR_1_ADC_PIN, OUT_CUR_1_ADC_ACQPS_SYS_CLKS);
    //SOC settings :OUTPUT CURRENT- DC MODULE-->2
    ADC_setupSOC(OUT_CUR_2_ADC_MODULE, OUT_CUR_2_ADC_SOC_NO1, OUT_CUR_2_ADC_TRIG_SOURCE,
                 OUT_CUR_2_ADC_PIN, OUT_CUR_2_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_CUR_2_ADC_MODULE, OUT_CUR_2_ADC_SOC_NO2, OUT_CUR_2_ADC_TRIG_SOURCE,
                 OUT_CUR_2_ADC_PIN, OUT_CUR_2_ADC_ACQPS_SYS_CLKS);
    //SOC settings :OUTPUT CURRENT- DC MODULE-->3
    ADC_setupSOC(OUT_CUR_3_ADC_MODULE, OUT_CUR_3_ADC_SOC_NO1, OUT_CUR_3_ADC_TRIG_SOURCE,
                 OUT_CUR_3_ADC_PIN, OUT_CUR_3_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_CUR_3_ADC_MODULE, OUT_CUR_3_ADC_SOC_NO2, OUT_CUR_3_ADC_TRIG_SOURCE,
                 OUT_CUR_3_ADC_PIN, OUT_CUR_3_ADC_ACQPS_SYS_CLKS);
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 10 lines) //
    //SOC settings :OUTPUT CURRENT- DC MODULE-->4
    ADC_setupSOC(OUT_CUR_4_ADC_MODULE, OUT_CUR_4_ADC_SOC_NO1, OUT_CUR_4_ADC_TRIG_SOURCE,
                 OUT_CUR_4_ADC_PIN, OUT_CUR_4_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_CUR_4_ADC_MODULE, OUT_CUR_4_ADC_SOC_NO2, OUT_CUR_4_ADC_TRIG_SOURCE,
                 OUT_CUR_4_ADC_PIN, OUT_CUR_4_ADC_ACQPS_SYS_CLKS);
    //SOC settings :OUTPUT CURRENT- DC MODULE-->5
    ADC_setupSOC(OUT_CUR_5_ADC_MODULE, OUT_CUR_5_ADC_SOC_NO1, OUT_CUR_5_ADC_TRIG_SOURCE,
                 OUT_CUR_5_ADC_PIN, OUT_CUR_5_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_CUR_5_ADC_MODULE, OUT_CUR_5_ADC_SOC_NO2, OUT_CUR_5_ADC_TRIG_SOURCE,
                 OUT_CUR_5_ADC_PIN, OUT_CUR_5_ADC_ACQPS_SYS_CLKS);


    // MODULE OUTPUT VOLTAGE
    //SOC settings :OUTPUT_VOLTAGE - DC MODULE-->1
    ADC_setupSOC(OUT_VOL_1_ADC_MODULE, OUT_VOL_1_ADC_SOC_NO1, OUT_VOL_1_ADC_TRIG_SOURCE,
                 OUT_VOL_1_ADC_PIN, OUT_VOL_1_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_VOL_1_ADC_MODULE, OUT_VOL_1_ADC_SOC_NO2, OUT_VOL_1_ADC_TRIG_SOURCE,
                 OUT_VOL_1_ADC_PIN, OUT_VOL_1_ADC_ACQPS_SYS_CLKS);
    //SOC settings :OUTPUT_VOLTAGE - DC MODULE-->2
    ADC_setupSOC(OUT_VOL_2_ADC_MODULE, OUT_VOL_2_ADC_SOC_NO1, OUT_VOL_2_ADC_TRIG_SOURCE,
                 OUT_VOL_2_ADC_PIN, OUT_VOL_2_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_VOL_2_ADC_MODULE, OUT_VOL_2_ADC_SOC_NO2, OUT_VOL_2_ADC_TRIG_SOURCE,
                 OUT_VOL_2_ADC_PIN, OUT_VOL_2_ADC_ACQPS_SYS_CLKS);
    //SOC settings :OUTPUT_VOLTAGE - DC MODULE-->3
    ADC_setupSOC(OUT_VOL_3_ADC_MODULE, OUT_VOL_3_ADC_SOC_NO1, OUT_VOL_3_ADC_TRIG_SOURCE,
                 OUT_VOL_3_ADC_PIN, OUT_VOL_3_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_VOL_3_ADC_MODULE, OUT_VOL_3_ADC_SOC_NO2, OUT_VOL_3_ADC_TRIG_SOURCE,
                 OUT_VOL_3_ADC_PIN, OUT_VOL_3_ADC_ACQPS_SYS_CLKS);
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 10 lines) //
    //SOC settings :OUTPUT_VOLTAGE - DC MODULE-->4
    ADC_setupSOC(OUT_VOL_4_ADC_MODULE, OUT_VOL_4_ADC_SOC_NO1, OUT_VOL_4_ADC_TRIG_SOURCE,
                 OUT_VOL_4_ADC_PIN, OUT_VOL_4_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_VOL_4_ADC_MODULE, OUT_VOL_4_ADC_SOC_NO2, OUT_VOL_4_ADC_TRIG_SOURCE,
                 OUT_VOL_4_ADC_PIN, OUT_VOL_4_ADC_ACQPS_SYS_CLKS);
    //SOC settings :OUTPUT_VOLTAGE - DC MODULE-->5
    ADC_setupSOC(OUT_VOL_5_ADC_MODULE, OUT_VOL_5_ADC_SOC_NO1, OUT_VOL_5_ADC_TRIG_SOURCE,
                 OUT_VOL_5_ADC_PIN, OUT_VOL_5_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(OUT_VOL_5_ADC_MODULE, OUT_VOL_5_ADC_SOC_NO2, OUT_VOL_5_ADC_TRIG_SOURCE,
                 OUT_VOL_5_ADC_PIN, OUT_VOL_5_ADC_ACQPS_SYS_CLKS);


    // MODULE TEMPERATURE
    //SOC settings :Temperature Sensing - DC MODULE-->1
    ADC_setupSOC(TEMP_1_ADC_MODULE,TEMP_1_ADC_SOC_NO1,TEMP_1_ADC_TRIG_SOURCE,
                 TEMP_1_ADC_PIN,TEMP_1_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(TEMP_1_ADC_MODULE,TEMP_1_ADC_SOC_NO2,TEMP_1_ADC_TRIG_SOURCE,
                 TEMP_1_ADC_PIN,TEMP_1_ADC_ACQPS_SYS_CLKS);
    //SOC settings :Temperature Sensing - DC MODULE-->2
    ADC_setupSOC(TEMP_2_ADC_MODULE,TEMP_2_ADC_SOC_NO1,TEMP_2_ADC_TRIG_SOURCE,
                 TEMP_2_ADC_PIN,TEMP_2_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(TEMP_2_ADC_MODULE,TEMP_2_ADC_SOC_NO2,TEMP_2_ADC_TRIG_SOURCE,
                 TEMP_2_ADC_PIN,TEMP_2_ADC_ACQPS_SYS_CLKS);
    //SOC settings :Temperature Sensing - DC MODULE-->3
    ADC_setupSOC(TEMP_3_ADC_MODULE,TEMP_3_ADC_SOC_NO1,TEMP_3_ADC_TRIG_SOURCE,
                 TEMP_3_ADC_PIN,TEMP_3_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(TEMP_3_ADC_MODULE,TEMP_3_ADC_SOC_NO2,TEMP_3_ADC_TRIG_SOURCE,
                 TEMP_3_ADC_PIN,TEMP_3_ADC_ACQPS_SYS_CLKS);
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 10 lines) //
    //SOC settings :Temperature Sensing - DC MODULE-->4
    ADC_setupSOC(TEMP_4_ADC_MODULE,TEMP_4_ADC_SOC_NO1,TEMP_4_ADC_TRIG_SOURCE,
                 TEMP_4_ADC_PIN,TEMP_4_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(TEMP_4_ADC_MODULE,TEMP_4_ADC_SOC_NO2,TEMP_4_ADC_TRIG_SOURCE,
                 TEMP_4_ADC_PIN,TEMP_4_ADC_ACQPS_SYS_CLKS);
    //SOC settings :Temperature Sensing - DC MODULE-->5
    ADC_setupSOC(TEMP_5_ADC_MODULE,TEMP_5_ADC_SOC_NO1,TEMP_5_ADC_TRIG_SOURCE,
                 TEMP_5_ADC_PIN,TEMP_5_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(TEMP_5_ADC_MODULE,TEMP_5_ADC_SOC_NO2,TEMP_5_ADC_TRIG_SOURCE,
                 TEMP_5_ADC_PIN,TEMP_5_ADC_ACQPS_SYS_CLKS);

    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 20 lines) //
    // MODULE INPUT VOLTAGE
    //SOC settings :INPUT_VOLTAGE - DC MODULE-->1
    ADC_setupSOC(IN_VOL_1_ADC_MODULE, IN_VOL_1_ADC_SOC_NO1, IN_VOL_1_ADC_TRIG_SOURCE,
                 IN_VOL_1_ADC_PIN, IN_VOL_1_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(IN_VOL_1_ADC_MODULE, IN_VOL_1_ADC_SOC_NO2, IN_VOL_1_ADC_TRIG_SOURCE,
                 IN_VOL_1_ADC_PIN, IN_VOL_1_ADC_ACQPS_SYS_CLKS);
    //SOC settings :INPUT_VOLTAGE - DC MODULE-->2
    ADC_setupSOC(IN_VOL_2_ADC_MODULE, IN_VOL_2_ADC_SOC_NO1, IN_VOL_2_ADC_TRIG_SOURCE,
                 IN_VOL_2_ADC_PIN, IN_VOL_2_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(IN_VOL_2_ADC_MODULE, IN_VOL_2_ADC_SOC_NO2, IN_VOL_2_ADC_TRIG_SOURCE,
                 IN_VOL_2_ADC_PIN,IN_VOL_2_ADC_ACQPS_SYS_CLKS);
    //SOC settings :INPUT_VOLTAGE - DC MODULE-->3
    ADC_setupSOC(IN_VOL_3_ADC_MODULE, IN_VOL_3_ADC_SOC_NO1, IN_VOL_3_ADC_TRIG_SOURCE,
                 IN_VOL_3_ADC_PIN, IN_VOL_3_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(IN_VOL_3_ADC_MODULE, IN_VOL_3_ADC_SOC_NO2, IN_VOL_3_ADC_TRIG_SOURCE,
                 IN_VOL_3_ADC_PIN, IN_VOL_3_ADC_ACQPS_SYS_CLKS);
    //SOC settings :INPUT_VOLTAGE - DC MODULE-->4
    ADC_setupSOC(IN_VOL_4_ADC_MODULE, IN_VOL_4_ADC_SOC_NO1, IN_VOL_4_ADC_TRIG_SOURCE,
                 IN_VOL_4_ADC_PIN, IN_VOL_4_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(IN_VOL_4_ADC_MODULE, IN_VOL_4_ADC_SOC_NO2, IN_VOL_4_ADC_TRIG_SOURCE,
                 IN_VOL_4_ADC_PIN, IN_VOL_4_ADC_ACQPS_SYS_CLKS);
    //SOC settings :INPUT_VOLTAGE - DC MODULE-->5
    ADC_setupSOC(IN_VOL_5_ADC_MODULE, IN_VOL_5_ADC_SOC_NO1, IN_VOL_5_ADC_TRIG_SOURCE,
                 IN_VOL_5_ADC_PIN, IN_VOL_5_ADC_ACQPS_SYS_CLKS);
    ADC_setupSOC(IN_VOL_5_ADC_MODULE, IN_VOL_5_ADC_SOC_NO2, IN_VOL_5_ADC_TRIG_SOURCE,
                 IN_VOL_5_ADC_PIN, IN_VOL_5_ADC_ACQPS_SYS_CLKS);
}



/*******************************************************************************************************************//**
 * setupEPWM5toTriggerADCSOC()
 *
 *    Configures PWM5 to generate a trigger event for the ADC start of conversions when CMPB == 25 on count Down.
 *
 **********************************************************************************************************************/
void setupEPWM5toTriggerADCSOC()
{
    // This function sets the ePWM ADC SOC source. Select SOC A when Time-base counter equal to
    // CMPB when the timer is decrementing
    EPWM_setADCTriggerSource(DC_CONVERTER_PWM_CMD1,EPWM_SOC_A,EPWM_SOC_TBCTR_D_CMPB );

    // Generate SOC event when CMPB == 25 on count Down.
    EPWM_setCounterCompareValue(DC_CONVERTER_PWM_CMD1,EPWM_COUNTER_COMPARE_B, 25);

    // Generate pulse on 1st even
    EPWM_setADCTriggerEventPrescale(DC_CONVERTER_PWM_CMD1,EPWM_SOC_A,1);

    // Enable SOC on A group
    EPWM_enableADCTrigger(DC_CONVERTER_PWM_CMD1,EPWM_SOC_A);
}


/*******************************************************************************************************************//**
 * disablePWMCLKCounting()
 *
 *    Disables the PWM clock
 *
 **********************************************************************************************************************/
void disablePWMCLKCounting(void)
{
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

/*******************************************************************************************************************//**
 * enablePWMCLKCounting()
 *
 *    Enables the PWM clock
 *
 **********************************************************************************************************************/
void enablePWMCLKCounting(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}


/*******************************************************************************************************************//**
 * setupDC_DCconverterPWM()
 *
 * Input parameters:
 *  uint32_t base1 - base address of PWM5. (Module 1)
 *  uint32_t base2 - base address of PWM2. (Module 2)
 *  uint32_t base3 - base address of PWM3. (Module 3)
 *  uint32_t base4 - base address of PWM1. (Module 4)
 *  uint32_t base5 - base address of PWM4. (Module 5)
 *  uint32_t base6 - base address of PWM6. (HS loop interrupt)
 *
 *    Configure each of the supplied PWM modules as required for the DC-DC Converter implementation.
 *    Currently a maximum of 3 Modules are supported and hence 3 Module specific PWMs. If there are < 3
 *    Modules configured (via NUM_SUPPORTED_MODULES), then only the enabled Module PWMs are configured.
 *
 *    PWM 5:
 *    - Configured as the master and generates the syncout pulse to PWM2, PWM3 and PWM4.
 *    PWMs 5, 2, 3, 1 and 4
 *    - Use a 100 MHz clock.
 *    - Have an up-down timer period of 500, which results in a PWM switching frequency of 100kHz.
 *    - Are all in the same phase.
 *    PWM 6:
 *    - Uses a 100 MHz clock.
 *    - Is used to generate the 100 kHz interrupt for the HighSpeedMonitoring() function. It does not generate a switching frequency
 *      output used by the DC-DC Controller.
 *    - Has a two count phase shift from the other 3 PWMs.
 *
 **********************************************************************************************************************/
void setupDC_DCconverterPWM(uint32_t base1, uint32_t base2, uint32_t base3, uint32_t base4, uint32_t base5, uint32_t base6, uint16_t pwm_period_ticks)
{

    // Module 1 PWM Setup
    // Shadow load mode
    EPWM_setPeriodLoadMode(base1,EPWM_PERIOD_SHADOW_LOAD);

    // Timer period for up-down count = (sys_clk_freq/(PWM_clk_freq*2)
    EPWM_setTimeBasePeriod(base1,pwm_period_ticks >>1);
    EPWM_setTimeBaseCounter(base1,0);
    EPWM_setPhaseShift(base1,0);

    // Counter mode for triangle: Up-Down count
    EPWM_setTimeBaseCounterMode(base1,EPWM_COUNTER_MODE_UP_DOWN);

    // EPWM module clock frequency = 100MHz
    EPWM_setClockPrescaler(base1,EPWM_CLOCK_DIVIDER_1,EPWM_HSCLOCK_DIVIDER_1);

    // set duty 0% initially
    EPWM_setCounterCompareValue(base1,EPWM_COUNTER_COMPARE_A,0);
    EPWM_setCounterCompareShadowLoadMode(base1,EPWM_COUNTER_COMPARE_A,EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // ePWMxA output goes low when Time base counter up equals COMPA
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
                                  EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    // ePWMxA output goes high when Time base counter down equals COMPA
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
                                  EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    // configure PWM 5 as master and PWM5 generates syncout pulse to EPWM2, EPWM3 and EPWM4.
    // EPWM5,2,3 are in same phase- current command PWMs for DC modules 1,2,3
    // EPWM4 is 2 counts leading from other PWMs and used for Control ISR event

    EPWM_disablePhaseShiftLoad(base1);
    EPWM_setSyncOutPulseMode(base1,EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (replace base4 to base6) //
    // control ISR PWM setup
    // Shadow load mode
    EPWM_setPeriodLoadMode(base6,EPWM_PERIOD_SHADOW_LOAD);
    // Timer period for up-down count = (sys_clk_freq/(PWM_clk_freq*2)

    EPWM_setTimeBasePeriod(base6,pwm_period_ticks >>1);
    EPWM_setTimeBaseCounter(base6,0);
    EPWM_setPhaseShift(base6,0);

    // Counter mode for triangle: Up-Down count
    EPWM_setTimeBaseCounterMode(base6,EPWM_COUNTER_MODE_UP_DOWN);

    // EPWM module clock frequency = 100MHz
    EPWM_setClockPrescaler(base6,EPWM_CLOCK_DIVIDER_1,EPWM_HSCLOCK_DIVIDER_1);


    // Set COMPB to generate Interrupt to trigger control ISR
    EPWM_setCounterCompareValue(base6,EPWM_COUNTER_COMPARE_B,25);
    EPWM_setCounterCompareShadowLoadMode(base6,EPWM_COUNTER_COMPARE_B,EPWM_COMP_LOAD_ON_CNTR_ZERO);


    EPWM_enablePhaseShiftLoad(base6);
    EPWM_setPhaseShift(base6,2);
    EPWM_setCountModeAfterSync(base6, EPWM_COUNT_MODE_UP_AFTER_SYNC);
}

/*******************************************************************************************************************//**
 * setPinsAsPWM()
 *
 *    Set GPIO16 as PWM5A, GPIO2 as PWM2A and GPIO4 as PWM3A.
 *    PWM4 Is used to generate the 100 kHz interrupt for the HighSpeedMonitoring() function and does not generate an
 *    output used by the DC-DC Controller.
 *
 **********************************************************************************************************************/
void setPinsAsPWM()
{
    GPIO_writePin(DC_CONVERTER_CMD1_H_GPIO,0);
    GPIO_setDirectionMode(DC_CONVERTER_CMD1_H_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_CONVERTER_CMD1_H_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_CONVERTER_CMD1_H_GPIO_PIN_CONFIG );

    GPIO_writePin(DC_CONVERTER_CMD2_H_GPIO,0);
    GPIO_setDirectionMode(DC_CONVERTER_CMD2_H_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_CONVERTER_CMD2_H_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_CONVERTER_CMD2_H_GPIO_PIN_CONFIG );

    GPIO_writePin(DC_CONVERTER_CMD3_H_GPIO,0);
    GPIO_setDirectionMode(DC_CONVERTER_CMD3_H_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_CONVERTER_CMD3_H_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_CONVERTER_CMD3_H_GPIO_PIN_CONFIG );
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 8 lines) //
    GPIO_writePin(DC_CONVERTER_CMD4_H_GPIO,0);
    GPIO_setDirectionMode(DC_CONVERTER_CMD4_H_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_CONVERTER_CMD4_H_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_CONVERTER_CMD4_H_GPIO_PIN_CONFIG );

    GPIO_writePin(DC_CONVERTER_CMD5_H_GPIO,0);
    GPIO_setDirectionMode(DC_CONVERTER_CMD5_H_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_CONVERTER_CMD5_H_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_CONVERTER_CMD5_H_GPIO_PIN_CONFIG );
}


/*******************************************************************************************************************//**
 * setupProfilingGPIO()
 *
 *    Configure the Profiling pins as outputs. By using these pins and their respective setProfilingGPIOx() and
 *    resetProfilingGPIOx() functions, a scope will show how much time is being spent in the ISRs and consequently
 *    how much load is being placed on the CPU.
 *
 **********************************************************************************************************************/
void setupProfilingGPIO(void)
{
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (disable settings for GPIO profiling1) //
    // No functions for GPIO profiling
    // Pin for PROFILING1 is originally desinged for Icmd4
    // Pin for PROFILING2 is desinged as TestPoint
//    GPIO_setDirectionMode(GPIO_PROFILING1,GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(GPIO_PROFILING2,GPIO_DIR_MODE_OUT);
//    GPIO_setQualificationMode(GPIO_PROFILING1,GPIO_QUAL_SYNC);
    GPIO_setQualificationMode(GPIO_PROFILING2,GPIO_QUAL_SYNC);
//    GPIO_setPinConfig(GPIO_PROFILING1_PIN_CONFIG);
    GPIO_setPinConfig(GPIO_PROFILING2_PIN_CONFIG);

}

/*******************************************************************************************************************//**
 * setDCModuleEnablePins()
 *
 *    Configure the Module Enable pins as Outputs. These pins are used to enable/disable an upstream Module.
 *
 **********************************************************************************************************************/
void setDCModuleEnablePins()
{
    // DC Module 1 Enable pin comes up by default in analog mode. We must switch it to digital mode.
    // Note: GPIO22 and GPIO23 are in a special analog mode at reset, and must be reconfigured for
    //       GPIO use by clearing their bits in GPAAMSEL. GPIO23's maximum toggle frequency is
    //       limited. The exact limit is TBD, but estimated to be 12 MHz.
    GPIO_writePin(DC_Module1_En_GPIO,0);
    GPIO_setDirectionMode(DC_Module1_En_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_Module1_En_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module1_En_GPIO_PIN_CONFIG);
    GPIO_setAnalogMode(DC_Module1_En_GPIO, GPIO_ANALOG_DISABLED);

    GPIO_writePin(DC_Module2_En_GPIO,0);
    GPIO_setDirectionMode(DC_Module2_En_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_Module2_En_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module2_En_GPIO_PIN_CONFIG);

    GPIO_writePin(DC_Module3_En_GPIO,0);
    GPIO_setDirectionMode(DC_Module3_En_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_Module3_En_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module3_En_GPIO_PIN_CONFIG);
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 8 lines) //
    GPIO_writePin(DC_Module4_En_GPIO,0);
    GPIO_setDirectionMode(DC_Module4_En_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_Module4_En_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module4_En_GPIO_PIN_CONFIG);

    GPIO_writePin(DC_Module5_En_GPIO,0);
    GPIO_setDirectionMode(DC_Module5_En_GPIO ,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DC_Module5_En_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module5_En_GPIO_PIN_CONFIG);

}


/*******************************************************************************************************************//**
 * setupCMPSS()
 *
 * Input parameters:
 *  uint32_t base1          - Base address of the comparator to configure.
 *  float current_limit     - Current above which a Trip event should occur.
 *  float current_max_sense - Max sensed current for this Module.
 *
 *    Called by setupBoardProtection() to configure the Comparator for each of the Modules/PWMs such that if the
 *    Module Output current exceeds the supplied current limit, the Comparator will generate a CTRIPH signal to
 *    the respective PWM, which will immediately disable that PWM.
 *
 **********************************************************************************************************************/
void setupCMPSS(uint32_t base1, float current_limit, float current_max_sense )
{
    //Enable CMPSS1
    CMPSS_enableModule(base1);

    //Use VDDA as the reference for comparator DACs
    CMPSS_configDAC(base1,
                    CMPSS_DACVAL_SYSCLK|CMPSS_DACREF_VDDA|CMPSS_DACSRC_SHDW);

    //Set DAC Hval with trip limit
    CMPSS_setDACValueHigh(base1,

                          (int16_t)((float)current_limit*(float)4096.0/
                                  (float)current_max_sense));

    // Make sure the asynchronous path of compare high
    // does not go to the OR gate with latched digital filter output
    // hence no additional parameter CMPSS_OR_ASYNC_OUT_W_FILT  is passed
    // comparator output is "not" inverted for high compare event
    CMPSS_configHighComparator(base1, CMPSS_INSRC_DAC );

    CMPSS_configFilterHigh(base1,CMPSS_FILTER_SAMPLE_PRESCALE,
                           CMPSS_FILTER_SAMPLE_WINDOW,CMPSS_FILTER_THRESHOLD );


    //Reset filter logic & start filtering
    CMPSS_initFilterHigh(base1);

    // Configure CTRIPOUT path
    CMPSS_configOutputsHigh(base1, CMPSS_TRIP_FILTER|CMPSS_TRIP_FILTER);

    //Comparator hysteresis control , set to 2x typical value
    CMPSS_setHysteresis(base1, 2);

    // Clear the latched comparator events
    CMPSS_clearFilterLatchHigh(base1);
}

// ---------------------------------------------------------------------------------------------------- BK: Jan/15/2019 ----- (add new function) //
/*******************************************************************************************************************//**
 * setupCMPSS_L()
 *
 * Input parameters:
 *  uint32_t base1          - Base address of the comparator to configure.
 *  float current_limit     - Current above which a Trip event should occur.
 *  float current_max_sense - Max sensed current for this Module.
 *
 *    Called by setupBoardProtection() to configure the Comparator for each of the Modules/PWMs such that if the
 *    Module Output current exceeds the supplied current limit, the Comparator will generate a CTRIPH signal to
 *    the respective PWM, which will immediately disable that PWM.
 *
 **********************************************************************************************************************/
void setupCMPSS_L(uint32_t base3, float current_limit, float current_max_sense )
{
    //Enable CMPSS
    CMPSS_enableModule(base3);

    //Use VDDA as the reference for comparator DACs
    CMPSS_configDAC(base3,
                    CMPSS_DACVAL_SYSCLK|CMPSS_DACREF_VDDA|CMPSS_DACSRC_SHDW);

    //Set DAC Hval with trip limit
    CMPSS_setDACValueLow(base3,

                          (int16_t)((float)current_limit*(float)4096.0/
                                  (float)current_max_sense));

    // Make sure the asynchronous path of compare high
    // does not go to the OR gate with latched digital filter output
    // hence no additional parameter CMPSS_OR_ASYNC_OUT_W_FILT  is passed
    // comparator output is "not" inverted for high compare event
    CMPSS_configLowComparator(base3, CMPSS_INSRC_DAC );

    CMPSS_configFilterLow(base3,CMPSS_FILTER_SAMPLE_PRESCALE,
                           CMPSS_FILTER_SAMPLE_WINDOW,CMPSS_FILTER_THRESHOLD );


    //Reset filter logic & start filtering
    CMPSS_initFilterLow(base3);

    // Configure CTRIPOUT path
    CMPSS_configOutputsLow(base3, CMPSS_TRIP_FILTER|CMPSS_TRIP_FILTER);

    //Comparator hysteresis control , set to 2x typical value
    CMPSS_setHysteresis(base3, 2);

    // Clear the latched comparator events
    CMPSS_clearFilterLatchLow(base3);
}



// ---------------------------------------------------------------------------------------------------- BK: Jan/16/2019 ----- (add function) //
void setInputPROTPins(void){

    // DIS_REV_PROT_MCU pin part
    GPIO_setDirectionMode(DC_Module1_POLARITY_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module1_POLARITY_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD); // GPIO_PIN_TYPE_STD: Push-pull output or floating input (gpio.h)
    GPIO_setPinConfig(DC_Module1_POLARITY_FAULT_IN_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(DC_Module2_POLARITY_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module2_POLARITY_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module2_POLARITY_FAULT_IN_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(DC_Module3_POLARITY_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module3_POLARITY_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module3_POLARITY_FAULT_IN_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(DC_Module4_POLARITY_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module4_POLARITY_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module4_POLARITY_FAULT_IN_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(DC_Module5_POLARITY_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module5_POLARITY_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module5_POLARITY_FAULT_IN_GPIO_PIN_CONFIG);

    // OCP_MCU pin part
    GPIO_setDirectionMode(DC_Module1_OC_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module1_OC_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module1_OC_FAULT_IN_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(DC_Module2_OC_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module2_OC_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module2_OC_FAULT_IN_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(DC_Module3_OC_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module3_OC_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module3_OC_FAULT_IN_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(DC_Module4_OC_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module4_OC_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module4_OC_FAULT_IN_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(DC_Module5_OC_FAULT_IN_GPIO ,GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DC_Module5_OC_FAULT_IN_GPIO ,GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(DC_Module5_OC_FAULT_IN_GPIO_PIN_CONFIG);
}

/*******************************************************************************************************************//**
 * EPWM_setCounterCompareValueOptimized()
 *
 * Input parameters:
 *  uint32_t base                        - base address of the PWM to update.
 *  EPWM_CounterCompareModule compModule - The compare module in use (A or B).
 *  uint16_t compCount                   - The new value to be compared against.
 *
 *    Called by ControlAndLowSpeedMonitoring() to update the Duty Cycle for each PWM based on the feedback values
 *    collected.
 *
 **********************************************************************************************************************/
void EPWM_setCounterCompareValueOptimized(uint32_t base,
                                                        EPWM_CounterCompareModule compModule,
                                                        uint16_t compCount)
{
    uint32_t registerOffset;

    // Get the register offset for the Counter compare
    registerOffset = EPWM_O_CMPA + (uint16_t)compModule;
    // Write to the counter compare registers.

    if((compModule == EPWM_COUNTER_COMPARE_A) ||
            (compModule == EPWM_COUNTER_COMPARE_B))
    {
        // write to CMPA or COMPB bits
        HWREGH(base + registerOffset + 1) = (uint16_t)compCount;
    }
    else
    { // write to COMPC or COMPD bits
        HWREGH(base + registerOffset) = compCount;
    }

}

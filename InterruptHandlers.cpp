
/* ============================================================================
System Name:    DC-DC converter

File Name:      InterruptHandlers.cpp

Target:         F28004x

Author:

Description:    Interrupt Handlers for the DC-DC Converter project.

===========================================================================  */


//*****************************************************************************
// the includes
//*****************************************************************************

#include <InterruptHandlers.hpp>
#ifdef _DEBUG_ROM
#include "stdio.h"
#include "string.h"
#endif

// ---------------------------------------------------------------------------------------------------- BK: Feb/05/2019 ----- (add variables) //
int OUTPUT_OC_PROTECTION_ENABLE = 0;
int OUTPUT_OC_PROTECTION_LOOP_ENABLE = 0;
int OUTPUT_OV_PROTECTION_ENABLE = 0;
int TEMP_OT_PROTECTION_ENABLE = 0;
int INPUT_UV_PROTECTION_ENABLE = 0;
int INPUT_OV_PROTECTION_ENABLE = 0;
int REVERSE_PROTECTION_ENABLE = 0;
int boardStatus_Bit = 0x0;

int INPUT_OC_PROTECTION_ENABLE = 0; // only use as flag, not protection

volatile float ip_cur_sensed;
// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (change variable to array) //
volatile float ip_vol_sensed[MAX_SUPPORTED_MODULES];

// The following could be changed to [NUM_SUPPORTED_MODULES], if the code for debug/verification of the individual module readings is removed (see below comment)
// With is set to MAX_SUPPORTED_MODULES, we don't have to worry about read/writing unallocated memory.
volatile float  temp_sensed[MAX_SUPPORTED_MODULES];
volatile float  op_cur_sensed[MAX_SUPPORTED_MODULES];
volatile float  op_vol_sensed[MAX_SUPPORTED_MODULES];


// Number of consecutive Fault retries.
volatile uint16_t numFaultRetries = 0;
volatile bool fault_recover_inProgress = FALSE;
volatile uint32_t fault_recover_countdown = 0;
volatile float m_DC_command;


volatile float offset_vol_sensed;
volatile int16_t avgCtrlWindowBuffIndex = 0;
volatile bool controlflag = FALSE;
volatile float vOut_AVGFiltered = 0;
volatile float iIn_AVGFiltered = 0;
volatile float vIn_AVGFiltered = 0;
volatile float iOut_AVGFiltered = 0;
volatile float vOut_sensedFiltered_avg = 0;
volatile float avg_Ctrlwindow_size_factor = (float)(1.0/AVG_CTRL_WINDOW_SIZE);
volatile float current_command_duty = 0;
volatile float m_debug_command_duty =25;

volatile float m_CURRENT_COMMAND_SET_VALUE =20.0;
//volatile float m_CURRENT_COMMAND_SET_VALUE_Slope = (0.4/70.0);
//volatile float m_CURRENT_COMMAND_SET_VALUE_INIT = (10.0/70.0);
// ------JL: Feb/11/2019, coefficiency updated with experiment test results
volatile float m_CURRENT_COMMAND_SET_VALUE_Slope = (0.3881/70.0);
volatile float m_CURRENT_COMMAND_SET_VALUE_INIT = (9.6627/70.0);
volatile float CURRENT_COMMAND_SET_FACTOR = 0.0;



volatile float  PI_KP = 0.01;
volatile float  PI_KI = 0.0;

volatile int16_t ENABLE_USER_DEFINED_CURRENT = 1;
volatile int16_t DUTY_CYCLE_DEBUG = 0;         // Use a specific duty cycle set in converter.h
volatile float m_Iout_DutyCycle =10.0;

volatile float current_command_debug = 0.01;
volatile float current_command = 0.01;
volatile float op_vol_sensed_avg = 0;
volatile float op_cur_sensed_avg = 0;
volatile float ip_vol_sensed_avg = 0;
volatile float temp_sensed_avg = 0;


// Adding individual module averages. These are here really only for debug/verification of the individual module readings.
// They could ultimately be removed, as the xxx_AVGBuff[] are really the ones used for calculations.
volatile float mod1_op_cur_sensed_avg = 0;

volatile float mod1_iOut_AVGBuff[AVG_CTRL_WINDOW_SIZE];

volatile float mod1_iOutmeasured = 0;

volatile float mod1_op_temp_sensed_avg = 0;

//-------------------------------------------------------------------------------------------------------JL:  Jan/23/2019 ----- (add 6 lines)//
volatile float op_temp = 0;
volatile float mod1_op_temp = 0;


volatile float mod1_temp_AVGBuff[AVG_MON_WINDOW_SIZE];

volatile float mod1_Tempmeasured = 0;

volatile float mod1_op_vOut_sensed_avg = 0;

volatile float mod1_vOut_AVGBuff[AVG_MON_WINDOW_SIZE];

volatile float mod1_vOutmeasured = 0;

// ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 15 lines) //
volatile float mod1_ip_vIn_sensed_avg = 0;

volatile float mod1_vIn_AVGBuff[AVG_MON_WINDOW_SIZE];

volatile float mod1_vInmeasured = 0;

// Instrumentation variables
    // These variables are visible only to this ISR and initialized here as well.
volatile int16_t avgMonWindowBuffIndex = 0;
volatile bool monitorflag = FALSE;
volatile float temp_AVGFiltered = 0;
volatile float avg_Monwindow_size_factor = (float)(1.0/AVG_MON_WINDOW_SIZE);
//control loop variables and fast monitored values
volatile float iIn_AVGBuff[AVG_CTRL_WINDOW_SIZE];
volatile float vIn_AVGBuff[AVG_CTRL_WINDOW_SIZE];
volatile float iOut_AVGBuff[AVG_CTRL_WINDOW_SIZE];
volatile float iInmeasured = 0;
volatile float vInmeasured = 0;
volatile float iOutmeasured = 0;
volatile float vOutRef = 20;

//Monitor loop - slower monitored values
volatile float tempmeasured = 0;
volatile float temp_AVGBuff[AVG_MON_WINDOW_SIZE];
volatile float vOutmeasured = 0;
volatile float vOut_AVGBuff[AVG_MON_WINDOW_SIZE];


// Variablet to calculate time period for can messages
unsigned int OC_200_Timer = 0;
unsigned int OC_150_Timer = 0;
unsigned int OV_Timer = 0;
unsigned int UV_Timer =0; 

unsigned int fault_bits = 0;

volatile enum enum_boardStatus boardtripstatus = boardStatus_Idle;
volatile enum enum_boardStatus maxRetriesBoardStatus = boardStatus_NoFault;

inline void IpVoltageAndOpCurrentMonitoring(void);
inline void OpVoltageAndTemp(void );
float TI_PIControlFunc( void );
extern Control_Loop m_ctlLoop;

// For running ADC Calibration (setting ROM trim values) when running from the Debugger.
#define device_cal (void    (*)(void))0x70282

#define CAN_MSG_ID 0x7DF
// voltage PI controller structure
DCL_PI gv = { PI_KP, PI_KI, 0, PI_OUT_UP_SAT, PI_OUT_LOW_SAT, 0, PI_INTG_MAX, PI_INTG_MIN  };

// Critical code to be copied in RAM
#pragma CODE_SECTION(".TI.ramfunc")
#pragma FUNC_ALWAYS_INLINE

/*******************************************************************************************************************//**
 * IpVoltageAndOpCurrentMonitoringISR()
 *
 *    This is the interrupt service routine (ISR) for the PWM4 triangle wave.
 *    This interrupt occurs at a 100kHz rate (every 10 us).
 *
 *    It calls the High speed monitoring function which gathers and filters
 *    analog readings for module voltages and currents, input voltages and currents and run the Fault monitoring
 *    process.
 *
 *    Note: This function runs from RAM.
 *
 **********************************************************************************************************************/
interrupt void IpVoltageAndOpCurrentMonitoringISR(void)
{
    // High speed monitoring function
    IpVoltageAndOpCurrentMonitoring();

    // Clear interrupt event flag
    EPWM_clearEventTriggerInterruptFlag(HS_MONITORING_INTR_PWM_BASE);
    //Clear interrupt event group flag
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}


/*******************************************************************************************************************//**
 * OpVoltageAndTempISR()
 *
 *    This is the interrupt service routine (ISR) for the CPU timer interrupt.
 *    This interrupt occurs at a 10kHz rate (every 100 us).
 *
 *    Its calls the Control and Low speed monitoring function which gathers and filters
 *    analog readings for module temperature and calls the control loop PI function using
 *    the voltage information gathered by the IpVoltageAndOpCurrentMonitoringISR() process.
 *
 *
 **********************************************************************************************************************/
interrupt void OpVoltageAndTempISR(void)
{
    OpVoltageAndTemp();
}


/*******************************************************************************************************************//**
 * IpVoltageAndOpCurrentMonitoring()
 *
 *    This function is called by the PWM4 triangle wave ISR which occurs at a 100kHz rate (every 10 us).
 *
 *     - Reads and averages the Module currents from the ADC.
 *     - Reads and averages the Module voltages from the ADC.
 *     - Monitors for Fault conditions including:
 *         Input under voltage
 *         Input over voltage
 *         Output over current
 *
 *    Any of the above fault conditions will stop all PWM activity and de-assert all Module enable lines.
 *
 **********************************************************************************************************************/
inline void IpVoltageAndOpCurrentMonitoring (void)
{
    // We want these zeroed on each ISR entry.
    op_cur_sensed_avg = 0;
    ip_vol_sensed_avg = 0;


    read_sensed_values_HS(&ip_cur_sensed, ip_vol_sensed, op_cur_sensed); // ip_cur_sensed was disabled in the function. ip_vol_sensed was changed to array


    //At start, controlflag is initialized to zero(FLASE).
    //One buffer each is used to store the samples of output currents, input voltages and input currents.
    //For the first time, All samples are stored and summed up till index of the buffer is reached to
    //the Max set value. Then it is averaged every sampling period.
    //Once the Max index is reached, controlflag is set to 1(TRUE).
    //Then as per FIFO method, starting from zero index the previous value is subtracted from sum and
    //new sensed value is added every sampling period.

    //if first time start/ reset then controlflag=0
    if(controlflag == FALSE)
    {

        // Sum the A/D Module Analog readings.
        for (uint16_t i=0; i < NUM_SUPPORTED_MODULES; i++)
        {
            // sum the three dc-dc modules' output currents
            op_cur_sensed_avg += op_cur_sensed[i];
            ip_vol_sensed_avg += ip_vol_sensed[i];
        }

        // average of three dc-dc modules' output currents
        iOut_AVGBuff[avgCtrlWindowBuffIndex] = op_cur_sensed_avg * SENSED_READING_AVERAGE;
        mod1_iOut_AVGBuff[avgCtrlWindowBuffIndex] = op_cur_sensed[0];

         // add the sensed current to sum
        iOut_AVGFiltered += iOut_AVGBuff[avgCtrlWindowBuffIndex];
        mod1_op_cur_sensed_avg += mod1_iOut_AVGBuff[avgCtrlWindowBuffIndex];

        vIn_AVGBuff[avgCtrlWindowBuffIndex] = ip_vol_sensed_avg * SENSED_READING_AVERAGE;
        mod1_vIn_AVGBuff[avgCtrlWindowBuffIndex] = ip_vol_sensed[0];

         // add the sensed current to sum
        vIn_AVGFiltered += vIn_AVGBuff[avgCtrlWindowBuffIndex];
        mod1_ip_vIn_sensed_avg += mod1_vIn_AVGBuff[avgCtrlWindowBuffIndex];

        // ---------------------------------------------------------------------------------------------------- JL: Jan/24/2019 ----- (add 1 lines) //
        avgCtrlWindowBuffIndex++;


        //increment index
        if(avgCtrlWindowBuffIndex >= AVG_CTRL_WINDOW_SIZE)
        {
            controlflag = TRUE;
            avgCtrlWindowBuffIndex=0;
        }
    }
    else
    {
        if(avgCtrlWindowBuffIndex < AVG_CTRL_WINDOW_SIZE)
        {
            //subtract the previous value in register of present index
            iOut_AVGFiltered-= iOut_AVGBuff[avgCtrlWindowBuffIndex];
            mod1_op_cur_sensed_avg -= mod1_iOut_AVGBuff[avgCtrlWindowBuffIndex];

            // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 6 lines) //
            vIn_AVGFiltered -= vIn_AVGBuff[avgCtrlWindowBuffIndex];
            mod1_ip_vIn_sensed_avg -= mod1_vIn_AVGBuff[avgCtrlWindowBuffIndex];

            // Sum the A/D Module Analog readings.
            for (uint16_t i=0; i < NUM_SUPPORTED_MODULES; i++)
            {
                // sum the three dc-dc modules' output currents
                op_cur_sensed_avg += op_cur_sensed[i];
                ip_vol_sensed_avg += ip_vol_sensed[i];
            }

            // average of three dc-dc modules' output currents
            iOut_AVGBuff[avgCtrlWindowBuffIndex] = op_cur_sensed_avg * SENSED_READING_AVERAGE;
            mod1_iOut_AVGBuff[avgCtrlWindowBuffIndex] = op_cur_sensed[0];

            // add the sensed current to sum
            iOut_AVGFiltered += iOut_AVGBuff[avgCtrlWindowBuffIndex];
            mod1_op_cur_sensed_avg += mod1_iOut_AVGBuff[avgCtrlWindowBuffIndex];


            // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 12 lines) //
            // average of three dc-dc modules' output currents
            vIn_AVGBuff[avgCtrlWindowBuffIndex] = ip_vol_sensed_avg * SENSED_READING_AVERAGE;
            mod1_vIn_AVGBuff[avgCtrlWindowBuffIndex] = ip_vol_sensed[0];
                         // add the sensed current to sum
            vIn_AVGFiltered += vIn_AVGBuff[avgCtrlWindowBuffIndex];
            mod1_ip_vIn_sensed_avg += mod1_vIn_AVGBuff[avgCtrlWindowBuffIndex];

            // ---------------------------------------------------------------------------------------------------- JL: Jan/24/2019 ----- (add 1 lines) //
            avgCtrlWindowBuffIndex++;
        }
        else
        {
            avgCtrlWindowBuffIndex=0;
        }

        // We don't calculate these until we're up and have some valid samples (ie controlflag == TRUE)
        //These are multi module (ie Module 1, Module 2, Module 3) values averaged together
        // The actual Iout value is calculated based on this equation:
        // Iout = Iout_SLOPE(SensedIout) + Iout_YINT
        iOutmeasured  = (Iout_SLOPE * (iOut_AVGFiltered * avg_Ctrlwindow_size_factor)) + Iout_YINT;

        mod1_iOutmeasured  = (Iout_SLOPE * (mod1_op_cur_sensed_avg * avg_Ctrlwindow_size_factor)) + Iout_YINT;


        
        //check for overcurrent condition
        if (mod1_iOutmeasured >= OVER_CURRENT_LIMIT_200_PERCENT){
            OC_200_Timer++;
        }
        else if(mod1_iOutmeasured > OVER_CURRENT_LIMIT_150_PERCENT && mod1_iOutmeasured < OVER_CURRENT_LIMIT_200_PERCENT){
            
            OC_150_Timer++;
        }
        else if((OC_200_Timer || OC_150_Timer) && mod1_iOutmeasured < OVER_CURRENT_LIMIT_200_PERCENT){
            OC_200_Timer =0;
            if(mod1_iOutmeasured < OVER_CURRENT_LIMIT_150_PERCENT){
                OC_150_Timer = 0;
            }
        }



        // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (commented 3 lines) //
//        // These are single input averaged values
//        iInmeasured  = iIn_AVGFiltered * avg_Ctrlwindow_size_factor * MAX_SENSE_INPUT_CURRENT;
//        vInmeasured  = vIn_AVGFiltered * avg_Ctrlwindow_size_factor * MAX_SENSE_INPUT_VOL;

        // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 6 lines) //

        vInmeasured  = (Vin_SLOPE * (vIn_AVGFiltered * avg_Ctrlwindow_size_factor)) + Vin_YINT;
        mod1_vInmeasured  = (Vin_SLOPE * (mod1_ip_vIn_sensed_avg * avg_Ctrlwindow_size_factor)) + Vin_YINT;



  }


}


/*******************************************************************************************************************//**
 * OpVoltageAndTemp()
 *
 *    This function is called by the CPU timer ISR which occurs at a 10kHz rate (every 100 us).
 *
 *     - Reads and averages the Module temperatures and Module Output voltages from the ADC.
 *     - Calls the control loop PI function supplying the sampled/averaged voltage information.
 *     - Monitors for Fault conditions including:
 *         Output over voltage
 *         Over temperature
 *
 *    Any of the above fault conditions will stop all PWM activity and de-assert all Module enable lines.
 *
 **********************************************************************************************************************/
inline void OpVoltageAndTemp(void )
{

    // We want these zeroed on each ISR entry.
    temp_sensed_avg = 0;
    op_vol_sensed_avg = 0;


    //At start, monitorflag is initialized to zero(FALSE).
    //One buffer is used to store the samples.
    //For the first time, All samples are stored and summed up till index of the buffer is reached to
    //the Max set value. Then it is averaged every sampling period.
    //Once the Max index is reached, monitorflag is set to 1(TRUE).
    //Then as per FIFO method, starting from zero index the previous value is subtracted from sum and
    //new sensed value is added every sampling period.
    setProfilingGPIO2();

    read_sensed_values_LS(op_vol_sensed, temp_sensed);



    if(monitorflag == FALSE)
    {
        // Sum the A/D Module Analog readings.
        for (uint16_t i=0; i < NUM_SUPPORTED_MODULES; i++)
        {
            // sum the three dc-dc modules' output voltages
            op_vol_sensed_avg += op_vol_sensed[i];

            // sum the three dc-dc modules' temperature readings
            temp_sensed_avg += temp_sensed[i];
        }

        // average of three dc-dc modules' output voltages
        vOut_AVGBuff[avgMonWindowBuffIndex] = op_vol_sensed_avg * SENSED_READING_AVERAGE;
        mod1_vOut_AVGBuff[avgMonWindowBuffIndex] = op_vol_sensed[0];

        // average of three dc-dc modules' temperature
        temp_AVGBuff[avgMonWindowBuffIndex] = temp_sensed_avg * SENSED_READING_AVERAGE;
        mod1_temp_AVGBuff[avgMonWindowBuffIndex] = temp_sensed[0];

        // add the sensed voltage to sum
        vOut_AVGFiltered+= vOut_AVGBuff[avgMonWindowBuffIndex];
        mod1_op_vOut_sensed_avg += mod1_vOut_AVGBuff[avgMonWindowBuffIndex];

        // add the sensed temperature to sum
        mod1_op_temp_sensed_avg += mod1_temp_AVGBuff[avgMonWindowBuffIndex];

        temp_AVGFiltered+= temp_AVGBuff[avgMonWindowBuffIndex++];


        //increment index
        if(avgMonWindowBuffIndex >=AVG_MON_WINDOW_SIZE)
        {
            monitorflag=TRUE;
            avgMonWindowBuffIndex=0;
        }
    }

    else
    {
        if(avgMonWindowBuffIndex < AVG_MON_WINDOW_SIZE)
        {
            //subtract the previous value in register of present index
            vOut_AVGFiltered-= vOut_AVGBuff[avgMonWindowBuffIndex];
            mod1_op_vOut_sensed_avg -= mod1_vOut_AVGBuff[avgMonWindowBuffIndex];

            //subtract the previous value in register of present index
            temp_AVGFiltered-= temp_AVGBuff[avgMonWindowBuffIndex];
            mod1_op_temp_sensed_avg -= mod1_temp_AVGBuff[avgMonWindowBuffIndex];

            // Sum the A/D Module Analog readings.
            for (uint16_t i=0; i < NUM_SUPPORTED_MODULES; i++)
            {
                // sum the three dc-dc modules' output voltages
                op_vol_sensed_avg += op_vol_sensed[i];

                // sum the three dc-dc modules' temperature readings
                temp_sensed_avg += temp_sensed[i];
            }

            // average of three dc-dc modules' output voltages
            vOut_AVGBuff[avgMonWindowBuffIndex] = op_vol_sensed_avg * SENSED_READING_AVERAGE;
            mod1_vOut_AVGBuff[avgMonWindowBuffIndex] = op_vol_sensed[0];

            // average of three dc-dc modules' temperature
            temp_AVGBuff[avgMonWindowBuffIndex] = temp_sensed_avg * SENSED_READING_AVERAGE;
            mod1_temp_AVGBuff[avgMonWindowBuffIndex] = temp_sensed[0];

            // add the sensed voltage to sum
            vOut_AVGFiltered+= vOut_AVGBuff[avgMonWindowBuffIndex];
            mod1_op_vOut_sensed_avg += mod1_vOut_AVGBuff[avgMonWindowBuffIndex];

            // add the sensed temperature to sum
            mod1_op_temp_sensed_avg += mod1_temp_AVGBuff[avgMonWindowBuffIndex];
                        temp_AVGFiltered+= temp_AVGBuff[avgMonWindowBuffIndex++];

            // We don't calculate these until we're up and have some valid samples (ie monitorflag == TRUE)
            vOut_sensedFiltered_avg = vOut_AVGFiltered * avg_Monwindow_size_factor;     // Take the average of the current 100 readings

            // These are multi module (ie Module 1, Module 2, Module 3) values averaged together
            // Output voltage is based on this equation:
            // Vout = Vout_SLOPE(SensedVout) + Vout_YINT
            vOutmeasured  = (Vout_SLOPE * vOut_sensedFiltered_avg) + Vout_YINT;
            mod1_vOutmeasured  = (Vout_SLOPE * (mod1_op_vOut_sensed_avg * avg_Monwindow_size_factor)) + Vout_YINT;

            //check for Over voltage
            if (mod1_vOutmeasured >= OVER_VOLTAGE){
                OV_Timer++;
            }
            else if( OV_Timer != 0){
                OV_Timer = 0;
            }
            if(mod1_vOutmeasured <= UNDER_VOLTAGE){
                UV_Timer = 1;
            }
            else if( UV_Timer != 0){
                UV_Timer = 0;
            }
            // Temperature is calculated differently as the representative Temperature as determined by the supplied PWM, increases as the Duty Cycle (and voltage)
            // decreases. Therefore the maximum voltage is represented by the minimum reading and vice versa. A full scale A/D reading indicates a minimum temperature.
            // Temperature is based on this equation:
            // Temp = TEMPERATURE_SLOPE(SensedTemp) + TEMPERATURE_YINT

           // tempmeasured = (TEMPERATURE_SLOPE1 * (temp_AVGFiltered * avg_Monwindow_size_factor)) + TEMPERATURE_YINT;

           // mod1_Tempmeasured  = (TEMPERATURE_SLOPE * (mod1_op_temp_sensed_avg * avg_Monwindow_size_factor)) + TEMPERATURE_YINT;
           // mod2_Tempmeasured  = (TEMPERATURE_SLOPE * (mod2_op_temp_sensed_avg * avg_Monwindow_size_factor)) + TEMPERATURE_YINT;
           // mod3_Tempmeasured  = (TEMPERATURE_SLOPE * (mod3_op_temp_sensed_avg * avg_Monwindow_size_factor)) + TEMPERATURE_YINT;
            // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 2 lines) //
           // mod4_Tempmeasured  = (TEMPERATURE_SLOPE * (mod4_op_temp_sensed_avg * avg_Monwindow_size_factor)) + TEMPERATURE_YINT;
           // mod5_Tempmeasured  = (TEMPERATURE_SLOPE * (mod5_op_temp_sensed_avg * avg_Monwindow_size_factor)) + TEMPERATURE_YINT;

            // ---------------------------------------------------------------------------------------------------- JL: Jan/23/2019 ----- (add 12 lines) //
            op_temp = temp_AVGFiltered * avg_Monwindow_size_factor;
            mod1_op_temp = mod1_op_temp_sensed_avg * avg_Monwindow_size_factor;

            tempmeasured = op_temp * (TEMPERATURE_SLOPE1 * op_temp * op_temp + TEMPERATURE_SLOPE2 * op_temp + TEMPERATURE_SLOPE3) + TEMPERATURE_YINT;

             mod1_Tempmeasured  = mod1_op_temp * (TEMPERATURE_SLOPE1 * mod1_op_temp * mod1_op_temp + TEMPERATURE_SLOPE2 * mod1_op_temp + TEMPERATURE_SLOPE3) + TEMPERATURE_YINT;

                         if(mod1_Tempmeasured >= TEMP_OT_TRIP_LIMIT){
                             /*Temp limit is still not specified*/
            }
        }
        else
        {
            avgMonWindowBuffIndex=0;
        }
    }







    // load current command to the PWM modules - this has no effect on Modules that are not enabled.
    EPWM_setCounterCompareValueOptimized(DC_CONVERTER_PWM_CMD1,EPWM_COUNTER_COMPARE_A,(int)current_command_duty);
    /******************************Closed loop related fucntionalities****************************************************
    EPWM_setCounterCompareValueOptimized(DC_CONVERTER_PWM_CMD2,EPWM_COUNTER_COMPARE_A,(int)current_command_duty);
    EPWM_setCounterCompareValueOptimized(DC_CONVERTER_PWM_CMD3,EPWM_COUNTER_COMPARE_A,(int)current_command_duty);
    // ---------------------------------------------------------------------------------------------------- BK: Jan/09/2019 ----- (add 2 lines) //
    EPWM_setCounterCompareValueOptimized(DC_CONVERTER_PWM_CMD4,EPWM_COUNTER_COMPARE_A,(int)current_command_duty);
    EPWM_setCounterCompareValueOptimized(DC_CONVERTER_PWM_CMD5,EPWM_COUNTER_COMPARE_A,(int)current_command_duty);
***************************************************************************************************************************/
    resetProfilingGPIO2();

 }



// This is the default PI control loop function currently being used. Another function can be substituted by calling setPIFunc()
float TI_PIControlFunc( void )
{
  return(DCL_runPI_C5(&m_ctlLoop.get_PI_Settings(), m_ctlLoop.get_vOutRef(), vOut_sensedFiltered_avg));
}

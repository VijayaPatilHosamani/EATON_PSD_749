/* ============================================================================
System Name:    DC-DC converter

File Name:      InterruptHandlers.hpp

Target:         F28004x

Author:

Description:    converter solution header file

===========================================================================  */

#ifndef CONVERTER_H
#define CONVERTER_H


//*****************************************************************************
// the includes
//*****************************************************************************

#include "converter_board.h"
#include "ControlLoop.hpp"



//*****************************************************************************
// globals
//*****************************************************************************

enum enum_boardStatus {
    boardStatus_Idle = 0,
    boardStatus_NoFault=1,
    boardStatus_OutputOverCurrentTrip = 2,
    boardStatus_OutputOverVoltageTrip = 4,
    boardStatus_InputUnderVoltageTrip = 5,
    boardStatus_InputOverVoltageTrip = 6,
    boardstatus_Overtemptrip = 7,
    boardstatus_MaxFaultRetries = 8,
    // ---------------------------------------------------------------------------------------------------- BK: Jan/16/2019 ----- (add 2 lines) //
    boardstatus_RevProtTrip = 9,
    boardstatus_OCPTrip = 10
};

#define RESET_TRIP_Bit                       0b0
#define OUTPUT_OVER_VOLTAGE_TRIP_Bit         0b01
#define OUTPUT_OVER_CURRENT_200P_TRIP_Bit    0b010
#define OUTPUT_OVER_CURRENT_150P_TRIP_Bit    0b0100
#define OVERLOAD_TRIP_Bit                    0b01000
#define OVER_TEMPERATURE_TRIP_Bit            0b010000
#define SHORT_CIRCUIT_TRIP_Bit               0b0100000
#define UNDER_VOLTAGE_TRIP_Bit               0b01000000

#define COMMAND_DUTY_SCALING (float)(0.4 * 500)
#define SLOW_AVERAGING_BUFFER_SZ 100
#define FAST_AVERAGING_BUFFER_SZ 10

// Allocated in the converter.cpp file.
extern Control_Loop *m_ctrl_loop[];

extern unsigned int fault_bits;
// Variablet to calculate time period for can messages
extern unsigned int OC_200_Timer ;
extern unsigned int OC_150_Timer ;
extern unsigned int OV_Timer  ;
extern unsigned int UV_Timer ; 

extern volatile float mod1_vInmeasured ;
extern volatile float mod1_iOutmeasured ;
extern volatile float mod1_Tempmeasured ;
extern volatile float mod1_vOutmeasured ;


#define NOMINAL_CURRENT_RATING 140
#define OVER_CURRENT_LIMIT_200_PERCENT (2 * NOMINAL_CURRENT_RATING) 
#define OVER_CURRENT_LIMIT_150_PERCENT (1.5 * NOMINAL_CURRENT_RATING)
#define NOMINAL_VOLTAGE 28
#define OVER_VOLTAGE (1.25 * NOMINAL_VOLTAGE)
#define UNDER_VOLTAGE (.9 * NOMINAL_VOLTAGE)
#endif

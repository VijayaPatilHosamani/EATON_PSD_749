/* ============================================================================
System Name:    DC-DC converter

File Name:      converter.cpp

Target:         F28004x

Author:

Description:    This is the dc-dc converter solution file. Where main() lives.

===========================================================================  */


//*****************************************************************************
// the includes
//*****************************************************************************

#include <InterruptHandlers.hpp>
#include "Fault_Handler.hpp"
#include "can_header.hpp"


//#include "can_header.h"
#ifdef _DEBUG_ROM
#include "stdio.h"
#include "string.h"
#endif


//#include "ControlLoop.hpp"
/*--------Global variables-------------*/
//sense variables


volatile int16_t SerialCommsTimer=0;
char disp_data[DISPLAY_DATA_SIZE];


void ControlAndLowSpeedMonitoring(void );
extern float TI_PIControlFunc( void );
void Converter_CAN_Init(void);

// Define the one and only Control Loop objects Class declarations
Control_Loop m_ctlLoop(OUT_REF_VOLT_FACTOR, 50, 350);

//static CAN_Fault_Msg_Struct sample_Can_Fault_Msg = {0};


//Main function
void main(void)
{


    // This routine sets up the basic device configuration such as
    // initializing PLL, copying code from FLASH to RAM,
    // this routine will also initialize the CPU timers that are used in
    // the background task for this system
    setupDevice();

    // Stop all PWM mode clock
    disablePWMCLKCounting();


    // Setup ADC for analog signals:
    // Input current, Input voltage.
    // Output current, Output voltage and temperature of DC modules 1,2,3
    setupADC();

    // Do this at least once when running from the Debugger to set the ADC trim values in ROM.
    //(*device_cal)();


    // Enable PWM Clocks
    enablePWMCLKCounting();

    //  Set up PWM event to trigger ADCSOC.
    setupEPWM5toTriggerADCSOC();


    // setup GPIO PWM pins
    setPinsAsPWM();

    //Configure GPIO16 & 17 for testing the control ISR runtime
    setupProfilingGPIO();

    //DC Module Enable/disable Pin configuration
    setDCModuleEnablePins();

    //Setup CAN
    Converter_CAN_Init();


    // ISR Mapping
    setupInterrupt();

    // IDLE loop. Just sit and loop forever.
        for(;;)  //infinite loop
        {
            //check for fault conditions
            Fault_handler();
/*
            sample_Can_Fault_Msg.Type = 0x03;
            sample_Can_Fault_Msg.data = 0x5555555;
            sample_Can_Fault_Msg.crc = 0xFFFF;

            CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, 8,
                            (uint16_t *)&sample_Can_Fault_Msg);

            //SysCtl_delay(200000);
            DEVICE_DELAY_US(20000); // to generate delay in microsecond
*/
        }

} //END MAIN CODE








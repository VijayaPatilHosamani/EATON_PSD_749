/********************************************************************************************************************
*   Hummingbird TekSystems Copyright © 2022                                                                         *
*   All rights reserved.  This file contains confidential information, Trade secrets and other proprietary          *
*   information of HeliTrak.  Any use, Disclosure, reproduction, modification, performance, display or transfer of  *
*   this file or any part of its contents is prohibited, except with the Express written authorization of HeliTrak. *
*   Rights in this program belong to:                                                                               *
*   Hummingbird TekSystems Inc                                                                                      *
*   6 Venture, Irvine, CA 92618                                                                                     *
*   United States                                                                                                   *
*                                                                                                                   *
*   Project      : EATON - QuickSiver Firmware                                                                      *
*   Subsystem    : CAN                                                                                              *                                                                            *
*   Author       : Anil                                                                                            *
*   Version      : 1.0                                                                                              *
*   Language     : C                                                                                               *
*   Compiler     : TI v21.6.0.LTS                                                                                   *
*   Modified By  :                                                                                                  *
*   Last updated : 13/07/2022                                                                                                  *
*   Reason       :                                                                                                  *
*   Traceability :                                                                                                  *
*   PR No.       :                                                                                                  *
********************************************************************************************************************/
/* =================================================================================================================
                                           Include Files
================================================================================================================== */
#include "driverlib.h"
#include "device.h"
#include "can_header.hpp"

/* =================================================================================================================
                                              Definitions
================================================================================================================== */

/* =================================================================================================================
                                            Module Static Vars
================================================================================================================== */

/* =================================================================================================================
                                              Global Vars
================================================================================================================== */
#ifdef TRANSMIT
volatile uint32_t txMsgCount = 0;
uint32_t txMsgSuccessful  = 1;
uint16_t txMsgData[4];
#else
volatile uint32_t rxMsgCount = MSGCOUNT;
uint16_t rxMsgData[4];
#endif
volatile unsigned long i;
volatile uint32_t errorFlag = 0;

/* =================================================================================================================
                                           Functions Declarations
================================================================================================================== */
__interrupt void canaISR(void);


/*******************************************************************************************************************
Function Name   : Converter_CAN_Init
Created By: Anil
Date Created:   13/07/2022
Modification History:
    Rev No          Date        Description

Tracability:

Abstract:   Initialize GPIO and configure GPIO pins for CANTX/CANRX
            Setting of bitrate
            Enabling CAN interrupt
            Initialize trasnmit message object
            Start can module operation

Parameter Details:
Input Element:  NONE
Output Element: void

*********************************************************************************************************************/
void Converter_CAN_Init(void)
{

    //
    // Initialize GPIO and configure GPIO pins for CANTX/CANRX
    // on module A.
    //
    Device_initGPIO();
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXA);

    //
    // Initialize the CAN controllers
    //
    CAN_initModule(CANA_BASE);

    //
    // Set up the CAN bus bit rate to 500kHz for each module
    // Refer to the Driver Library User Guide for information on how to set
    // tighter timing control. Additionally, consult the device data sheet
    // for more information about the CAN module clocking.
    //
    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, BIT_RATE , 20);

    //
    // Enable interrupts on the CAN A peripheral.
    //
    CAN_enableInterrupt(CANA_BASE, CAN_INT_IE0 | CAN_INT_ERROR |
                        CAN_INT_STATUS);

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
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    // This registers the interrupt handler in PIE vector table.
    //
    Interrupt_register(INT_CANA0,&canaISR);

    //
    // Enable the CAN-A interrupt signal
    //
    Interrupt_enable(INT_CANA0);

    CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

#ifdef TRANSMIT
    //
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      CAN Module: A
    //      Message Object ID Number: 1
    //      Message Identifier: 0X7DF
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 4 Bytes
    //
    CAN_setupMessageObject(CANA_BASE, TX_MSG_OBJ_ID, CAN_ID,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_TX_INT_ENABLE, MSG_DATA_LENGTH);

#else
    //
    // Initialize the receive message object used for receiving CAN messages.
    // Message Object Parameters:
    //      CAN Module: A
    //      Message Object ID Number: 1
    //      Message Identifier: 0x15555555
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 4 Bytes (Note that DLC field is a "don't care"
    //      for a Receive mailbox
    //
    CAN_setupMessageObject(CANA_BASE, RX_MSG_OBJ_ID, 0x155,
                           CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0,
                           CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH);
#endif

    //
    // Start CAN module A operations
    //
    CAN_startModule(CANA_BASE);

}

/*******************************************************************************************************************
Function Name   : canaISR
Created By: Anil
Date Created:   13/07/2022
Modification History:
    Rev No          Date        Description

Tracability:

Abstract:   CAN A ISR - The interrupt service routine called when a CAN interrupt is triggered on CAN module A.

Parameter Details:
Input Element:  NONE
Output Element: void

*********************************************************************************************************************/

//

__interrupt void
canaISR(void)
{
    uint32_t status;

    //
    // Read the CAN-B interrupt status to find the cause of the interrupt
    //
    status = CAN_getInterruptCause(CANA_BASE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CAN_getStatus(CANA_BASE);

        //
        // Check to see if an error occurred.
        //
#ifdef TRANSMIT
        if(((status  & ~(CAN_STATUS_TXOK)) != CAN_STATUS_LEC_MSK) &&
           ((status  & ~(CAN_STATUS_TXOK)) != CAN_STATUS_LEC_NONE))
#else
        if(((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) &&
           ((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
#endif
        {
            //
            // Set a flag to indicate some errors may have occurred.
            //
            errorFlag = 1;
        }
    }
#ifdef TRANSMIT
    else if(status == TX_MSG_OBJ_ID)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANA_BASE, TX_MSG_OBJ_ID);

        //
        // Increment a counter to keep track of how many messages have been
        // transmitted.
        txMsgCount++;

        //
        // Since the message was transmitted, clear any error flags.
        //
        errorFlag = 0;

        //
        // Clear the message transmitted successful Flag.
        //
        txMsgSuccessful  = 0;
    }
#else
    else if(status == RX_MSG_OBJ_ID)
    {
        //
        // Get the received message
        //
        CAN_readMessage(CANA_BASE, RX_MSG_OBJ_ID, rxMsgData);

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANA_BASE, RX_MSG_OBJ_ID);

        //
        // Decrement the counter after a message has been received.
        //
        rxMsgCount--;

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
    }
#endif
    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//
// End of File
//

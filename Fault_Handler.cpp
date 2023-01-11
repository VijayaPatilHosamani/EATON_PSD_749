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
*   Author       : Anil                                                                                                 *
*   Version      : 1.0                                                                                              *
*   Language     : C                                                                                                *
*   Compiler     : TI v21.6.0.LTS                                                                                   *
*   Modified By  :                                                                                                  *
*   Last updated : 07/13/2022                                                                                                  *
*   Reason       :                                                                                                  *
*   Traceability :                                                                                                  *
*   PR No.       :                                                                                                  *
********************************************************************************************************************/

/* =================================================================================================================
                                           Include Files
================================================================================================================== */
#include <InterruptHandlers.hpp>
#include "can_header.hpp"

/* =================================================================================================================
                                              Definitions
================================================================================================================== */
#define CAN_MSG_ID 0x7DF
#define CRC16 0x8005
#define DURATION_5_SECONDS 500000
#define DURATION_5_MINUTES 30000000
#define DURATION_5_MILLISECONDS 50



/* =================================================================================================================
                                            Module Static Vars
================================================================================================================== */

/* =================================================================================================================
                                              Global Vars
================================================================================================================== */
extern uint32_t txMsgSuccessful;
static CAN_Fault_Msg_Struct Can_Fault_Msg = {0};

/* =================================================================================================================
                                           Functions Declarations
================================================================================================================== */



/*******************************************************************************************************************
Function Name   : gen_crc16
Created By: Anil
Date Created:   13/07/2022
Modification History:
    Rev No          Date        Description

Tracability:

Abstract: This function generates a 16bit CRC of type CRC16_CCITT_FALSE

Parameter Details:
Input Element:  data_p, length
Output Element: uint16_t

*********************************************************************************************************************/

static uint16_t gen_crc16(const uint8_t* data_p,uint16_t length){
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}






/*******************************************************************************************************************
Function Name   : Fault_handler
Created By: Anil
Date Created:   07/13/2022
Modification History:
    Rev No          Date        Description

Tracability:

Abstract: This function checks if any of the fault conditions sustained over a predefined period of time
            Then sets the fault_bits accordingly.
            if any of the faults bits are set then it Triggers a CAN message.


Parameter Details:
Input Element:  NONE
Output Element: void

*********************************************************************************************************************/




void Fault_handler(void)
{
    // because if sampling Iout at every 10us so one increment count will take 10usec and 500000 count is equal to  5 sec

    //Over Current 200% for duration of 5 seconds set the fault bits accordingly
    if(OC_200_Timer > DURATION_5_SECONDS)
    {
        fault_bits |= OUTPUT_OVER_CURRENT_200P_TRIP_Bit;
        OC_200_Timer = 0;
    }
    //Over Current 150% for duration of 5 minutes set the fault bits accordingly
    if(OC_150_Timer > DURATION_5_MINUTES){
        fault_bits |= OUTPUT_OVER_CURRENT_150P_TRIP_Bit;
        OC_150_Timer = 0;
    }
    //Over voltage 125 % for duration of 5ms set the fault bits accordingly
    if(OV_Timer > DURATION_5_MILLISECONDS){
        fault_bits |= OUTPUT_OVER_VOLTAGE_TRIP_Bit;
        OV_Timer = 0;
    }
    //Under voltage condition set the fault bits accordingly
    if(UV_Timer != 0)
    {
        fault_bits |= UNDER_VOLTAGE_TRIP_Bit;
        UV_Timer = 0;
    }

    /* To check working of CAN set the bit of the fault_bits variable with these macros
    OUTPUT_OVER_VOLTAGE_TRIP_Bit
    OUTPUT_OVER_CURRENT_200P_TRIP_Bit
    OUTPUT_OVER_CURRENT_150P_TRIP_Bit
    OVERLOAD_TRIP_Bit
    OVER_TEMPERATURE_TRIP_Bit
    SHORT_CIRCUIT_TRIP_Bit
    UNDER_VOLTAGE_TRIP_Bit

    ex: fault_bits = OUTPUT_OVER_VOLTAGE_TRIP_Bit;
    if you want delay between the CAN messages then add the below instruction
    DEVICE_DELAY_US(250000);
    250000 is in micro seconds
     */

    if(fault_bits)
    {
        Can_Fault_Msg.Msg_id = 0x08;
        //Check if Overvoltage bit is set
        if((fault_bits & OUTPUT_OVER_VOLTAGE_TRIP_Bit) == OUTPUT_OVER_VOLTAGE_TRIP_Bit){
            //send overvoltage fault message
            Can_Fault_Msg.Type = 0x01;
            Can_Fault_Msg.data = mod1_vOutmeasured;
            Can_Fault_Msg.crc = gen_crc16((uint8_t *)&Can_Fault_Msg,6);

            CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, 8,
                            (uint16_t *)&Can_Fault_Msg);
            fault_bits = fault_bits & (~(OUTPUT_OVER_VOLTAGE_TRIP_Bit));


        }
        //Check if Overcurrent 200 percent bit is set
        if((fault_bits & OUTPUT_OVER_CURRENT_200P_TRIP_Bit) == OUTPUT_OVER_CURRENT_200P_TRIP_Bit){
            //send overCurrent_200A fault message
            Can_Fault_Msg.Type = 0x02;
            Can_Fault_Msg.data = mod1_iOutmeasured;
            Can_Fault_Msg.crc = gen_crc16((uint8_t *)&Can_Fault_Msg,6);

            CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, 8,
                            (uint16_t *)&Can_Fault_Msg);

            fault_bits = fault_bits & (~(OUTPUT_OVER_CURRENT_200P_TRIP_Bit));

        }
        //Check if Overcurrent 150 percent bit is set
        if((fault_bits & OUTPUT_OVER_CURRENT_150P_TRIP_Bit) == OUTPUT_OVER_CURRENT_150P_TRIP_Bit){
            //send overcurrent 150A fault message
            Can_Fault_Msg.Type = 0x03;
            Can_Fault_Msg.data = mod1_iOutmeasured;
            Can_Fault_Msg.crc = gen_crc16((uint8_t *)&Can_Fault_Msg,6);

            CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, 8,
                            (uint16_t *)&Can_Fault_Msg);

            fault_bits = fault_bits & (~(OUTPUT_OVER_CURRENT_150P_TRIP_Bit));

        }
        //Check if Overload trip bit is set
        if((fault_bits & OVERLOAD_TRIP_Bit) == OVERLOAD_TRIP_Bit){
            //send Overload fault message
            Can_Fault_Msg.Type = 0x04;
            Can_Fault_Msg.data = mod1_iOutmeasured;
            Can_Fault_Msg.crc = gen_crc16((uint8_t *)&Can_Fault_Msg,6);

            CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, 8,
                            (uint16_t *)&Can_Fault_Msg);

            fault_bits = fault_bits & (~(OVERLOAD_TRIP_Bit));

        }
        //Check if Overtemperature trip bit is set
        if((fault_bits & OVER_TEMPERATURE_TRIP_Bit) == OVER_TEMPERATURE_TRIP_Bit){
            //send OverTemp fault message
            Can_Fault_Msg.Type = 0x05;
            Can_Fault_Msg.data = mod1_Tempmeasured;
            Can_Fault_Msg.crc = gen_crc16((uint8_t *)&Can_Fault_Msg,6);

            CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, 4,
                            (uint16_t *)&Can_Fault_Msg);

            fault_bits = fault_bits & (~( OVER_TEMPERATURE_TRIP_Bit));

        }
        //check if Short circuit trip bit is set
        if((fault_bits & SHORT_CIRCUIT_TRIP_Bit) == SHORT_CIRCUIT_TRIP_Bit){
            //send Short Ciruit fault message
            Can_Fault_Msg.Type = 0x06;
            Can_Fault_Msg.data = mod1_iOutmeasured;
            Can_Fault_Msg.crc = gen_crc16((uint8_t *)&Can_Fault_Msg,6);

            CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, 8,
                            (uint16_t *)&Can_Fault_Msg);

            fault_bits = fault_bits & (~(SHORT_CIRCUIT_TRIP_Bit));

        }
        // check if under voltage trip bit is set
        if((fault_bits & UNDER_VOLTAGE_TRIP_Bit) == UNDER_VOLTAGE_TRIP_Bit){
            //send Under voltage fault message
            Can_Fault_Msg.Type = 0x07;
            Can_Fault_Msg.data = mod1_vOutmeasured;
            Can_Fault_Msg.crc = gen_crc16((uint8_t *)&Can_Fault_Msg,6);

            CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, 8,
                            (uint16_t *)&Can_Fault_Msg);

            fault_bits = fault_bits & (~(UNDER_VOLTAGE_TRIP_Bit));

        }

    }

}





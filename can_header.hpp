#ifndef CAN_HEADER_HPP_
#define CAN_HEADER_HPP_

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
*   Last updated : 13/07/2022                                                                                                  *
*   Reason       :                                                                                                  *
*   Traceability :                                                                                                  *
*   PR No.       :                                                                                                  *
********************************************************************************************************************/
/* =================================================================================================================
                                           Include Files
================================================================================================================== */

/* =================================================================================================================
                                              Definitions
================================================================================================================== */
#define TRANSMIT
#define BIT_RATE    500000
#define CAN_ID      0x7DF
#ifdef TRANSMIT
#define TX_MSG_OBJ_ID    1
#else
#define RX_MSG_OBJ_ID    1
#endif
#define MSG_DATA_LENGTH  4
//#define MSGCOUNT        10

/* =================================================================================================================
                                              Variable Decalaration
================================================================================================================== */
typedef struct CAN_Fault_Struct{
    uint8_t Msg_id;
    uint8_t Type;
    float data;
    uint16_t crc;
}CAN_Fault_Msg_Struct;


/* =================================================================================================================
                                           Functions Declarations
================================================================================================================== */





#endif

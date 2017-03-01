/**************************************************************************************************
  Filename:       GenericApp.h
  Revised:        $Date: 2007-10-27 17:22:23 -0700 (Sat, 27 Oct 2007) $
  Revision:       $Revision: 15795 $

  Description:    This file contains the Generic Application definitions.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef GENERICAPP_H
#define GENERICAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define GENERICAPP_ENDPOINT           10

#define GENERICAPP_PROFID             0x0F04
#define GENERICAPP_DEVICEID           0x0001
#define GENERICAPP_DEVICE_VERSION     0
#define GENERICAPP_FLAGS              0

#define GENERICAPP_IN_CLUSTERS        4
#define GENERICAPP_OUT_CLUSTERS       3  

#define GENERICAPP_CLUSTERID                  0x0001   // I/O
#define GENERICAPP_CLUSTERID_START            0x0010   // I
#define GENERICAPP_CLUSTERID_STOP             0x0011   // I
  
#define GENERICAPP_CLUSTERID_SYNC             0x0020   // I
#define GENERICAPP_CLUSTERID_ECG_SYNC_OVER    0x0021   // O
//#define GENERICAPP_CLUSTERID_TEMPR_SYNC_OVER  0x0022   //O
//#define GENERICAPP_CLUSTERID_SPO2_SYNC_OVER   0x0023    //O

#define GENERICAPP_CLUSTERID_ECG_RESULT       0x0030   // O
//#define GENERICAPP_CLUSTERID_TEMPR_RESULT   0x0031   // O
//#define GENERICAPP_CLUSTERID_SPO2_RESULT   0x0032   // O
  
  
// Send SYNC Message Timeout
#define GENERICAPP_SEND_SYNC_FILE_TIMEOUT   1000     // 文件与文件之间同步间隔为1s
#define GENERICAPP_SEND_SYNC_PACKET_TIMEOUT 240       // 包与包之间同步间隔为80ms*3
  
// Sample ECG Timeout
#define GENERICAPP_SAMPLE_ECG_TIMEOUT 10000     // Every 10ms = 100Hz  8ms每次太频繁，收不到数据
  
// Measure status show timeout
#define GENERICAPP_MEASURE_STATUS_SHOW      500     // 500ms闪烁一次
  
// Application Events (OSAL) - These are bit weighted definitions.
  // 除了Ox8000-SYS_EVENT_MSG均可用，一共可以注册15个用户事件
//#define GENERICAPP_SEND_MSG_EVT       0x0001

#define GENERICAPP_START_MEASURE      0x0002
#define GENERICAPP_STOP_MEASURE       0x0004
#define GENERICAPP_ECG_MEAS_BUFF_FULL 0x0008
#define GENERICAPP_ECG_SYNC           0x0010
#define GENERICAPP_ECG_MEASURE_LOGO   0x0020  
  
/* OLED show coordinates*/
#define DEVICE_INFO_X               10
#define DEVICE_INFO_Y               0
#define DEVICE_INFO_SIZE            12

#define DEVICE_INFO_ONLINE_IDLE_ID      1
#define DEVICE_INFO_ONLINE_IDLE         "On_Ready "
  
#define DEVICE_INFO_OFFLINE_IDLE_ID     2
#define DEVICE_INFO_OFFLINE_IDLE        "Off_Ready"

#define DEVICE_INFO_ONLINE_MEASURE_ID   3
#define DEVICE_INFO_ONLINE_MEASURE      "On_Meas  "

#define DEVICE_INFO_OFFLINE_MEASURE_ID  4 
#define DEVICE_INFO_OFFLINE_MEASURE     "Off_Meas "
  
#define DEVICE_INFO_FIND_NWK_ID         5
#define DEVICE_INFO_FIND_NWK            "Find_NWK "
  
#define DEVICE_INFO_CLOSING_ID          6
#define DEVICE_INFO_CLOSING             "Closing  "
  
#define DEVICE_INFO_ERROR_ID            7
#define DEVICE_INFO_ERROR               "Error    "
  
#define DEVICE_INFO_SYNC_DATA_ID        8
#define DEVICE_INFO_SYNC_DATA           "Sync_data"
 
#define HR_CHAR_X                       18
#define HR_CHAR_Y                       16
#define HR_CHAR_SIZE                    32
  
  
#define HR_RESULT_X                     58
#define HR_RESULT_Y                     16
#define HR_RESULT_SIZE                  32
  
#define HR_WAIT_SYMBOL_X                58
#define HR_WAIT_SYMBOL_Y                39
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
  ECG_ONLINE_IDLE,
  ECG_ONLINE_MEASURE,
  ECG_OFFLINE_IDLE,
  ECG_OFFLINE_MEASURE,
  ECG_FIND_NETWORK,
  ECG_SYNC_DATA,
  ECG_CLOSING
} EcgSystemStatus_t;

typedef enum
{
  SYNC_READ_DIR,
  SYNC_READ_DATA,
  SYNC_SEND_DATA,
  SYNC_CLOSE_FILE,
} SyncStatus_t;

typedef enum
{
  WORK_STATUS,
  IDLE_STATUS
} WorkStatus_t;
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void GenericApp_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GENERICAPP_H */

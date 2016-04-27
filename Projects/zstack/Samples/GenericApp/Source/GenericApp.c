/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2010-12-21 10:27:34 -0800 (Tue, 21 Dec 2010) $
  Revision:       $Revision: 24670 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
******************************************************************************/

/*********************************************************************
  This application is for ECG
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "hal_oled.h"
#include "hal_ecg_measure.h"
#include "hal_rtc_ds1302.h"
#include "hal_SDcard.h"
#include "exfuns.h"
#include "ff.h"

#include "string.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_InClusterList[GENERICAPP_IN_CLUSTERS] =
{
  GENERICAPP_CLUSTERID,
  GENERICAPP_CLUSTERID_START,
  GENERICAPP_CLUSTERID_STOP,
  GENERICAPP_CLUSTERID_SYNC
};

const cId_t GenericApp_OutClusterList[GENERICAPP_OUT_CLUSTERS] =
{
  GENERICAPP_CLUSTERID,
  GENERICAPP_CLUSTERID_SYNC_OVER,
  GENERICAPP_CLUSTERID_ECG_RESULT
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,                  //  int Endpoint;
  GENERICAPP_PROFID,                    //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,                  //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,            //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                     //  int   AppFlags:4;
  GENERICAPP_IN_CLUSTERS,               //  byte  AppNumInClusters;
  (cId_t *)GenericApp_InClusterList,    //  byte *pAppInClusterList;
  GENERICAPP_OUT_CLUSTERS,              //  byte  AppNumOutClusters;
  (cId_t *)GenericApp_OutClusterList    //  byte *pAppOutClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

// This is for GenericApp state machine。
EcgSystemStatus_t EcgSystemStatus;

SyncStatus_t SyncStatus;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

char fileName[30]; // store file name
char pathname[30]; // read file name
uint8 *dataSendBuffer;
uint8 *dataSendBufferTemp;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void GenericApp_HandleKeys( byte shift, byte keys );
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );

void GenericApp_EcgMeasCB(void);
void GenericApp_LeaveNetwork( void );
void GenericApp_HandleNetworkStatus( devStates_t GenericApp_NwkStateTemp);
void GenericApp_HandleBufferFull( void );
void GenericApp_GetWriteName( char *fileName );
void GenericApp_SyncData(void);
bool GenericApp_OpenDir(void);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( byte task_id )
{
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );

  // Register for ecg measure and event
  HalEcgMeasConfig(GenericApp_EcgMeasCB);
  
  // Init ECG status
  EcgSystemStatus = ECG_OFFLINE_IDLE;
  SyncStatus = SYNC_READ_DIR;
    
  // Init SD card and fatfs
  while(SD_Initialize());
  exfuns_init();      // 申请文件系统内存
  f_mount(0,fs);      // 挂载文件系统  
  f_mkdir("0:D");     // 创建文件夹
  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif
  HalOledShowString(0,0,32,"OFF-IDLE");
  HalOledRefreshGram();
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          
          GenericApp_HandleNetworkStatus(GenericApp_NwkState);
          
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  // Start Measure
  if ( events & GENERICAPP_START_MEASURE )
  {
    // Change status and start measure
    if( EcgSystemStatus == ECG_OFFLINE_IDLE ) // 离线状态
    {
      EcgSystemStatus = ECG_OFFLINE_MEASURE;
      HalOledShowString(0,0,32,"OFF-MEAS");
      
      // 打开文件并创建文件 保证不会有两次测量的数据混在一个文件中
      GenericApp_GetWriteName(fileName);
      f_open(file,fileName,FA_CREATE_ALWAYS | FA_WRITE);

      HalEcgMeasStart( GENERICAPP_SAMPLE_ECG_TIMEOUT , ECG_BUFFER_FOR_SD );
    }
    else if ( EcgSystemStatus == ECG_ONLINE_IDLE ) // 在线状态
    {
      EcgSystemStatus = ECG_ONLINE_MEASURE;
      HalOledShowString(0,0,32,"ON-MEAS");
      
      HalEcgMeasStart( GENERICAPP_SAMPLE_ECG_TIMEOUT , ECG_BUFFER_FOR_ZIGBEE );
    }
    HalOledRefreshGram();
    
    // return unprocessed events
    return (events ^ GENERICAPP_START_MEASURE);
  }
  
  // Handle Measure result
  // send to GW or store to SD card
  if ( events & GENERICAPP_ECG_MEAS_BUFF_FULL )
  {
     // Send to GW or store in SD
    GenericApp_HandleBufferFull();
    
    // return unprocessed events
    return (events ^ GENERICAPP_ECG_MEAS_BUFF_FULL);
  }
  
  // Close Measure
  if ( events & GENERICAPP_STOP_MEASURE )
  {
    // Close Measrue
    HalEcgMeasStop();
    
    // Change status
    if( EcgSystemStatus == ECG_OFFLINE_MEASURE ) // 离线测量
    {
      // 关闭文件
      f_close(file);
      
      EcgSystemStatus = ECG_OFFLINE_IDLE;
      HalOledShowString(0,0,32,"OFF-IDLE");
      
    }
    else if ( EcgSystemStatus == ECG_ONLINE_MEASURE ) // 在线测量
    {
      EcgSystemStatus = ECG_ONLINE_IDLE;
      HalOledShowString(0,0,32,"ON-IDLE");
    }
    HalOledRefreshGram();
    
    // return unprocessed events
    return (events ^ GENERICAPP_STOP_MEASURE);
  }
  
  // SYNC data
  if ( events & GENERICAPP_ECG_SYNC )
  {
    if(EcgSystemStatus == ECG_SYNC_DATA)
      GenericApp_SyncData();
    else if(SyncStatus != SYNC_READ_DIR) // 断网了关闭文件释放空间
    {
      // 关闭文件释放buffer
      f_close(file);
      osal_mem_free(dataSendBuffer);
    }
    
    // return unprocessed events
    return (events ^ GENERICAPP_ECG_SYNC);
  }  
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void GenericApp_HandleKeys( byte shift, byte keys )
{
  if(keys & HAL_KEY_SW_6)   //Link button be pressed
  {
    if( EcgSystemStatus == ECG_OFFLINE_IDLE ) // 离线-->寻找网络
    {
      if( ZDApp_StartJoiningCycle() == FALSE )
        if( ZDOInitDevice(0) == ZDO_INITDEV_LEAVE_NOT_STARTED) //Start Network
          ZDOInitDevice(0);
      EcgSystemStatus = ECG_FIND_NETWORK;
    }
    else if( EcgSystemStatus == ECG_ONLINE_IDLE) // 在线-->离线
    {
      // Leave Network
      GenericApp_LeaveNetwork(); 
      EcgSystemStatus = ECG_CLOSING;
     
      HalOledShowString(0,0,32,"CLOSE");
      HalOledRefreshGram();
      
    }
    else if ( EcgSystemStatus == ECG_FIND_NETWORK ) // 寻找网络-->离线
    {
      // Stop search network
      ZDApp_StopJoiningCycle();
      EcgSystemStatus = ECG_OFFLINE_IDLE;
      
      HalOledShowString(0,0,32,"OFF-IDLE");
      HalOledRefreshGram();
    }
    else // Online , Offline measure , closing , SYNC
    {}//do nothing
  }
  
  /* Work button be pressed */
  if(keys & HAL_KEY_SW_7)   
  {
    if( EcgSystemStatus == ECG_OFFLINE_IDLE || EcgSystemStatus == ECG_ONLINE_IDLE ) // 空闲状态 启动测量
      osal_set_event(GenericApp_TaskID,GENERICAPP_START_MEASURE);
    else if( EcgSystemStatus == ECG_OFFLINE_MEASURE || EcgSystemStatus == ECG_ONLINE_MEASURE ) // 测量状态 关闭测量
      osal_set_event(GenericApp_TaskID,GENERICAPP_STOP_MEASURE);
    else
    {} // find network , closing , SYNC 

  }

    
  
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
      // "the" message
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
      
    case GENERICAPP_CLUSTERID_START:
      if(EcgSystemStatus == ECG_ONLINE_IDLE)  // 在线空闲才能启动测量
      {
        HalOledShowString(20,0,16,"start");
        HalOledShowString(20,15,16,"V0.56");
        HalOledRefreshGram();
        osal_set_event( GenericApp_TaskID , GENERICAPP_START_MEASURE );
      }
      break;
      
    case GENERICAPP_CLUSTERID_STOP:  
      if(EcgSystemStatus == ECG_ONLINE_MEASURE) // 在线测量才能停止测量
      {
        HalOledShowString(20,0,16,"stop");
        HalOledShowString(20,15,16,"V0.56");
        HalOledRefreshGram();      
        osal_set_event( GenericApp_TaskID , GENERICAPP_STOP_MEASURE );
      }
      break;
      
    case GENERICAPP_CLUSTERID_SYNC:
      if(EcgSystemStatus == ECG_ONLINE_IDLE)  // 在线空闲才能同步
      {
        HalOledShowString(20,0,16,"SYNC");
        HalOledShowString(20,15,16,"V0.56");
        HalOledRefreshGram();
        
        EcgSystemStatus = ECG_SYNC_DATA;
        osal_set_event( GenericApp_TaskID , GENERICAPP_ECG_SYNC );
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleNetworkStatus
 *
 * @brief  According to EcgSystemStatus to do different thing
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_HandleNetworkStatus( devStates_t GenericApp_NwkStateTemp)
{
  if( GenericApp_NwkStateTemp == DEV_END_DEVICE) //connect to GW
  {
      EcgSystemStatus = ECG_ONLINE_IDLE;
      HalOledShowString(0,0,32,"ON-IDLE");
  }
  else if( EcgSystemStatus != ECG_OFFLINE_IDLE) // Find network -- 1.coordinate lose 2.first connect to coordinate 
  { // 关闭搜索后，可能由于OSAL的timer事件设置，再进入一次ZDO_STATE_CHANGE，上面的判断就是为了排除这种情况
    if ( EcgSystemStatus == ECG_ONLINE_MEASURE ) // Online measure status
      HalEcgMeasStop();        // stop measure
    
    EcgSystemStatus = ECG_FIND_NETWORK;
    HalOledShowString(0,0,32,"FIND-NWK");     
  }
    
  HalOledRefreshGram();
  
}


/*********************************************************************
 * @fn      GenericApp_HandleBufferFull
 *
 * @brief   Send to GW or store in SD when the buffer full
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_HandleBufferFull( void )
{
  uint16 *dataTemp;
  
  if (  EcgSystemStatus == ECG_ONLINE_MEASURE  ) // 在线测量
  {
    // Get data and send data
    HalEcgMeasReadFromBuf( &dataTemp );
    AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                   GENERICAPP_CLUSTERID_ECG_RESULT,
                   ECG_WAVEFORM_SAMPLER_NUM_PER_PACKET*2,
                   (uint8 *)dataTemp,
                   &GenericApp_TransID,
                   AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
  }
  else if (  EcgSystemStatus == ECG_OFFLINE_MEASURE  ) // 离线测量
  {
    // Get data and write to SD card
    HalEcgMeasReadFromBuf( &dataTemp );
    f_write(file,dataTemp,512,&bw);
  }
  
  
}


/*********************************************************************
 * @fn      GenericApp_EcgMeasCB
 *
 * @brief   For handle the ecg measture result.Get and store value 
 *          then set event according to different status.
 *
 * @param   none
 *
 * @return  none
 */
uint16 a = 65;
void GenericApp_EcgMeasCB(void)
{
  uint16 ECGSample;
  BufOpStatus_t OpStatus;
  
  //采集数据
  //ECGSample = HalEcgMeasSampleVal();
  ECGSample = a;
  a++;
  if( a == 91 )
    a = 65;
  
  //Write to buffer
  OpStatus = HalEcgMeasWriteToBuf(ECGSample);

  //根据情况执行不同的事件
  if (OpStatus == PingPongBuf_WRITE_SWITCH) // Success and switch buff
  {
    osal_set_event(GenericApp_TaskID,GENERICAPP_ECG_MEAS_BUFF_FULL);
  }
  else if(OpStatus == PingPongBuf_WRITE_FAIL )// fail
  {
    HalEcgMeasBuffReset();
  }
  else  //Success
  { 
    //do nothing
  }
}

/*********************************************************************
 * @fn      GenericApp_LeaveNetwork
 *
 * @brief   Let device leave network.
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_LeaveNetwork( void )
{
  NLME_LeaveReq_t leaveReq;

  osal_memset((uint8 *)&leaveReq,0,sizeof(NLME_LeaveReq_t));
  osal_memcpy(leaveReq.extAddr,NLME_GetExtAddr(),Z_EXTADDR_LEN);

  leaveReq.removeChildren = FALSE; // Only false shoule be use.
  leaveReq.rejoin = FALSE;  
  leaveReq.silent = FALSE;

  NLME_LeaveReq( &leaveReq );
}


/*********************************************************************
 * @fn      GenericApp_GetWriteName
 *
 * @brief   Get the RTC time and make file name.
 *
 * @param   char *
 *          0:D/20xx-xx-xx xx-xx-xx x.txt + \0 = 30Byte
 *
 * @return  
 *
 */
void GenericApp_GetWriteName( char *fileName )
{
  RTCStruct_t RTCStruct;
  HalRTCGetOrSetFull(RTC_DS1302_GET,&RTCStruct);

  // Make file name
  fileName[0] = '0';
  fileName[1] = ':';
  fileName[2] = 'D';
  fileName[3] = '/';
  fileName[4] = '2';
  fileName[5] = '0';
  fileName[6] = RTCStruct.year/10 + '0';
  fileName[7] = RTCStruct.year%10 + '0';
  fileName[8] = '-';
  fileName[9] = RTCStruct.month/10 + '0';
  fileName[10] = RTCStruct.month%10 + '0';
  fileName[11] = '-';
  fileName[12] = RTCStruct.date/10 + '0';
  fileName[13] = RTCStruct.date%10 + '0';
  fileName[14] = ' ';
  fileName[15] = RTCStruct.hour/10 + '0';
  fileName[16] = RTCStruct.hour%10 + '0';
  fileName[17] = '-';
  fileName[18] = RTCStruct.min/10 + '0';
  fileName[19] = RTCStruct.min%10 + '0';
  fileName[20] = '-';
  fileName[21] = RTCStruct.sec/10 + '0';
  fileName[22] = RTCStruct.sec%10 + '0';
  fileName[23] = ' ';  
  fileName[24] = RTCStruct.week + '0';
  fileName[25] = '.';
  fileName[26] = 't';
  fileName[27] = 'x';
  fileName[28] = 't';
  fileName[29] = '\0';
}


/*********************************************************************
 * @fn      GenericApp_SyncData
 *
 * @brief   Sync data.同步的状态机设计
 *
 * @param  
 *
 * @return  
 *
 */
void GenericApp_SyncData(void)
{
  
  switch(SyncStatus)
  {
  case SYNC_READ_DIR:
    if(GenericApp_OpenDir() == TRUE)
    {
      f_open(file,(char *)pathname,FA_OPEN_EXISTING | FA_READ); //  打开文件
      // 申请空间
      dataSendBuffer = osal_mem_alloc(sizeof(uint8)*500);
      if(dataSendBuffer == NULL)
      {
        f_close(file);
        EcgSystemStatus = ECG_ONLINE_IDLE; // 同步结束
        return;
      }
      // 切换状态并启动下次时间
      SyncStatus = SYNC_READ_DATA;
      osal_set_event( GenericApp_TaskID , GENERICAPP_ECG_SYNC );
    }
    else
    {
      EcgSystemStatus = ECG_ONLINE_IDLE; // 同步结束
      AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID_SYNC_OVER,
                       0,
                       NULL,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
      EcgSystemStatus = ECG_ONLINE_IDLE;
      HalOledShowString(0,0,32,"ON-IDLE");
      HalOledRefreshGram();
    }
    
    break;
  case SYNC_READ_DATA:
    f_read(file,dataSendBuffer,500,&br);
    dataSendBufferTemp = dataSendBuffer;
    br = br/20;
    if(br == 0) // 没有数据了
      SyncStatus = SYNC_CLOSE_FILE;
    else
      SyncStatus = SYNC_SEND_DATA;
    osal_set_event( GenericApp_TaskID , GENERICAPP_ECG_SYNC );
    break;
  case SYNC_SEND_DATA:
    if(br == 0) //该buffer同步结束
      SyncStatus = SYNC_READ_DATA;
    else
    {
      // 发送数据
      AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID_ECG_RESULT,
                       ECG_WAVEFORM_SAMPLER_NUM_PER_PACKET*2,
                       dataSendBufferTemp,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
      br--;
      dataSendBufferTemp += 20;
      
    }
    // 50ms后再次启动事件
    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_ECG_SYNC,
                        GENERICAPP_SEND_SYNC_PACKET_TIMEOUT );
    break;
    
  case SYNC_CLOSE_FILE:
    // 关闭文件释放buffer
    f_close(file);
    f_unlink((char *)pathname);
    osal_mem_free(dataSendBuffer);
    // 切换状态，1s后再次启动同步事件
    SyncStatus = SYNC_READ_DIR;
    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_ECG_SYNC,
                        GENERICAPP_SEND_SYNC_FILE_TIMEOUT );
    break;
  }

}


/*********************************************************************
 * @fn      GenericApp_OpenDir
 *
 * @brief   找到指定目录下文件
 *
 * @param  
 *
 * @return  true is over,flase is not over
 *
 */
bool GenericApp_OpenDir(void)
{
  uint8 res = 0;
  DIR *fddir = 0;         // 目录
  FILINFO *finfo = 0;     // 文件信息
  uint8 *fn = 0;          // 长文件名
  
  // initilize pathname
  strcpy(pathname,"0:D/");
  
  // 申请内存
  fddir = (DIR *)osal_mem_alloc(sizeof(DIR));
  finfo = (FILINFO *)osal_mem_alloc(sizeof(FILINFO));
  if(fddir == NULL || finfo == NULL)
  {
    if(fddir != NULL)
      osal_mem_free(fddir);
    if(finfo != NULL)
      osal_mem_free(finfo);
    return FALSE;
  }
  
  finfo->lfsize = 28 + 1;
  finfo->lfname = osal_mem_alloc(finfo->lfsize);
  if(finfo->lfname == NULL)
  {
    osal_mem_free(finfo->lfname);
    osal_mem_free(fddir);
    osal_mem_free(finfo);
    return FALSE;
  }
  
  // 打开源目录
  res = f_opendir(fddir,"0:D");

  if(res == 0)  // 打开目录成功
  {
    while(res == 0)
    {
      res = f_readdir(fddir,finfo);   //读取目录下的一个文件
     
      if(res != FR_OK || finfo->fname[0] == 0) // 出错或者读到了结尾
      {
        osal_mem_free(finfo->lfname);
        osal_mem_free(fddir);
        osal_mem_free(finfo);
        return FALSE;
      }
      
      if(finfo->fname[0] == '.') // 忽略上层文件
        continue;
      
      /* 存在文件,保存字符串 */
      fn = (uint8 *)(*finfo->lfname?finfo->lfname:finfo->fname);
      strcat((char *)pathname,(char *)fn);
      
      break;
    }
  }
  
  osal_mem_free(finfo->lfname);
  osal_mem_free(fddir);
  osal_mem_free(finfo);
  return TRUE;  
}

/*********************************************************************
*********************************************************************/

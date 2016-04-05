/**************************************************************************************************
  Filename:       hal_battery_monitor.c
  Revised:        $Date: 2016-03-12 19:37:16 +0800 (Sat, 12 Mar 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to the OLED Service.


  Copyright 2016 Bupt. All rights reserved.

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
  contact kylinnevercry@gami.com. 

  使用Timer1 作为ECG采集的定时器
**************************************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/
#include "hal_ecg_measure.h"
#include "hal_led.h"

#if (defined HAL_ECG_MEASURE) && (HAL_ECG_MEASURE == TRUE)
/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/


/***************************************************************************************************
 *                                              MACROS
 ***************************************************************************************************/


/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/

/**************************************************************************************************
 *                                        INNER GLOBAL VARIABLES
 **************************************************************************************************/



/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/


/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalEcgMeasInit
 *
 * @brief   Initilize ECG measure
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalEcgMeasInit(void)
{
  //设置定时器
  T1CTL = 0x0f;   //DIV 128; Up/Down 模式;
  T1CC0H = 0xFF;
  T1CC0L = 0xFF;  //0x01F4:周期为4ms；0x0271:周期为5ms;0x03E8:周期为8ms;0x04E2:周期为10ms;0x09C4:周期为20ms
  
  IEN1  |= 0x02;  //bit1:T1IE
  //T1OVFIM = 0;    //T1 overflow interrupt mask
}



/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINE
 ***************************************************************************************************/

/**************************************************************************************************
 * @fn      halTimer1Isr
 *
 * @brief   Timer1 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halTimer1Isr, T1_VECTOR )
{
  HAL_ENTER_ISR();
  

  HalLedSet(HAL_LED_1,HAL_LED_MODE_TOGGLE);
  //halProcessTimer1Interrupt();
  
  HAL_EXIT_ISR();
}


#else
void HalEcgMeasInit(void);
void HalEcgMeasEnable(void);
void HalEcgMeasDisable(void);
void HalEcgMeasGetValue(void);



#endif /* HAL_OLED */
/**************************************************************************************************
  Filename:       hal_ecg_measure.h
  Revised:        $Date: 2016-04-05 15:41:16 +0800 (Thu, 5 Apr 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to the battery measurement.


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
**************************************************************************************************/

#ifndef HAL_ECG_MEASURE_H
#define HAL_ECG_MEASURE_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_board.h"

/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

  
/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
/* Timer clock pre-scaler definitions for 16bit timer1 */
#define HAL_TIMER1_16_TC_DIV1     0x00  /* No clock pre-scaling */
#define HAL_TIMER1_16_TC_DIV8     0x04  /* Clock pre-scaled by 8 */
#define HAL_TIMER1_16_TC_DIV32    0x08  /* Clock pre-scaled by 32 */
#define HAL_TIMER1_16_TC_DIV128   0x0c  /* Clock pre-scaled by 128 */
#define HAL_TIMER1_16_TC_BITS     0x0c  /* Bits 3:2 */
  
/* Operation Mode definitions */
#define HAL_TIMER1_OPMODE_STOP      0x00  /* Free Running Mode, Count from 0 to Max */
#define HAL_TIMER1_OPMODE_FREERUN   0x01  /* Free Running Mode, Count from 0 to Max */
#define HAL_TIMER1_OPMODE_MODULO    0x02  /* Modulo Mode, Count from 0 to CompareValue */
#define HAL_TIMER1_OPMODE_UP_DOWN   0x03  /* up-down Mode, Count from 0 to CompareValue to 0*/
#define HAL_TIMER1_OPMODE_BITS      0x03  /* Bits 1:0 */
  
/* Prescale settings 
 * 这里的值用于设置分频和计算计数值用，二者要对应，不然计算会出错 
 * count = (clock * time)/ div
 * 修改分频值在这里修改
 */
#define HAL_TIMER1_16_PRESCALE      HAL_TIMER1_16_TC_DIV128
#define HAL_TIMER1_16_PRESCALE_VAL  128
  
/* Clock settings */
#define HAL_TIMER_32MHZ             32
  
/* mode setting 
 * 修改计数模式在这里修改 
 */
#define HAL_TIMER1_16_OPMODE        HAL_TIMER1_OPMODE_UP_DOWN
  
/***************************************************************************************************
 *                                             TYPEDEFS
 ***************************************************************************************************/
typedef void (*halEcgMeasCBack_t) (void);

/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize ECG measure timer and port config.
 */
extern void HalEcgMeasInit(void);

/*
 * ECG measure config.
 */
extern void HalEcgMeasConfig( halEcgMeasCBack_t cBack );

/*
 * Start ECG measure. 选择模模式-最大262140us-0.26s 选择up-down模式-最大524280us=0.52s
 * 输入为微秒数
 */
extern void HalEcgMeasStart(uint32 timePerTick);


/*
 * Stop ECG measure.
 */
extern void HalEcgMeasStop(void);

#ifdef __cplusplus
}
#endif  
#endif
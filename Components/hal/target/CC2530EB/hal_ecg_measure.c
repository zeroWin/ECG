/**************************************************************************************************
  Filename:       hal_ecg_measure.c
  Revised:        $Date: 2016-04-05 15:41:16 +0800 (Tues, 5 Apr 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to the ECG measure.


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
#include "hal_timer.h"



#if (defined HAL_ECG_MEASURE) && (HAL_ECG_MEASURE == TRUE)
/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/
#define HW_TIMER_1        0x00
#define HW_TIMER_MAX      0x01

#define IEN1_T1IE     0x02    /* Timer1 Interrupt Enable bit*/
#define TIMIF_T1OVFIM 0x40    /* Timer1 overflow Enable bit*/
  
#define T1STAT_OVFIF  0x20    /* Timer1 overflow interrupt flag */


/* Default all timers to use channel 0 */
#define TCHN_T1CCTL   &(X_T1CCTL0)
#define TCHN_T1CCL    &(X_T1CC0L)
#define TCHN_T1CCH    &(X_T1CC0H)
#define TCNH_T1OVF    &(X_TIMIF)
#define TCHN_T1OVFBIT TIMIF_T1OVFIM
#define TCHN_T1INTBIT IEN1_T1IE

/***************************************************************************************************
 *                                              MACROS
 ***************************************************************************************************/


/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/
typedef struct
{
	bool configured;      //--TRUE
	bool intEnable;       //--enable
	uint8 opMode;         // user choose on macro define
	uint8 channel;        // don't use 
	uint8 channelMode;    // don't use
	uint8 prescale;       // user choose on macro define
	uint8 prescaleVal;    // user choose on macro define
	uint8 clock;          // user choose on macro define
	halEcgMeasCBack_t callBackFunc; // user choose when use api
} halTimerSettings_t;

typedef struct
{
	uint8 volatile XDATA *TxCCTL;
	uint8 volatile XDATA *TxCCH;
	uint8 volatile XDATA *TxCCL;
	uint8 volatile XDATA *TxOVF;
	uint8 ovfbit;
	uint8 intbit;
} halTimerChannel_t;

/**************************************************************************************************
 *                                        INNER GLOBAL VARIABLES
 **************************************************************************************************/
static halTimerSettings_t halTimerRecord[HW_TIMER_MAX];
static halTimerChannel_t  halTimerChannel[HW_TIMER_MAX];

PingPongBuf_t *pingPongBuf_ECG;

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
uint8 halTimerSetCount(uint8 hwtimerid, uint32 timePerTick);
uint8 halTimerSetPrescale(uint8 hwtimerid, uint8 prescale);
uint8 halTimerSetOpMode(uint8 hwtimerid, uint8 opMode);
uint8 HalTimerInterruptEnable(uint8 hwtimerid, uint8 channelMode, bool enable);
void halProcessTimer1Interrupt(void);

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalEcgMeasInit
 *
 * @brief   Initilize ECG measure. Timer and port service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalEcgMeasInit(void)
{
  // 定时器初始化
  /* Setup prescale & clock for timer1 */  
  halTimerRecord[HW_TIMER_1].prescale = HAL_TIMER1_16_PRESCALE;
  halTimerRecord[HW_TIMER_1].clock = HAL_TIMER_32MHZ;
  halTimerRecord[HW_TIMER_1].prescaleVal = HAL_TIMER1_16_PRESCALE_VAL;
  
  /* Setup Timer1 register */
  halTimerChannel[HW_TIMER_1].TxCCTL = TCHN_T1CCTL;   /* 使用溢出模式，该寄存器无用 */
  halTimerChannel[HW_TIMER_1].TxCCL = TCHN_T1CCL;     /* 比较值寄存器 低8位 */
  halTimerChannel[HW_TIMER_1].TxCCH = TCHN_T1CCH;     /* 比较值寄存器 高8位 */
  halTimerChannel[HW_TIMER_1].TxOVF = TCNH_T1OVF;     /* Timers 1/3/4 Interrupt Mask/Flag register bit6为Timer 1 overflow interrupt mask */
  halTimerChannel[HW_TIMER_1].ovfbit = TCHN_T1OVFBIT; /* bit6:TCHN_T1OVFBIT = 0x40 */
  halTimerChannel[HW_TIMER_1].intbit = TCHN_T1INTBIT; /* IEN1-bit2:TCHN_T1INTBIT = 0x02 */
  
  
  // AD8232 端口初始化
  
  
//  //Setting ADC reference volage 所有ADC参考电压由HalAdcInit()统一设置。
//  // 这里保留代码但不使用
//  HalAdcSetReference(ECG_MEASURE_RefVol);
  
  //数据存储空间初始化
  pingPongBuf_ECG = PingPongBufInit(ECG_WAVEFORM_SAMPLER_NUM_PER_PACKET);
}

/***************************************************************************************************
* @fn      HalEcgMeasConfig
*
* @brief   Configure the ECG measure Serivce
*
* @param   cBack - Pointer to the callback function
*
* @return 
***************************************************************************************************/
void HalEcgMeasConfig(halEcgMeasCBack_t cBack)
{
  halTimerRecord[HW_TIMER_1].configured = TRUE;
  halTimerRecord[HW_TIMER_1].opMode = HAL_TIMER1_16_OPMODE;
  halTimerRecord[HW_TIMER_1].intEnable = TRUE;
  halTimerRecord[HW_TIMER_1].callBackFunc = cBack;
}


/***************************************************************************************************
* @fn      HalEcgMeasStart
*
* @brief   Start the Ecg Meas Service
*
* @param   timerPerTick - number of micro sec per tick, (ticks x prescale) / clock = usec/tick
*
* @return  
***************************************************************************************************/
void HalEcgMeasStart(uint32 timePerTick)
{
  if( halTimerRecord[HW_TIMER_1].configured )
  {
    if( halTimerRecord[HW_TIMER_1].opMode == HAL_TIMER1_OPMODE_UP_DOWN )
      halTimerSetCount(HW_TIMER_1,timePerTick/2);
    else
      halTimerSetCount(HW_TIMER_1,timePerTick);
    
    halTimerSetPrescale(HW_TIMER_1,halTimerRecord[HW_TIMER_1].prescale);
    halTimerSetOpMode(HW_TIMER_1,halTimerRecord[HW_TIMER_1].opMode);
    
    // enable interruput
    HalTimerInterruptEnable(HW_TIMER_1, HAL_TIMER_CH_MODE_OVERFLOW, halTimerRecord[HW_TIMER_1].intEnable );
  }
}


/***************************************************************************************************
* @fn      HalEcgMeasStop
*
* @brief   Stop the Ecg Meas Service
*
* @param   timerId - ID of the timer
*
* @return  
***************************************************************************************************/
void HalEcgMeasStop(void)
{
  T1CTL &= ~(HAL_TIMER1_OPMODE_BITS);
  T1CTL |= HAL_TIMER1_OPMODE_STOP;
}


/***************************************************************************************************
* @fn      halTimerSetCount
*
* @brief   Stop the Timer Service
*
* @param   hwtimerid - ID of the timer
*          timerPerTick - Number micro sec per ticks 输入是微秒 10的-6
*
* @return  Status - OK or Not OK
*          copy from timer.c on TI z-stack 2.3.0.
***************************************************************************************************/
uint8 halTimerSetCount(uint8 hwtimerid, uint32 timePerTick)
{
  uint16  count;
  uint8   high, low;

  /* Load count = ((sec/tick) x clock) / prescale */
  count = (uint16)((timePerTick * halTimerRecord[hwtimerid].clock) / halTimerRecord[hwtimerid].prescaleVal);
  high = (uint8)(count >> 8);
  low = (uint8)count;

  *(halTimerChannel[hwtimerid].TxCCH) = high;
  *(halTimerChannel[hwtimerid].TxCCL) = low;

  return HAL_TIMER_OK;
}


/***************************************************************************************************
* @fn      halTimerSetPrescale
*
* @brief   Stop the Timer Service
*
* @param   hwtimerid - ID of the timer
*          prescale - Prescale of the clock
*
* @return  Status - OK or Not OK
*          copy from timer.c on TI z-stack 2.3.0.
***************************************************************************************************/
uint8 halTimerSetPrescale(uint8 hwtimerid, uint8 prescale)
{
	switch (hwtimerid)
	{
	case HW_TIMER_1:
		T1CTL &= ~(HAL_TIMER1_16_TC_BITS);
		T1CTL |= prescale;
		break;
//	case HW_TIMER_3:
//		T3CTL &= ~(HAL_TIMER34_8_TC_BITS);
//		T3CTL |= prescale;
//		break;
//	case HW_TIMER_4:
//		T4CTL &= ~(HAL_TIMER34_8_TC_BITS);
//		T4CTL |= prescale;
		break;
	default:
		return HAL_TIMER_INVALID_ID;
	}
	return HAL_TIMER_OK;
}


/***************************************************************************************************
* @fn      halTimerSetOpMode
*
* @brief   Setup operate modes
*
* @param   hwtimerid - ID of the timer
*          opMode - operation mode of the timer
*
* @return  Status - OK or Not OK
*          copy from timer.c on TI z-stack 2.3.0.
***************************************************************************************************/
uint8 halTimerSetOpMode(uint8 hwtimerid, uint8 opMode)
{
	/* Load Waveform Generation Mode */
	switch (opMode)
	{
	case HAL_TIMER_MODE_NORMAL:
		switch (hwtimerid)
		{
		case HW_TIMER_1:
			T1CTL &= ~(HAL_TIMER1_OPMODE_BITS);
			T1CTL |= HAL_TIMER1_OPMODE_FREERUN;
			break;
//		case HW_TIMER_3:
//			T3CTL &= ~(HAL_TIMER34_OPMODE_BITS);
//			T3CTL |= HAL_TIMER34_OPMODE_FREERUN;
//			break;
//		case HW_TIMER_4:
//			T4CTL &= ~(HAL_TIMER34_OPMODE_BITS);
//			T4CTL |= HAL_TIMER34_OPMODE_FREERUN;
//			break;
		default:
			return HAL_TIMER_INVALID_ID;
		}
		break;

	case HAL_TIMER_MODE_CTC:
		switch (hwtimerid)
		{
		case HW_TIMER_1:
			T1CTL &= ~(HAL_TIMER1_OPMODE_BITS);
			T1CTL |= HAL_TIMER1_OPMODE_MODULO;
			break;
//		case HW_TIMER_3:
//			T3CTL &= ~(HAL_TIMER34_OPMODE_BITS);
//			T3CTL |= HAL_TIMER34_OPMODE_MODULO;
//			break;
//		case HW_TIMER_4:
//			T4CTL &= ~(HAL_TIMER34_OPMODE_BITS);
//			T4CTL |= HAL_TIMER34_OPMODE_MODULO;
//			break;
		default:
			return HAL_TIMER_INVALID_ID;
		}
		break;

//	case HAL_TIMER_MODE_STOP:
//		if (hwtimerid == HW_TIMER_1)
//		{
//			T1CTL &= ~(HAL_TIMER1_OPMODE_BITS);
//			T1CTL |= HAL_TIMER1_OPMODE_STOP;
//		}
//		break;
        case HAL_TIMER1_OPMODE_UP_DOWN:
                if ( hwtimerid == HW_TIMER_1 )
                {
			T1CTL &= ~(HAL_TIMER1_OPMODE_BITS);
			T1CTL |= HAL_TIMER1_OPMODE_UP_DOWN;                  
                }
                break;
	default:
		return HAL_TIMER_INVALID_OP_MODE;
	}
	return HAL_TIMER_OK;
}

/***************************************************************************************************
* @fn      HalTimerInterruptEnable
*
* @brief   Setup operate modes
*
* @param   hwtimerid - ID of the timer
*          channelMode - channel mode
*          enable - TRUE or FALSE
*
* @return  Status - OK or Not OK
*          copy from timer.c on TI z-stack 2.3.0.
***************************************************************************************************/
uint8 HalTimerInterruptEnable(uint8 hwtimerid, uint8 channelMode, bool enable)
{
	switch (channelMode)
	{
	case HAL_TIMER_CH_MODE_OVERFLOW:

		if (enable)
		{
			*(halTimerChannel[hwtimerid].TxOVF) |= halTimerChannel[hwtimerid].ovfbit;
		}
		else
		{
			*(halTimerChannel[hwtimerid].TxOVF) &= ((halTimerChannel[hwtimerid].ovfbit) ^ 0xFF);
		}
		break;

	case HAL_TIMER_CH_MODE_OUTPUT_COMPARE:
	case HAL_TIMER_CH_MODE_INPUT_CAPTURE:

		if (enable)
		{
			//*(halTimerChannel[hwtimerid].TxCCTL) |= T134CCTL_IM;
		}
		else
		{
			//*(halTimerChannel[hwtimerid].TxCCTL) &= ~(T134CCTL_IM);
		}
		break;

	default:
		return HAL_TIMER_INVALID_CH_MODE;
	}

	if (halTimerRecord[hwtimerid].intEnable)
	{
		IEN1 |= halTimerChannel[hwtimerid].intbit;
	}
	else
	{
		IEN1 &= ((halTimerChannel[hwtimerid].intbit) ^ 0xFF);
	}
	return HAL_TIMER_OK;
}


/***************************************************************************************************
* @fn      halProcessTimer1Interrupt()
*
* @brief   Processes Timer 1 Events.
*
* @param
*
* @return
***************************************************************************************************/
void halProcessTimer1Interrupt(void)
{
  if(T1STAT & T1STAT_OVFIF)
  {
    //中断标志自动清除
    if(halTimerRecord[HW_TIMER_1].callBackFunc)
      (halTimerRecord[HW_TIMER_1].callBackFunc)();
  }
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
  
  halProcessTimer1Interrupt();
  
  HAL_EXIT_ISR();
}


#else
void HalEcgMeasInit(void);
void HalEcgMeasConfig( halTimerCBack_t cBack );
void HalEcgMeasStart(uint32 timePerTick);
void HalEcgMeasStop(void);

#endif /* HAL_ECG_MEASURE */
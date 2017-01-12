
/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          wang_1549_pwm.h
* Compiler :     
* Revision :     Revision
* Date :         2017-01-07
* Updated by :   
* Description :  
*                
* 
* LPC11C14 sytem clock: 48Mhz
* system clock: 48MHz
********************************************************************************
* This edition can only be used in DCBoard V6.0 using MornSun DC/DC Module
*******************************************************************************/

#ifndef __WANG_1549_PWM_H_
#define __WANG_1549_PWM_H_

#include "board.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define SCT0_PWM                LPC_SCT0    /* Use SCT0 for PWM */
#define SCT1_PWM                LPC_SCT1    /* Use SCT1 for PWM */
#define SCT2_PWM                LPC_SCT2    /* Use SCT2 for PWM */
#define SCT3_PWM                LPC_SCT3    /* Use SCT3 for PWM */
#define SCT0_PWM0_PIN           0           /* COUT0 SCT0_OUT0  */
#define SCT0_PWM1_PIN           1           /* COUT1 SCT0_OUT1  */
#define SCT0_PWM2_PIN           2           /* COUT2 SCT0_OUT2  */
#define SCT1_PWM0_PIN           0           /* COUT0 SCT1_OUT0  */
#define SCT1_PWM1_PIN           1           /* COUT1 SCT1_OUT1  */
#define SCT1_PWM2_PIN           2           /* COUT2 SCT1_OUT2  */
#define SCT2_PWM0_PIN           0           /* COUT0 SCT2_OUT0  */
#define SCT2_PWM1_PIN           1           /* COUT1 SCT2_OUT1  */
#define SCT2_PWM2_PIN           2           /* COUT2 SCT2_OUT2  */
#define SCT3_PWM0_PIN           0           /* COUT0 SCT3_OUT0  */
#define SCT3_PWM1_PIN           1           /* COUT1 SCT3_OUT1  */
#define SCT3_PWM2_PIN           2           /* COUT2 SCT3_OUT2  */
#define PWM_Event1              1           /* Index of SCT0 PWM0 event  */
#define PWM_Event2              2           /* Index of SCT0 PWM1 event  */
#define PWM_Event3              3           /* Index of SCT0 PWM2 event  */
#define SCT0_PWM_RATE           24660       /* SCT0 PWM frequency 15 KHz */
#define SCT1_PWM_RATE           15000       /* SCT1 PWM frequency 15 KHz */
#define SCT2_PWM_RATE           15000       /* SCT2 PWM frequency 15 KHz */
#define SCT3_PWM_RATE           15000       /* SCT3 PWM frequency 15 KHz */

#define A4935_nEN_LOW           Chip_GPIO_SetPinState(LPC_GPIO,1,17,0)
#define	A4935_nEN_HIGH          Chip_GPIO_SetPinState(LPC_GPIO,1,17,1)
#define A4935_nCOAST_LOW        Chip_GPIO_SetPinState(LPC_GPIO,0,30,0)
#define	A4935_nCOAST_HIGH       Chip_GPIO_SetPinState(LPC_GPIO,0,30,1)

#define LED_RUN_ON              Chip_GPIO_SetPinState(LPC_GPIO,1,25,0)
#define	LED_RUN_OFF             Chip_GPIO_SetPinState(LPC_GPIO,1,25,1)
#define	LED_UV_ON               Chip_GPIO_SetPinState(LPC_GPIO,0,15,0)
#define LED_UV_OFF              Chip_GPIO_SetPinState(LPC_GPIO,0,15,1)
#define	LED_OT_ON               Chip_GPIO_SetPinState(LPC_GPIO,0,16,0)
#define	LED_OT_OFF              Chip_GPIO_SetPinState(LPC_GPIO,0,16,1)
#define LED_SC_ON               Chip_GPIO_SetPinState(LPC_GPIO,1,4,0)
#define LED_SC_OFF              Chip_GPIO_SetPinState(LPC_GPIO,1,4,1)

/*SCTx_OUTx function assignment. The value is the pin 
* number to be assigned to this function. PIO0_0 = 0, ..., 
* PIO1_0 = 32, ..., PIO2_11 = 75  x = 0,1,2
* SCT0_PWM SCT1_PWM SCT2_PWM SCT3_PWM的 0 1 2 可以映射到0端口的任意引脚
*/
#define SCT0_PWM0               73                  /* 配置SCT0 PWM0 输出通道所连的引脚 P2_9 */
//#define SCT0_PWM1             24                  /* 配置SCT0 PWM1 输出通道所连的引脚 P0_3 */
//#define SCT0_PWM2             33                  /* 配置SCT0 PWM2 输出通道所连的引脚 P1_1 */

#define SCT1_PWM0               51                  /* 配置SCT1 PWM0 输出通道所连的引脚 P1_19 */
#define SCT1_PWM1               50                  /* 配置SCT1 PWM1 输出通道所连的引脚 P1_18 */
#define SCT1_PWM2               1                   /* 配置SCT1 PWM2 输出通道所连的引脚 P0_1  */

#define SCT2_PWM0               9                   /* 配置SCT2 PWM0 输出通道所连的引脚 P0_9  */
#define SCT2_PWM1               18                  /* 配置SCT2 PWM1 输出通道所连的引脚 P0_18 */
#define SCT2_PWM2               0                   /* 配置SCT2 PWM2 输出通道所连的引脚 P0_0  */


//unsigned char MotorFlag = 0xFF;
//volatile unsigned char CurSpd = 1;
/**
 * @brief	Converts a percentage to ticks
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	percent	: Percentage to convert (0 - 1000)
 * @return	Number ot ticks corresponding to given percentage
 * @note	Do not use this function when using very low
 *          pwm rate (like 100Hz or less), on a chip that has
 *          very high frequency as the calculation might
 *          cause integer overflow
 */
STATIC INLINE uint32_t SCTPWM_PercentageToTicks(LPC_SCT_T *pSCT, uint16_t percent)
{
    return (Chip_SCTPWM_GetTicksPerCycle(pSCT) * percent) >> 16;
}

/**
 * @brief	Get number of ticks on per PWM cycle
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	index	: Index of the PWM 1 to N (see notes)
 * @param	percent	: Percentage to convert (0 - 1000)
 * @return	None
 * @note	@a index will be 1 to N where N is the "Number of
 *          match registers available in the SCT - 1" or
 *          "Number of OUTPUT pins available in the SCT" whichever
 *          is minimum. The new duty cycle will be effective only
 *          after completion of current PWM cycle.
 */
STATIC INLINE void SCTPWM_SetDutyCycle(LPC_SCT_T *pSCT, uint8_t index, uint16_t percent)
{
    Chip_SCT_SetMatchReload(pSCT, (CHIP_SCT_MATCH_REG_T)index, SCTPWM_PercentageToTicks(pSCT, percent));
}

struct PWMstruct
{
    unsigned long ulTA,ulTB,ulTC;	//脉冲宽度
    unsigned long ulTa,ulTb,ulTc;
};

void SCTPWM_SetOutPin(LPC_SCT_T *pSCT, uint8_t index, uint8_t pin);
void SCTPWM_Init(void);
static long mysine60(unsigned long ulAngle);
static void 
SpaceVectorModulate( unsigned long ulAmplitude, unsigned long ulAngle, struct PWMstruct * pPWMout);
void PWMSetFrequency(uint32_t ulPWMClk);

void MotorStop(void);
void MotorRun(void);

void UART_RXIntHandlerRB1(LPC_USART_T *pUART, RINGBUFF_T *pRB);
#endif /* __BOARD_API_H_ */

// end of the file


/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          wang_1549_adc.h
* Compiler :     
* Revision :     Revision
* Date :         2017-01-12
* Updated by :   
* Description :  
*                
* 
* LPC11C14 sytem clock: 48Mhz
* system clock: 48MHz
********************************************************************************
* This edition can only be used in DCBoard V6.0 using MornSun DC/DC Module
*******************************************************************************/

#ifndef __WANG_1549_ADC_H_
#define __WANG_1549_ADC_H_

#include "board.h"

#if defined(BOARD_KEIL_MCB1500)
/* ADC is connected to the pot */
#define BOARD_ADC_CH 0
#define ANALOG_INPUT_PORT   1
#define ANALOG_INPUT_BIT    1
#define ANALOG_FIXED_PIN    SWM_FIXED_ADC1_0

#elif defined(BOARD_NXP_LPCXPRESSO_1549)
/* ADC is connected to the pot on LPCXPresso base boards */
#define BOARD_ADC_CH0           0
#define BOARD_ADC_CH1           1
#define BOARD_ADC_CH2           2
#define BOARD_ADC_CH3           3
#define BOARD_ADC_CH4           4
#define BOARD_ADC_CH5           5
#define BOARD_ADC_CH6           6
#define BOARD_ADC_CH7           7
#define BOARD_ADC_CH8           8
#define BOARD_ADC_CH9           9
#define BOARD_ADC_CH10          10
#define BOARD_ADC_CH11          11
#define ANALOG_INPUT_PORT       0
#define ANALOG_INPUT_BIT        9
#define ANALOG_FIXED_PIN0       SWM_FIXED_ADC1_0            //P1_1
#define ANALOG_FIXED_PIN1       SWM_FIXED_ADC1_1            //P0_9
#define ANALOG_FIXED_PIN2       SWM_FIXED_ADC1_2            //P0_10
#define ANALOG_FIXED_PIN3       SWM_FIXED_ADC1_3            //P0_11
#define ANALOG_FIXED_PIN4       SWM_FIXED_ADC1_4            //P1_2
#define ANALOG_FIXED_PIN5       SWM_FIXED_ADC1_5            //P1_3
#define ANALOG_FIXED_PIN6       SWM_FIXED_ADC1_6            //P0_13
#define ANALOG_FIXED_PIN7       SWM_FIXED_ADC1_7            //P0_14
#define ANALOG_FIXED_PIN8       SWM_FIXED_ADC1_8            //P0_15
#define ANALOG_FIXED_PIN9       SWM_FIXED_ADC1_9            //P0_16
#define ANALOG_FIXED_PIN10      SWM_FIXED_ADC1_10           //P1_4
#define ANALOG_FIXED_PIN11      SWM_FIXED_ADC1_11           //P1_5

#else
#warning "Using ADC channel 8 for this example, please select for your board"
#define BOARD_ADC_CH 8
#endif

//uint16_t AD_Value[2][3];

void showValudeADC(LPC_ADC_T *pADC);
void ADC_Init();

#endif

// end of the file

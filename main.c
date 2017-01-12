
/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          main.c
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

#include "board.h"
#include <stdio.h>
#include "wang_1549_adc.h"
#include "wang_1549_pwm.h"
#include "wang_1549_spi.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

static volatile int ticks;
#define TICKRATE_HZ (1000)      /* 100 ticks per second */

#define UART_RB_SIZE 64

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

static bool sequence0Complete, sequence1Complete, threshold1Crossed;

/*****************************************************************************
 * Public functions
 ****************************************************************************/

static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];
const char inst1[] = "000000000000000\r\n";
const char inst2[] = "000000000000000\r\n";
uint8_t data[8] = {0};
/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* UART Pin mux function - note that SystemInit() may already setup your
   pin muxing at system startup */
static void Init_UART_PinMux(void)
{
#if defined(BOARD_KEIL_MCB1500)
    /* UART signals on pins PIO2_6 (FUNC0, U0_TXD) and PIO2_7 (FUNC0, U0_RXD) */
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 7, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 6, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));

    /* UART signal muxing via SWM */
    Chip_SWM_MovablePortPinAssign(SWM_UART0_RXD_I, 2, 7);
    Chip_SWM_MovablePortPinAssign(SWM_UART0_TXD_O, 2, 6);

#elif defined(BOARD_NXP_LPCXPRESSO_1549)
    /* UART signals on pins PIO0_13 (FUNC0, U0_TXD) and PIO0_18 (FUNC0, U0_RXD) */
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 7, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
    Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 6, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));

    /* UART signal muxing via SWM */
    Chip_SWM_MovablePortPinAssign(SWM_UART0_RXD_I, 2, 7);
    Chip_SWM_MovablePortPinAssign(SWM_UART0_TXD_O, 2, 6);

#else
#warning "No UART nmuxing defined for this example"
#endif
}

void GPIO_Init(void)
{
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,1,17);           //A4935_nEN
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,0,30);           //A4935_nCOAST
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,1,25);           //LED_RUN
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,0,15);           //LED_UV
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,0,16);           //LED_OT
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,1,4);            //LED_SC
    Chip_GPIO_SetPinDIRInput(LPC_GPIO,0,10);            //
    Chip_GPIO_SetPinDIRInput(LPC_GPIO,1,1);             //
    
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,0,17);
    Chip_GPIO_SetPinState(LPC_GPIO,0,17,0);

    LED_OT_OFF;
    LED_SC_OFF;
    LED_UV_OFF;
    LED_RUN_OFF;
    
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 0, 
        (IOCON_MODE_INACT | IOCON_FUNC0));
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, 
        (IOCON_MODE_INACT | IOCON_FUNC0));
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 9, 
        (IOCON_MODE_INACT | IOCON_FUNC0));
    
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,0,0);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,0,18);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO,0,9);
    Chip_GPIO_SetPinState(LPC_GPIO,0,0,1);
    Chip_GPIO_SetPinState(LPC_GPIO,0,18,1);
    Chip_GPIO_SetPinState(LPC_GPIO,0,9,1);    
     
    //配置 端口输出
//    Chip_GPIO_SetPinDIROutput(LPC_GPIO,0,1);
//    Chip_GPIO_SetPinDIROutput(LPC_GPIO,1,18);
//    Chip_GPIO_SetPinDIROutput(LPC_GPIO,1,19);
//    Chip_GPIO_SetPinState(LPC_GPIO,0,1,0);
//    Chip_GPIO_SetPinState(LPC_GPIO,1,18,0);
//    Chip_GPIO_SetPinState(LPC_GPIO,1,19,0);     
}

void UART_IntEnable()
{
    Init_UART_PinMux();
    /* Before using the ring buffers, initialize them using the ring
        buffer init function */
    RingBuffer_Init(&rxring, rxbuff, 1, 8);
    RingBuffer_Init(&txring, txbuff, 1, 8);
    /* Enable receive data and line status interrupt */
    Chip_UART_IntEnable(DEBUG_UART, UART_INTEN_RXRDY);
    Chip_UART_IntDisable(DEBUG_UART, UART_INTEN_TXRDY);	/* May not be needed */
    
    /* Enable UART interrupt */
    NVIC_EnableIRQ(UART0_IRQn);
}

/**
 * @brief	Handle interrupt from ADC0 sequencer A
 * @return	Nothing
 */
void ADC0A_IRQHandler(void)
{
    uint32_t pending;

    /* Get pending interrupts */
    pending = Chip_ADC_GetFlags(LPC_ADC0);

    /* Sequence A completion interrupt */
    if (pending & ADC_FLAGS_SEQA_INT_MASK) {
        sequence0Complete = true;
    }

    /* Clear any pending interrupts */
    Chip_ADC_ClearFlags(LPC_ADC0, pending);
}

/**
 * @brief	Handle interrupt from ADC1 sequencer A
 * @return	Nothing
 */
void ADC1A_IRQHandler(void)
{
    uint32_t pending;

    /* Get pending interrupts */
    pending = Chip_ADC_GetFlags(LPC_ADC1);

    /* Sequence A completion interrupt */
    if (pending & ADC_FLAGS_SEQA_INT_MASK) {
        sequence1Complete = true;
    }

    /* Clear Sequence A completion interrupt */
    Chip_ADC_ClearFlags(LPC_ADC1, ADC_FLAGS_SEQA_INT_MASK);
}

/**
 * @brief	Handle threshold interrupt from ADC1
 * @return	Nothing
 */
void ADC1_THCMP_IRQHandler(void)
{
    uint32_t pending;

    /* Get pending interrupts */
    pending = Chip_ADC_GetFlags(LPC_ADC1);

    /* Threshold crossing interrupt on ADC input channel */
    if (pending & ADC_FLAGS_THCMP_MASK(BOARD_ADC_CH1)) {
        threshold1Crossed = true;
    }

    /* Clear threshold interrupt */
    Chip_ADC_ClearFlags(LPC_ADC1, ADC_FLAGS_THCMP_MASK(BOARD_ADC_CH1));
}

/**
 * @brief	main routine for ADC example
 * @return	Function should not exit
 */
int main(void)
{
    SystemCoreClockUpdate();
    Board_Init();
    //串口中断初始化
    UART_IntEnable();
    //端口初始化
    GPIO_Init();
    //PWM初始化
    
    SCTPWM_Init();
    //初始化ADC
    //ADC_Init();
    SysTick_Config(SystemCoreClock / TICKRATE_HZ);
    
    PWMSetFrequency(20000);               //PWM 频率 14850  
    MotorRun();
    
    /* Endless loop */
    while (1) 
    {
        /* Sleep until something happens */
        __WFI();
        
        if (threshold1Crossed) {
            threshold1Crossed = false;
            DEBUGSTR("********ADC1 threshold event********\r\n");
        }

        /* Is a conversion sequence complete? */
        if (sequence0Complete) {
            sequence0Complete = false;
            showValudeADC(LPC_ADC0);
        }
        
        if (sequence1Complete) {
            sequence1Complete = false;
            showValudeADC(LPC_ADC1);     
        }
    }
}

// end of the file

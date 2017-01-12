
/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          wang_1549_adc.c
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

/*******************************************************
*十二路PWM SCT0 SCT1 SCT2 SCT3 各三路0 1 2
*
*
*******************************************************/
#include "wang_1549_adc.h"

uint16_t AD_Value[2][3];
/*****************************************************************************
 * Private functions
 ****************************************************************************/
//读取12个通道的值
void showValudeADC(LPC_ADC_T *pADC)
{
    int index, j;
    uint32_t rawSample;

    if (pADC == LPC_ADC0) 
    {
        index = 0;
    }
    else 
    {
        index = 1;
    }

    /* Get raw sample data for channels 0-11 */
    for (j = 0; j < 12; j++) 
    {
        rawSample = Chip_ADC_GetDataReg(pADC, j);       //读取ADCp 通道j的值
        /* Show some ADC data */
        //								溢出位    				数据有效位
        if ((rawSample & (ADC_DR_OVERRUN | ADC_SEQ_GDAT_DATAVALID)) != 0) 
        {
            DEBUGOUT("ADC%d_%d: Sample value = 0x%x (Data sample %d)\r\n", index, j,
                    ADC_DR_RESULT(rawSample), j);
        }
    }
}



void ADC_Init()
{
    //初始化ADC外设 设置12位ADC和正常功率
    /* Setup ADC for 12-bit mode and normal power */
    Chip_ADC_Init(LPC_ADC0, 0);
    //Chip_ADC_Init(LPC_ADC1, 0);

    /* Setup for maximum ADC clock rate */
    //ADC最大时钟频率设置？
    Chip_ADC_SetClockRate(LPC_ADC0, ADC_MAX_SAMPLE_RATE);
    //Chip_ADC_SetClockRate(LPC_ADC1, ADC_MAX_SAMPLE_RATE);

    /* For ADC0, seqeucner A will be used without threshold events.
        It will be triggered manually by the sysTick interrupt and
        only monitor the internal temperature sensor. */
    /* 对于ADC0，seqeucner A 将被用于无阈值事件 手动触发sysTick中断
        只能检测内部温度传感器 */
    //
    Chip_ADC_SetupSequencer(LPC_ADC0, ADC_SEQA_IDX, (ADC_SEQ_CTRL_CHANSEL(1) |
													 ADC_SEQ_CTRL_CHANSEL(2) |
													 ADC_SEQ_CTRL_CHANSEL(4) |
													 ADC_SEQ_CTRL_CHANSEL(5) |
													 ADC_SEQ_CTRL_CHANSEL(6) |
													 ADC_SEQ_CTRL_MODE_EOS));

    /* Power up the internal temperature sensor */
    //内部温度传感器上电
//	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_TS_PD);

    /* For ADC0, select temperature sensor for channel 0 on ADC0 */
//	Chip_ADC_SetADC0Input(LPC_ADC0, ADC_INSEL_TS);

    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 2, 
        (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
    
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 3, 
        (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
    	
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 4, 
        (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
    
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6, 
        (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
    
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 7, 
        (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
     
    //通过SWM分配ADC0_1到p0_9
    Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_1);
    Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_2);
    Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_4);
    Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_5);
    Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_6);
    

    /* Use higher voltage trim for both ADCs */
    //使用高电压调整ADC？
    Chip_ADC_SetTrim(LPC_ADC0, ADC_TRIM_VRANGE_HIGHV);
    //Chip_ADC_SetTrim(LPC_ADC1, ADC_TRIM_VRANGE_HIGHV);

    /* For ADC1, sequencer A will be used with threshold events.
        It will be triggered manually by the sysTick interrupt and
        only monitors the ADC1 input. */
    //设置ADC1转换序列A 所有通道 转换完成中断
//	Chip_ADC_SetupSequencer(LPC_ADC1, ADC_SEQA_IDX,
//							(ADC_SEQ_CTRL_CHANSEL_MASK |  ADC_SEQ_CTRL_MODE_EOS));
    //设置ADC1转换序列A 通道0-3 转换完成中断						
    Chip_ADC_SetupSequencer(LPC_ADC1, ADC_SEQA_IDX,
                            (0x031 |  ADC_SEQ_CTRL_MODE_EOS));
    //设置ADC1转换序列 通道0 通道1转换完成中断
//	Chip_ADC_SetupSequencer(LPC_ADC1, ADC_SEQA_IDX,
//							(ADC_SEQ_CTRL_CHANSEL(BOARD_ADC_CH1) | ADC_SEQ_CTRL_MODE_EOS));
    /* Disables pullups/pulldowns and disable digital mode */
    //禁用上拉/下拉 禁用数字模式
    //ADC1_0  P1_1
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 1, 
//		(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
    //禁用上拉下拉数字模式 ADC1_0 P1_1
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 1, 
//		(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
//	//ADC1_4 P1_2
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 2, 
//		(IOCON_MODE_INACT | IOCON_DIGMODE_EN));
//	//ADC1_5 P1_3	
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 3, 
//		(IOCON_MODE_INACT | IOCON_DIGMODE_EN));

    /* Assign ADC1_0 to PIO1_1 via SWM (fixed pin) */
    //通过SWM分配ADC1_1到p0_9
//	Chip_SWM_EnableFixedPin(ANALOG_FIXED_PIN0);
//	Chip_SWM_EnableFixedPin(ANALOG_FIXED_PIN4);
//	Chip_SWM_EnableFixedPin(ANALOG_FIXED_PIN5);

    /* Need to do a calibration after initialization and trim */
    //初始化和调整后需要校准
    Chip_ADC_StartCalibration(LPC_ADC0);
    //Chip_ADC_StartCalibration(LPC_ADC1);
    //等待校准完成
    while (!(Chip_ADC_IsCalibrationDone(LPC_ADC0))) {}
    //while (!(Chip_ADC_IsCalibrationDone(LPC_ADC1))) {}

    /* Setup threshold 0 low and high values to about 25% and 75% of max for
        ADC1 only */
    //Chip_ADC_SetThrLowValue(LPC_ADC1, 0, ((1 * 0xFFF) / 4));
    //Chip_ADC_SetThrHighValue(LPC_ADC1, 0, ((3 * 0xFFF) / 4));

    /* Clear all pending interrupts */
    Chip_ADC_ClearFlags(LPC_ADC0, Chip_ADC_GetFlags(LPC_ADC0));
    //Chip_ADC_ClearFlags(LPC_ADC1, Chip_ADC_GetFlags(LPC_ADC1));

    /* Enable sequence A completion interrupts for ADC0 */
    //使能中断
//	Chip_ADC_EnableInt(LPC_ADC0, ADC_INTEN_SEQA_ENABLE);
    Chip_ADC_EnableInt(LPC_ADC0, ADC_INTEN_SEQA_ENABLE |
							 ADC_INTEN_CMP_ENABLE(ADC_INTEN_CMP_CROSSTH, BOARD_ADC_CH1));
    
    Chip_ADC_EnableInt(LPC_ADC0, ADC_INTEN_SEQA_ENABLE |
							 ADC_INTEN_CMP_ENABLE(ADC_INTEN_CMP_CROSSTH, BOARD_ADC_CH2));

    Chip_ADC_EnableInt(LPC_ADC0, ADC_INTEN_SEQA_ENABLE |
							 ADC_INTEN_CMP_ENABLE(ADC_INTEN_CMP_CROSSTH, BOARD_ADC_CH4));
    
    Chip_ADC_EnableInt(LPC_ADC0, ADC_INTEN_SEQA_ENABLE |
							 ADC_INTEN_CMP_ENABLE(ADC_INTEN_CMP_CROSSTH, BOARD_ADC_CH5));

    Chip_ADC_EnableInt(LPC_ADC0, ADC_INTEN_SEQA_ENABLE |
							 ADC_INTEN_CMP_ENABLE(ADC_INTEN_CMP_CROSSTH, BOARD_ADC_CH6));
    

    /* Enable sequence A completion and threshold crossing interrupts for ADC1_1 */
//	Chip_ADC_EnableInt(LPC_ADC1, ADC_INTEN_SEQA_ENABLE |
//					   ADC_INTEN_CMP_ENABLE(ADC_INTEN_CMP_CROSSTH, BOARD_ADC_CH0));
//		
//	Chip_ADC_EnableInt(LPC_ADC1, ADC_INTEN_SEQA_ENABLE |
//					   ADC_INTEN_CMP_ENABLE(ADC_INTEN_CMP_CROSSTH, BOARD_ADC_CH1));
//		
//	Chip_ADC_EnableInt(LPC_ADC1, ADC_INTEN_SEQA_ENABLE |
//					   ADC_INTEN_CMP_ENABLE(ADC_INTEN_CMP_DISBALE, BOARD_ADC_CH2));
    

    /* Use threshold 0 for ADC channel and enable threshold interrupt mode for
        channel as crossing */
//	Chip_ADC_SelectTH0Channels(LPC_ADC1, ADC_THRSEL_CHAN_SEL_THR1(BOARD_ADC_CH1));
//	Chip_ADC_SetThresholdInt(LPC_ADC1, BOARD_ADC_CH1, ADC_INTEN_THCMP_CROSSING);
//	
//	Chip_ADC_SelectTH0Channels(LPC_ADC1, ADC_THRSEL_CHAN_SEL_THR1(BOARD_ADC_CH0));
//	Chip_ADC_SetThresholdInt(LPC_ADC1, BOARD_ADC_CH0, ADC_INTEN_THCMP_CROSSING);

    /* Enable related ADC NVIC interrupts */
    //使能ADC中断
    NVIC_EnableIRQ(ADC0_SEQA_IRQn);
    //NVIC_EnableIRQ(ADC1_SEQA_IRQn);
    //NVIC_EnableIRQ(ADC1_THCMP);

    /* Enable sequencers */
    //使能序列中断
    Chip_ADC_EnableSequencer(LPC_ADC0, ADC_SEQA_IDX);
    //Chip_ADC_EnableSequencer(LPC_ADC1, ADC_SEQA_IDX);
}

// end of the file

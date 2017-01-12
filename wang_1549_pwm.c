
/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          wang_1549_pwm.c
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
#include "wang_1549_pwm.h"

bool Rx_read;
bool RUN_START;
bool ErrSc;
bool ErrOt;
bool ErrUv;
bool ErrStop;
bool TxReady;

static struct PWMstruct sPWMout;
//最小脉宽
static unsigned long g_ulMinPulseWidth = 100;
uint8_t UartData[9] = {0};
uint8_t	TxDataBuff[8] = {0xAA,0,0,0,0,0,0,0};
uint8_t TxBuff[8],RxBuff[8];
uint8_t OK[] = {0xff,0xff,0xff};
uint16_t counter_data = 6;

static uint16_t Ir = 0x1D5;
static uint8_t motorstate = 0xD1,motorspeed = 6,motorddirection = 0x3c;
static uint8_t self_checking = 0,masterstate = 0,motormoder = 0;
static uint8_t	hardwareversion = 0x05,softwareversion = 0x05;
static uint32_t UF_time = 0;
uint8_t check;          //00 切换到自检状态 01 下一步 02 系统维护	
/* Setup the OUTPUT pin corresponding to the PWM index */
void SCTPWM_SetOutPin(LPC_SCT_T *pSCT, uint8_t index, uint8_t pin)
{
    int ix = (int) index;
    pSCT->EVENT[ix].CTRL = ix |(1 << 12);                   //事件ix为配置触发事件，触发来源于匹配寄存器index
    pSCT->EVENT[ix].STATE = 1 << 0;	                        //启动事件ix
    pSCT->OUT[pin].SET = (1 << ix);	                        //事件ix 触发引脚输出  事件ix触发引脚输出高电平
    pSCT->OUT[pin].CLR = (1 << 0) |(1 << (ix + 3));	        //事件 0和事件(ix+3) 清除引脚输出	事件0 触发引脚输出清零 拉低引脚

    /* Clear the output in-case of conflict */
    pSCT->RES = (pSCT->RES & ~(3 << (pin << 1))) | (0x01 << (pin << 1));

    /* Set and Clear do not depend on direction */
    pSCT->OUTPUTDIRCTRL = (pSCT->OUTPUTDIRCTRL & ~(3 << (pin << 1)));
}

/**
 * @brief	Initialize the all SCT/PWM clock and reset  PWM频率初始值设置
 * @param	None
 * @return	None
 */
void SCTPWM_Init(void)
{
    //Chip_SCTPWM_Init(SCT0_PWM);
    Chip_SCTPWM_Init(SCT1_PWM);
    //Chip_SCTPWM_Init(SCT2_PWM);
    //Chip_SCTPWM_Init(SCT3_PWM);
    
    //Chip_SCTPWM_SetRate(SCT0_PWM, SCT0_PWM_RATE);     //初始化计数匹配计数器0及SCT0
    Chip_SCTPWM_SetRate(SCT1_PWM, SCT1_PWM_RATE);       //初始化计数匹配计数器0及SCT1
    //Chip_SCTPWM_SetRate(SCT2_PWM, SCT2_PWM_RATE);     //初始化计数匹配计数器0及SCT2   //下桥臂 4935
    //Chip_SCTPWM_SetRate(SCT3_PWM, SCT3_PWM_RATE);     //初始化计数匹配计数器0及SCT3
    Chip_SCT_EnableEventInt(SCT1_PWM,SCT_EVT_0);
    //Chip_SCT_EnableEventInt(SCT3_PWM,1);
    NVIC_EnableIRQ(SCT1_IRQn);
    NVIC_EnableIRQ(SCT3_IRQn);
    /* Enable SWM clock before altering SWM */
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);     //启动SWM时钟

#if defined(BOARD_NXP_LPCXPRESSO_1549)
    /* Connect SCT output 1 to PIO0_29 */
    //SCT0_PWM SCT1_PWM SCT2_PWM SCT3_PWM的 0 1 2 可以映射到0端口的任意引脚
    Chip_SWM_MovablePinAssign(SWM_SCT0_OUT0_O, SCT0_PWM0);      //配置SCT0_PWM0输出通道所连的引脚P1_7  39
    
    Chip_SWM_MovablePinAssign(SWM_SCT1_OUT0_O, SCT1_PWM0);      //SCT1_PWM 输出引脚P1_4 35
    Chip_SWM_MovablePinAssign(SWM_SCT1_OUT1_O, SCT1_PWM1);      //P1_5
    Chip_SWM_MovablePinAssign(SWM_SCT1_OUT2_O, SCT1_PWM2);      //P1_6
//  Chip_SWM_EnableFixedPin(SWM_FIXED_SCT1_OUT3);                //pin P0_2
//  Chip_SWM_EnableFixedPin(SWM_FIXED_SCT1_OUT4);                //pin P0_3
//  Chip_SWM_EnableFixedPin(SWM_FIXED_SCT1_OUT5);                //pin P0_14
    //4935 电机驱动 SCT2 PWM引脚外设配置
//  Chip_SWM_MovablePinAssign(SWM_SCT2_OUT0_O, SCT2_PWM0);       //P0_24
//  Chip_SWM_MovablePinAssign(SWM_SCT2_OUT1_O, SCT2_PWM1);       //P0_25
//  Chip_SWM_MovablePinAssign(SWM_SCT2_OUT2_O, SCT2_PWM2);       //P0_26
#endif

    Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);  //配置完成，关闭SWM时钟
    
    Chip_SCTPWM_SetOutPin(SCT0_PWM, PWM_Event1, SCT0_PWM0_PIN);     //配置SCT0事件1 通道0?
    Chip_SCTPWM_SetOutPin(SCT0_PWM, PWM_Event2, SCT0_PWM1_PIN);     //配置SCT0事件2 通道1
//  Chip_SCTPWM_SetOutPin(SCT0_PWM, PWM_Event3, SCT0_PWM2_PIN);	    //配置SCT0事件3 通道2
    
    Chip_SCTPWM_SetOutPin(SCT1_PWM, PWM_Event1, SCT1_PWM0_PIN);     //配置SCT1事件1 通道0?
    Chip_SCTPWM_SetOutPin(SCT1_PWM, PWM_Event2, SCT1_PWM1_PIN);     //配置SCT1事件2 通道1
    Chip_SCTPWM_SetOutPin(SCT1_PWM, PWM_Event3, SCT1_PWM2_PIN);     //配置SCT1事件3 通道2

    SCT1_PWM->EVENT[4].CTRL = 4 | (1 << 12);
    SCT1_PWM->EVENT[4].STATE = 1;
    SCT1_PWM->EVENT[5].CTRL = 4 | (1 << 12);
    SCT1_PWM->EVENT[5].STATE = 1;
    SCT1_PWM->EVENT[6].CTRL = 4 | (1 << 12);
    SCT1_PWM->EVENT[6].STATE = 1;
    
//  SCTPWM_SetOutPin(SCT2_PWM, PWM_Event1, SCT2_PWM0_PIN);       //配置SCT2事件1 通道0?
//  SCTPWM_SetOutPin(SCT2_PWM, PWM_Event2, SCT2_PWM1_PIN);       //配置SCT2事件2 通道1
//  SCTPWM_SetOutPin(SCT2_PWM, PWM_Event3, SCT2_PWM2_PIN);       //配置SCT2事件3 通道2
//  
//  SCT2_PWM->EVENT[4].CTRL = 4 | (1 << 12);
//  SCT2_PWM->EVENT[4].STATE = 1;
//  SCT2_PWM->EVENT[5].CTRL = 4 | (1 << 12);
//  SCT2_PWM->EVENT[5].STATE = 1;
//  SCT2_PWM->EVENT[6].CTRL = 4 | (1 << 12);
//  SCT2_PWM->EVENT[6].STATE = 1;
    
    /* Start with 0% duty cycle PWM占空比初始值为30 */
    Chip_SCTPWM_SetDutyCycle(SCT0_PWM, PWM_Event1,
                                Chip_SCTPWM_PercentageToTicks(SCT0_PWM, 50));
    Chip_SCTPWM_SetDutyCycle(SCT0_PWM, PWM_Event2, 
                                Chip_SCTPWM_PercentageToTicks(SCT0_PWM, 55));
    
    Chip_SCTPWM_SetDutyCycle(SCT1_PWM, PWM_Event1, 
                                Chip_SCTPWM_PercentageToTicks(SCT1_PWM, 30));
    Chip_SCTPWM_SetDutyCycle(SCT1_PWM, PWM_Event2, 
                                Chip_SCTPWM_PercentageToTicks(SCT1_PWM, 30));
    Chip_SCTPWM_SetDutyCycle(SCT1_PWM, PWM_Event3, 
                                Chip_SCTPWM_PercentageToTicks(SCT1_PWM, 30));
    //4935电机驱动使用 下桥臂
//	Chip_SCTPWM_SetDutyCycle(SCT2_PWM, PWM_Event1, 
//				Chip_SCTPWM_PercentageToTicks(SCT2_PWM, 30));
//	Chip_SCTPWM_SetDutyCycle(SCT2_PWM, PWM_Event2, 
//				Chip_SCTPWM_PercentageToTicks(SCT2_PWM, 30));
//	Chip_SCTPWM_SetDutyCycle(SCT2_PWM, PWM_Event3, 
//				Chip_SCTPWM_PercentageToTicks(SCT2_PWM, 30));
    
    Chip_SCTPWM_Start(SCT0_PWM);
    Chip_SCTPWM_Start(SCT1_PWM);
    //Chip_SCTPWM_Start(SCT2_PWM);
}

//减速系数                                
const unsigned long FactorTab[2][14] = { {1, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 1},
                                         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

//?细分步进                                   
const unsigned long AddStepTab[2][14] = { {10, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 24},
                                          {10, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 24}};                                    

//电流 第二次 降低低速时电流
const unsigned long IrTab[2][14] = {{0x1A0, 0x1A0, 0x1BA, 0x1D5, 0x205, 0x240, 0x27A, 0x2B0, 0x2E0, 0x330, 0x380, 0x3C0, 0x400, 0x400},
                                    {0x120, 0x130, 0x120, 0x110, 0x130, 0x138, 0x140, 0x158, 0x178, 0x190, 0x1A8, 0xF0, 0xF0, 0xF0}};

//电机速度                              
volatile unsigned char CurSpd = 1;
unsigned char MotorFlag = 0;

//!*****************************************************************************
//
// A table of the value of the sine function for the 0 - 60 degrees with
// 241 entries (that is, [0] = 0 degrees, [241] = 60 degrees, Q2). Return is
// in 0.16 fixed point format.
// Created by FENG XQ 2009-05-23
//!*****************************************************************************
static const unsigned short g_pusFixedSine60Table[] = {
    0,                                                            //0=0 DEGREE
    286,  572,  858,  1144, 1430, 1716, 2001, 2287, 2573, 2859,   //1-10
    3144, 3430, 3715, 4001, 4286, 4572, 4857, 5142, 5427, 5712,  
    5997, 6281, 6566, 6850, 7135, 7419, 7703, 7987, 8271, 8554,
    8838, 9121, 9404, 9687, 9970, 10252,10534,10817,11098,11380,
    11662,11943,12224,12505,12785,13066,13346,13626,13905,14185,  //51-60
    14464,14742,15021,15299,15577,15855,16132,16409,16686,16962,
    17238,17514,17789,18064,18339,18613,18887,19161,19434,19707,
    19980,20252,20524,20795,21066,21336,21607,21876,22146,22415,
    22683,22951,23219,23486,23753,24019,24285,24550,24815,25080,
    25343,25607,25870,26132,26394,26656,26917,27177,27437,27697,  //101-110
    27956,28214,28472,28729,28986,29242,29498,29753,30007,30261,
    30515,30767,31019,31271,31522,31772,32022,32271,32520,32768,
    33015,33262,33508,33754,33998,34242,34486,34729,34971,35212,
    35453,35693,35933,36172,36410,36647,36884,37120,37355,37590,
    37824,38057,38289,38521,38752,38982,39212,39441,39669,39896,  //151-160
    40122,40348,40573,40797,41021,41243,41465,41686,41906,42126,
    42344,42562,42779,42995,43211,43425,43639,43852,44064,44275,
    44486,44695,44904,45112,45319,45525,45730,45935,46138,46341,
    46543,46744,46944,47143,47341,47538,47735,47930,48125,48318,
    48511,48703,48894,49084,49273,49461,49648,49834,50019,50203,  //201-210
    50387,50569,50751,50931,51111,51289,51467,51643,51819,51993,
    52167,52339,52511,52682,52851,53020,53187,53354,53519,53684,
    53847,54010,54171,54332,54491,54650,54807,54963,55118,55273,
    55426,55578,55729,55879,56028,56175,56322,56468,56612,56756   //240=60 DEGREE
};

//!*****************************************************************************
//! Computes an approximation of the sine of the input angle 0-60 degree.
//! \param ulAngle is an angle expressed as a Q4 format.
//! \return Returns the sine of the angle, in Q16 fixed point format.
//! Created by FENG XQ 2009-05-22
//!*****************************************************************************
static long mysine60(unsigned long ulAngle)
{
    unsigned long ulIdx;
    ulIdx = (ulAngle + 2) >> 2;
    ulIdx = g_pusFixedSine60Table[ulIdx];
    return (ulIdx);
}

//!*****************************************************************************
//! Computes space vector modulated waveforms.
//! \param ulAngle is the current angle of the waveform expressed as a Q4 format in degree.
//! \param ulAmplitude is the amplitude of the waveform, as a Q12 fixed point value.
//! \param pPWMout is a pointer to three unsigned longs to be filled in with the pulse width 
//! of the waveforms.
//! Created by FENG XQ 2009-05-22
//!*****************************************************************************
static void
SpaceVectorModulate( unsigned long ulAmplitude, unsigned long ulAngle, struct PWMstruct * pPWMout)
{
    unsigned long ulSector, ulSine1, ulSine2, ulExtra, ulTemp1, ulTemp2;

    // Clip the amplitude to a maximum of one.
    if (ulAmplitude >= 0xFFF)
    {
        ulAmplitude = 0xFFF;
    }

    ulSector = (unsigned long) ulAngle / 960;               //计算扇区 , 0-5
    ulAngle %= 960;                                         //0-60 degree
     
    ulTemp1 = mysine60(ulAngle);                            //ulAngle=Q4   return sin0-sin60 in Q16  
    ulTemp2 = mysine60(960 - ulAngle);
   
    ulSine1 = (ulTemp1 * ulAmplitude + 0x800) >>12;         //Q16
    ulSine2 = (ulTemp2 * ulAmplitude + 0x800) >>12;         //Q16
    
//		DEBUGOUT("ulSine1 = 0x%x \r\n", ulSine2);
    // Determine the percentage of remaining time to be split between the two zero states.
    ulExtra = (65536 - ulSine1 - ulSine2) / 2;

    // Compute the pulse width based on the sector.
    switch(ulSector)
    {
        // Sector zero resides between 0 and 60 degrees.
        case 0:
        {
            // The vector sequence is 0, 1, 2, 7, 2, 1, 0.
            pPWMout->ulTA=(ulExtra*62259 + 65536) >> 17;  //U ?????
                        pPWMout->ulTa= (65536 - pPWMout->ulTA);	
            pPWMout->ulTB=((ulSine2 + ulExtra)*62259 + 65536) >> 17;                    //V
                        pPWMout->ulTb= (65536 - pPWMout->ulTB);
            pPWMout->ulTC=((ulSine1 + ulSine2 + ulExtra)*62259 + 65536) >> 17;          //W
                        pPWMout->ulTc= (65536 - pPWMout->ulTC);
            break;
        }
        
        // Sector one resides between 60 and 120 degrees.
        case 1:
        {
            // The vector sequence is 0, 3, 2, 7, 2, 3, 0.
            pPWMout->ulTA=((ulSine1 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTa= (65536 - pPWMout->ulTA);
            pPWMout->ulTB= (ulExtra*62259 + 65536) >> 17;
                        pPWMout->ulTb= 65536 - pPWMout->ulTB;
            pPWMout->ulTC= ((ulSine1 + ulSine2 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTc= 65536 - pPWMout->ulTC;
            break;
        }

        // Sector two resides between 120 and 180 degrees.
        case 2:
        {
            // The vector sequence is 0, 3, 4, 7, 4, 3, 0.
            pPWMout->ulTA=((ulSine1 + ulSine2 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTa= 65536 - pPWMout->ulTA;
            pPWMout->ulTB=(ulExtra*62259 + 65536) >> 17;
                        pPWMout->ulTb= 65536 - pPWMout->ulTB;
            pPWMout->ulTC=((ulSine2 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTc= 65536 - pPWMout->ulTC;
            break;
        }

        // Sector three resides between 180 and 240 degrees.
        case 3:
        {
            // The vector sequence is 0, 5, 4, 7, 4, 5, 0.
            pPWMout->ulTA=((ulSine1 + ulSine2 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTa= 65536 - pPWMout->ulTA;
            pPWMout->ulTB=((ulSine1 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTb= 65536 - pPWMout->ulTB;
            pPWMout->ulTC=(ulExtra*62259 + 65536) >> 17;
                        pPWMout->ulTc= 65536 - pPWMout->ulTC;
            break;
        }

        // Sector four resides between 240 and 300 degress.
        case 4:
        {
            // The vector sequence is 0, 5, 6, 7, 6, 5, 0.
            pPWMout->ulTA=((ulSine2 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTa= 65536 - pPWMout->ulTA;
            pPWMout->ulTB=((ulSine1 + ulSine2 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTb= 65536 - pPWMout->ulTB;
            pPWMout->ulTC=((ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTc= 65536 - pPWMout->ulTC;
            break;
        }
        
        // Sector five resides between 300 and 360 degrees.  This is also the
        // default case, for angles larger than 360 degrees (which should not
        // occur, but just in case).
        case 5:
        default:
        {
            // The vector sequence is 0, 1, 6, 7, 6, 1, 0.
            pPWMout->ulTA=(ulExtra*62259 + 65536) >> 17;
                        pPWMout->ulTa= 65536 - pPWMout->ulTA;
            pPWMout->ulTB=((ulSine1 + ulSine2 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTb= 65536 - pPWMout->ulTB;
            pPWMout->ulTC=((ulSine1 + ulExtra)*62259 + 65536) >> 17;
                        pPWMout->ulTc= 65536 - pPWMout->ulTC;
            break;
        }
    }
}

//!*****************************************************************************
//! 函数名称: void PWMSetFrequency(void)
//! 函数功能: 设置PWM频率
//! 输入参数: uint32_t ulPWMClk 
//!                    ulPWMClk = ???? / PWM??
//! 输出参数: 无
//! 返回值  : 无
//! 详细描述: 无
//!*****************************************************************************
void PWMSetFrequency(uint32_t ulPWMClk)
{
    Chip_SCT_SetMatchReload(SCT1_PWM, SCT_MATCH_0, 
                            Chip_Clock_GetSystemClockRate() / ulPWMClk);
    //4935 电机驱动 
//	Chip_SCT_SetMatchReload(SCT2_PWM, SCT_MATCH_0, 
//									Chip_Clock_GetSystemClockRate() / (ulPWMClk));
}

//!*****************************************************************************
//! 函数名称: void PWMSetWidth(void)
//! 函数功能: 设置PWM 占空比
//! 输入参数: 无
//! 输出参数: 无
//! 返回值  : 无
//! 详细描述: 无
//!*****************************************************************************
static void PWMSetWidth(void)
{
    SCTPWM_SetDutyCycle(SCT1_PWM, PWM_Event1,sPWMout.ulTA);
    SCTPWM_SetDutyCycle(SCT1_PWM, PWM_Event2,sPWMout.ulTB);
    SCTPWM_SetDutyCycle(SCT1_PWM, PWM_Event3,sPWMout.ulTC);
    SCTPWM_SetDutyCycle(SCT1_PWM, 4,sPWMout.ulTa);
    SCTPWM_SetDutyCycle(SCT1_PWM, 5,sPWMout.ulTb);
    SCTPWM_SetDutyCycle(SCT1_PWM, 6,sPWMout.ulTc);

    //4935 电机驱动 下桥臂 脉宽调制
//  SCTPWM_SetDutyCycle(SCT2_PWM, PWM_Event1,sPWMout.ulTA + 60);            //加入死区
//	SCTPWM_SetDutyCycle(SCT2_PWM, PWM_Event2,sPWMout.ulTB + 60);
//	SCTPWM_SetDutyCycle(SCT2_PWM, PWM_Event3,sPWMout.ulTC + 60);
//	SCTPWM_SetDutyCycle(SCT2_PWM, 4,sPWMout.ulTa - 60);
//	SCTPWM_SetDutyCycle(SCT2_PWM, 5,sPWMout.ulTb - 60);
//	SCTPWM_SetDutyCycle(SCT2_PWM, 6,sPWMout.ulTc - 60);
}

//停止电机
void MotorStop(void)
{
    //A4935_nEN_LOW;
    //A4935_nCOAST_HIGH;
    A4935_nCOAST_LOW;
    masterstate = 0x1D;
    Chip_SCTPWM_Stop(SCT1_PWM);
    //Chip_GPIO_SetPinState(LPC_GPIO,0,1,0);
    //Chip_GPIO_SetPinState(LPC_GPIO,1,18,0);
    //Chip_GPIO_SetPinState(LPC_GPIO,1,19,0);
    //Chip_SCTPWM_Stop(SCT2_PWM);
}

//启动电机
void MotorRun(void)
{
    A4935_nEN_HIGH;
    A4935_nCOAST_HIGH;
    masterstate = 0xD1;
    Chip_SCTPWM_Start(SCT1_PWM);
    //Chip_SCTPWM_Start(SCT2_PWM);
}

//串口 接收函数
void UART_RXIntHandlerRB1(LPC_USART_T *pUART, RINGBUFF_T *pRB)
{
    static uint8_t key,i;
    static uint8_t RxCheck;
    uint8_t ch;
    /* New data will be ignored if data not popped in time */
    while ((Chip_UART_GetStatus(pUART) & UART_STAT_RXRDY) != 0) 
    {
        ch = Chip_UART_ReadByte(pUART);
        if(ch == 0xaa)
        {
            key = 0;
            i = 1;
            RxCheck = 0;
        }
        if(i)
        {
            UartData[key] = ch;
            key++;
            if(key >= 8)
            {
                i = 0;
                Rx_read = 1;
            }
        }
    }
}

uint8_t TxCheck()
{
    uint8_t i,TxCheck = 0;
    for(i=1;i<7;i++)               //数据校验
    {
        TxCheck = TxCheck + TxDataBuff[i];
    }
    return TxCheck;
}

/*
      检查A4935 fault flag 输出,正常为0x00
       FF1  FF2
      *********************************************
       0    0    No Faults
       0    1    Short-to-supply,ground or load
       1    0    over temperature
       1    1    VDD, VREG, Bootstrap undervoltage
      **********************************************
    */ 
//读取A4935反馈信号 FF1 FF2
void Err_Value()
{
    uint8_t A4935_FF_Value;
    static unsigned char ErrOtCnt = 0, ErrScCnt = 0, ErrUvCnt = 0,ErrOFFCnt = 0;
    //读取A4935反馈信号 FF1 FF2
    A4935_FF_Value = (Chip_GPIO_GetPinState(LPC_GPIO,0,10) << 1) + Chip_GPIO_GetPinState(LPC_GPIO,1,1);
    if(A4935_FF_Value)
    {
        switch (A4935_FF_Value)
        {
            case 0x01:                          //B0000 0001: short to supply, ground, load
                ErrScCnt++;
                if(ErrScCnt > 8)
                {
                    LED_SC_ON;
                    ErrSc = 1;
                    ErrScCnt = 0;
                }
                break;
            case 0x02:                          //B0000 0010:over temperature
                ErrOtCnt++;
                if(ErrOtCnt > 8)
                {
                    LED_OT_ON;
                    ErrOt = 1;
                    ErrOtCnt = 0;
                }
                break;
            case 0x03:                           //B0000 0011: undervoltage
                ErrUvCnt++;
                if(ErrUvCnt > 8)
                {
                    LED_UV_ON;
                    ErrUv = 1;
                    ErrUvCnt = 0;
                }
                break;
            default:
                ErrOtCnt = 0;
                ErrScCnt = 0;
                ErrUvCnt = 0;
                break;
        }
    }
    else 
    {
        ErrOFFCnt++;
        if(ErrOFFCnt > 100)
        {
            LED_SC_OFF;
            LED_OT_OFF;
            LED_UV_OFF;
            ErrUv = 0;
            ErrSc = 0;
            ErrOt = 0;
            ErrOFFCnt = 0;
        }
    }
}

//自检程序
void checkmotor(void)
{

}

//串口接收数据处理
void UART_Rx(void)
{
    masterstate = UartData[2];
    switch (UartData[1])
    {
        case 0x60:
            check = UartData[8];
            if((masterstate == 0x02) && (check == 0x01 || check == 0x02))
            {
                checkmotor(); 
                TxBuff[0] = 0xA0;
                TxBuff[1] = masterstate;
                TxBuff[2] = ErrStop << 2            //电机不转
                        | ErrUv << 3                //过压
                        | ErrSc << 4                //过流
                        | ErrOt << 5;               //过温					
                TxBuff[3] = motorstate;
                TxBuff[4] = 0;
                TxBuff[5] = 0;
                TxBuff[6] = 0;
                //电机模式
                TxBuff[7] = ErrUv << 1              //过压
                        | ErrSc << 2                //过流
                        | ErrOt << 3;               //过温
                Chip_UART_SendBlocking(DEBUG_UART,&TxBuff,8);
            }
            break;
        case 0x61:	
            motorddirection = UartData[3];
            motorstate = UartData[4];
            Ir = (UartData[5] << 8) + UartData[6];
            motorspeed = UartData[7];
            motormoder = UartData[8];
            switch (motorstate)
            {
                case 0x1D:MotorStop();
                    break;
                case 0xD1:MotorRun();
                    break;
                default:
                    break;
            }
            break;
        case 0xb2:        //开始自检 自检结束后返回 B0
            //masterstate = UartData[2];
            break;
        case 0x50:
            TxBuff[0] = 0xB0;
            TxBuff[1] = masterstate;                    //主机状态
            TxBuff[2] = hardwareversion;                //硬件版本
            TxBuff[3] = softwareversion;                //软件版本
            //上电后运行时间
            TxBuff[4] = UF_time >> 24;
            TxBuff[5] = (UF_time & 0xffffff) >> 16;
            TxBuff[6] = (UF_time & 0xffff) >> 8;
            TxBuff[7] = UF_time & 0xff;
            Chip_UART_SendBlocking(DEBUG_UART,&TxBuff,8);
            break;
        case 0x51:
            TxBuff[0] = 0xB1;
            TxBuff[1] = masterstate;
            TxBuff[2] = motorddirection;
            TxBuff[3] = motorstate;
            //设定电流
            TxBuff[4] = Ir >> 8;
            TxBuff[5] = Ir & 0xff;
            
            TxBuff[6] = motorspeed;
            //电机模式
            TxBuff[7] = motormoder;
            Chip_UART_SendBlocking(DEBUG_UART,&TxBuff,8);
            break;
        default:
            break;
    }
}

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing 
 * 每毫秒执行一次
 */
void SysTick_Handler(void)
{
    static uint32_t count,i,j,n;
    static uint16_t value = 0;

    count++;
    i++;
    
    /* Every 1/2 second */
    j++;
    if (j >= 500) 
    {
        j = 0;

        /* Manual start for ADC1 conversion sequence A */
        //手动启动ADC转换
        Chip_ADC_StartSequencer(LPC_ADC0, ADC_SEQA_IDX);
        Chip_ADC_StartSequencer(LPC_ADC1, ADC_SEQA_IDX);
    }
    value = !value;

    Chip_GPIO_SetPinState(LPC_GPIO,0,17,value);
    //检测错误
    Err_Value();
    //运行灯闪烁
    if(motorstate == 0xD1)
    {
        if(i >= 500)
        {
            i = 0;
            Chip_GPIO_SetPinToggle(LPC_GPIO, 1, 25);        //运行指示灯
            
            if(ErrUv | ErrSc | ErrOt)                       //发送错误 
            {
                n++;
                if(n >= 10)
                {
                    n = 0;
                    MotorStop();
                }
                
                TxDataBuff[0] = 0xA0;
                TxDataBuff[1] = masterstate;
                TxDataBuff[4] = RUN_START;
                TxDataBuff[3] = 0;
                TxDataBuff[7] = ErrUv << 1                  //过压
                            | ErrSc << 2                    //过流
                            | ErrOt << 3;                   //过温
                Chip_UART_SendBlocking(DEBUG_UART,&TxDataBuff,8);
            }
        }
    }
    else
    {
        LED_RUN_OFF;
    }
    //上电后运行计时 一分钟
    if(count >= 1000)
    {
        count = 0;
        UF_time++;
        
        TxDataBuff[0] = 0XA1;
        TxDataBuff[1] = masterstate;            //主机状态
        TxDataBuff[2] = motorddirection;        //实际方向
        TxDataBuff[3] = RUN_START;              //运行状态
        //总电流
        TxDataBuff[4] = Ir >> 8;
        TxDataBuff[5] = Ir & 0xff;

        TxDataBuff[6] = 0;
        TxDataBuff[7] = 0;
        Chip_UART_SendBlocking(DEBUG_UART,&TxDataBuff,8);
        
        TxDataBuff[0] = 0XA2;
        TxDataBuff[1] = masterstate;            //主机状态
        //电源电压
        TxDataBuff[2] = 0xA0;
        TxDataBuff[3] = 0xA0;
        //温度
        TxDataBuff[4] = 0;
        TxDataBuff[5] = 0;
        //实际转速
        TxDataBuff[6] = 0;
        TxDataBuff[7] = 0;
        Chip_UART_SendBlocking(DEBUG_UART,&TxDataBuff,8);
    }
}

void SCT1_IRQHandler(void)
{
    static long PWM_Counter = 0;
    long lchopper;	
    if(motorddirection == 0x3c)     //顺时针旋转
    {
        if (PWM_Counter <= 0)
        {
            PWM_Counter = 5760 ;
        }
        lchopper = PWM_Counter;
 
        PWM_Counter -= motorspeed;  //AddStepTab[MotorFlag][CurSpd];
    }
    if(motorddirection == 0xc3)	    //逆时针旋转
    {
        if (PWM_Counter >= 5760)
        {
            PWM_Counter = 0 ;
        }
        lchopper = PWM_Counter;
 
        PWM_Counter += motorspeed;//AddStepTab[MotorFlag][CurSpd];
    }
    SpaceVectorModulate( Ir, lchopper, &sPWMout);//IrTab[MotorFlag][CurSpd]

    if (sPWMout.ulTA < g_ulMinPulseWidth)
    { 
        sPWMout.ulTA = g_ulMinPulseWidth; 
        sPWMout.ulTa= 65536 - sPWMout.ulTA;
    }
    if ((62259 - sPWMout.ulTA) < g_ulMinPulseWidth)
    { 
        sPWMout.ulTA = 62259 - g_ulMinPulseWidth;
        sPWMout.ulTa= 65536 - sPWMout.ulTA;
    }
    if (sPWMout.ulTB < g_ulMinPulseWidth)
    { 
        sPWMout.ulTB = g_ulMinPulseWidth;
        sPWMout.ulTb= 65536 - sPWMout.ulTB;
    }
    if ((62259 - sPWMout.ulTB) < g_ulMinPulseWidth)
    { 
        sPWMout.ulTB = 62259 - g_ulMinPulseWidth;
        sPWMout.ulTb= 65536 - sPWMout.ulTB;
    }
    if (sPWMout.ulTC < g_ulMinPulseWidth)
    { 
        sPWMout.ulTC = g_ulMinPulseWidth;
        sPWMout.ulTc= 65536 - sPWMout.ulTC;
    }
    if ((62259 - sPWMout.ulTC) < g_ulMinPulseWidth)
    { 
        sPWMout.ulTC = 62259 - g_ulMinPulseWidth;
        sPWMout.ulTc= 65536 - sPWMout.ulTC;
    }
    
    PWMSetWidth();

    Chip_SCT_ClearEventFlag(SCT1_PWM,SCT_EVT_0);
}

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void UART0_IRQHandler(void)
{
    /* Want to handle any errors? Do it here. */

    /* Use default ring buffer handler. Override this with your own
        code if you need more capability. */
    UART_RXIntHandlerRB1(DEBUG_UART,&txring);
    Chip_UART_TXIntHandlerRB(DEBUG_UART,&txring);

    //处理接收函数
    if(Rx_read)
    {
        Rx_read = 0;
        UART_Rx();
        //Chip_UART_SendBlocking(DEBUG_UART,&OK,3);
    }
}

///////////////////////////////////////////////////////////////////////////////
//串口 接收 命令 命令格式
//		起始位|数字位	|				|			电流值		|速度值	|		无	|
//		BYTE0	|	Bit1	|	Bit2	|	Bit3	|	Bit4	|	Bit5	|	Bit6	|	Bit7	|
//		0xAA	|	0x07	|	0xB1	|								|				|				|	校验	|	
//		0xAA	|	0x07	|	0xB2	|								|				|				|	校验	|
//														反馈	|运行状态|无		|	无
//		0xAA	|	0x07	|	0xB3	|				|				|				|				|	校验	|
//
///////////////////////////////////////////////////////////////////////////////
/******************************************************************************
*串口发送 电机状态 数据格式
*			
*
******************************************************************************/
//void SCT3_IRQHandler(void)
//{
//	Chip_GPIO_SetPortToggle(LPC_GPIO,0,28);
//	Chip_SCT_ClearEventFlag(SCT3_PWM,0);
//}


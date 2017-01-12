
/*******************************************************************************
*
*       Copyright(c) 2008-2017; Beijing HeartCare Medical Co. LTD.
*
*       All rights reserved.  Protected by international copyright laws.
*       Knowledge of the source code may NOT be used to develop a similar product.
*
* File:          wang_1549_spi.h
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

#ifndef _WANG_1549_SPI_H__
#define  _WANG_1549_SPI_H__

#include "board.h"

void Init_SPI_PinMux(void);
void errorSPI(void);
void setupSpiMaster();
void WriteSpiMssg(uint16_t *xferPtr, uint32_t xferSize);

#endif

// end of the file

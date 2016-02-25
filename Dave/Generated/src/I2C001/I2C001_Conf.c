/*******************************************************************************
**  DAVE App Name : I2C001       App Version: 1.0.28               
**  This file is generated by DAVE, User modification to this file will be    **
**  overwritten at the next code generation.                                  **
*******************************************************************************/



/*CODE_BLOCK_BEGIN[I2C001_Conf.c]*/
/******************************************************************************
 Copyright (c) 2011, Infineon Technologies AG                                **
 All rights reserved.                                                        **
                                                                             **
 Redistribution and use in source and binary forms, with or without          **
 modification,are permitted provided that the following conditions are met:  **
                                                                             **
 *Redistributions of source code must retain the above copyright notice,     **
 this list of conditions and the following disclaimer.                       **
 *Redistributions in binary form must reproduce the above copyright notice,  **
 this list of conditions and the following disclaimer in the documentation   **
 and/or other materials provided with the distribution.                      **
 *Neither the name of the copyright holders nor the names of its contributors**
 may be used to endorse or promote products derived from this software       ** 
 without specific prior written permission.                                  **
                                                                             **
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" **
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   **
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  **
 ARE  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE  **
 LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        **
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF        **
 SUBSTITUTE GOODS OR  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS   **
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN     **
 CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)      **
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE  **
 POSSIBILITY OF SUCH DAMAGE.                                                 **
                                                                             **
 To improve the quality of the software, users are encouraged to share       **
 modifications, enhancements or bug fixes with Infineon Technologies AG      **
 dave@infineon.com).                                                         **
                                                                             **
*******************************************************************************
**                                                                           **
**                                                                           **
** PLATFORM : Infineon XMC4000/XMC1000 Series                                **
**                                                                           **
** COMPILER : Compiler Independent                                           **
**                                                                           **
** AUTHOR   : App Developer                                                  **
**                                                                           **
** MAY BE CHANGED BY USER [yes/no]: Yes                                      **
**                                                                           **
** MODIFICATION DATE : Jan 28, 2013                                          **
******************************************************************************/
/**
 * @file   I2C001_Conf.c
 *
 * @App Version I2C001 <1.0.28>
 *
 * @brief  Configuration file generated based on UI settings 
 *         of I2C001 App
 *
 */
/*
 * Revision History
 * 28 Jan 2013 v1.0.22  Changes from 1.0.12 are
 *                      1. Modified as per coding guidelines and MISRA checks.
 *                      2. Updated for alignments in revision history.
 *                      3. Updated to support XMC1000 devices.
 *
 */

/******************************************************************************
 ** INCLUDE FILES                                                            **
 *****************************************************************************/
#include <DAVE3.h>




const I2C001Handle_type I2C001_Handle0  = 
{
  /* Temp Code for testing Eval functions */

   .USICChannel = USIC_U2C0,/* USIC Channel No */
   .I2CRegs = USIC2_CH0, /* Register Base address */
   .BitRate  = (uint32_t) 100,/* Bit Rate */
   .TxLimit = (uint8_t) 1,/* FIFO Tigger Level */
   .RxLimit = (uint8_t) 0,/* FIFO Tigger Level */
   .TxFifoSize = (uint8_t) 2,/* Tx FIFO Size */
   .RxFifoSize = (uint8_t) 2,/* Rx FIFO Size */
   .StartCondRecvIntEn = (bool) 0, /* Protocol specific interrupt enable */
   .RepStartCondRecvIntEn = (bool) 0, /* Protocol specific interrupt enable */
   .StopCondRecvIntEn = (bool) 0, /* Protocol specific interrupt enable */
   .NackDetectIntEn = (bool) 0, /* Protocol specific interrupt enable */
   .ArbLostIntEn = (bool) 0, /* Protocol specific interrupt enable */   
   .ErrorIntEn = (bool) 0, /* Protocol specific interrupt enable */   
   .AckIntEn = (bool) 0 /* Protocol specific interrupt enable */   
     
};


const I2C001Handle_type I2C001_Handle1  = 
{
  /* Temp Code for testing Eval functions */

   .USICChannel = USIC_U0C0,/* USIC Channel No */
   .I2CRegs = USIC0_CH0, /* Register Base address */
   .BitRate  = (uint32_t) 100,/* Bit Rate */
   .TxLimit = (uint8_t) 1,/* FIFO Tigger Level */
   .RxLimit = (uint8_t) 0,/* FIFO Tigger Level */
   .TxFifoSize = (uint8_t) 1,/* Tx FIFO Size */
   .RxFifoSize = (uint8_t) 1,/* Rx FIFO Size */
   .StartCondRecvIntEn = (bool) 0, /* Protocol specific interrupt enable */
   .RepStartCondRecvIntEn = (bool) 0, /* Protocol specific interrupt enable */
   .StopCondRecvIntEn = (bool) 0, /* Protocol specific interrupt enable */
   .NackDetectIntEn = (bool) 0, /* Protocol specific interrupt enable */
   .ArbLostIntEn = (bool) 0, /* Protocol specific interrupt enable */   
   .ErrorIntEn = (bool) 0, /* Protocol specific interrupt enable */   
   .AckIntEn = (bool) 0 /* Protocol specific interrupt enable */   
     
};

/*CODE_BLOCK_END*/ 


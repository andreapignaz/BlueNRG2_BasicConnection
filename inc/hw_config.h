/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
 * File Name          : hw_config.h
 * Author             : IoT-EC Application team
 * Version            : V1.3.0
 * Date               : 20-July-2020
 * Description        : STEVAL-BCN002V1 Hardware configuration header file
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HW_CONFIG_H_
#define _HW_CONFIG_H_

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ble_const.h"

/* Exported types ------------------------------------------------------- */

void Platform_Init(void);
void GPIO_Configuration(void);
void Interrupts_EXT_IO_Config(void);
void DMA_Configuration(void);
void ADC_DMA_Configuration(uint8_t xMode);


#endif /* _HW_CONFIG_H_ */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/

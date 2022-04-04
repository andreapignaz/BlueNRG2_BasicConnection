/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
 * File Name          : HWAdvanceFeature.h
 * Author             : IoT-EC Application team
 * Version            : 1.3.0
 * Date               : 20-July-2020
 * Description        : STEVAL-BCN002V1 Hardware advance feature header file
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HW_ADVANCE_FEATURES_H_
#define _HW_ADVANCE_FEATURES_H_

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>

/* Exported types ------------------------------------------------------- */
typedef enum
{
  ACC_NOT_USED     = 0x00,
  ACC_6D_OR_TOP    = 0x01,
  ACC_6D_OR_LEFT   = 0x02,
  ACC_6D_OR_BOTTOM = 0x03,
  ACC_6D_OR_RIGHT  = 0x04,
  ACC_6D_OR_UP     = 0x05,
  ACC_6D_OR_DOWN   = 0x06,
  ACC_TILT         = 0x08,
  ACC_FREE_FALL    = 0x10,
  ACC_SINGLE_TAP   = 0x20,
  ACC_DOUBLE_TAP   = 0x40,
  ACC_WAKE_UP      = 0x80
} AccEventType;

/* Exported functions ------------------------------------------------------- */
void InitHWFeatures(void);
void DisableHWFeatures(void);

void EnableHWPedometer(void);
void DisableHWPedometer(void);
void ResetHWPedometer(void);
uint16_t GetStepHWPedometer(void);

void EnableHWFreeFall(void);
void DisableHWFreeFall(void);

void EnableHWDoubleTap(void);
void DisableHWDoubleTap(void);

void EnableHWSingleTap(void);
void DisableHWSingleTap(void);

void EnableHWWakeUp(void);
void DisableHWWakeUp(void);

void EnableHWTilt(void);
void DisableHWTilt(void);

void EnableHWOrientation6D(void);
void DisableHWOrientation6D(void);
AccEventType GetHWOrientation6D(void);

#endif /* _HW_ADVANCE_FEATURES_H_ */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/

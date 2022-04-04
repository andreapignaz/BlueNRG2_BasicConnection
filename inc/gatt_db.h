/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
 * File Name          : gatt_db.h
 * Author             : IoT-EC
 * Version            : V1.3.0
 * Date               : 20-July-2020
 * Description        : STEVAL-BCN002V1 GATT DB and handle GATT events headers
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

#ifndef _GATT_DB_H_
#define _GATT_DB_H_

#include "sensor.h"

/** 
 * @brief Structure containing acceleration value of each axis.
 */
typedef struct {
	int32_t AXIS_X;
	int32_t AXIS_Y;
	int32_t AXIS_Z;
} AxesRaw_t;

enum {
	HW_SERVICE_INDEX = 0, CONFIG_SERVICE_INDEX = 1, CONSOLE_SERVICE_INDEX = 2
};

extern uint8_t Services_Max_Attribute_Records[];

tBleStatus Add_HW_SW_ServW2ST_Service(void);
tBleStatus Add_ConsoleW2ST_Service(void);
tBleStatus Add_ConfigW2ST_Service(void);

tBleStatus AccGyroMag_Update(AxesRaw_t *Acc, AxesRaw_t *Gyro, AxesRaw_t *Mag);
tBleStatus Environmental_Update(int32_t Press, uint16_t Hum, int16_t Temp);
tBleStatus Terminal_Update(uint8_t *data, uint8_t length);
tBleStatus GasGauge_Update(uint32_t voltage_mV);
tBleStatus ECompass_Update(uint16_t xAngle);
tBleStatus Quaternion_Update(QuaternionAxes_t *data);
tBleStatus FlightSense_Update(uint16_t Distance);
tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte);
tBleStatus FreeFall_Notify(void);
tBleStatus Led_Update(FunctionalState xState);
tBleStatus Config_Notify(uint32_t Feature, uint8_t Command, uint8_t data);
tBleStatus Term_Update(uint8_t *data, uint16_t length);
uint32_t getTimestamp(void);

void Read_Request_CB(uint16_t handle);
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length);

#define W2ST_CONSOLE_MAX_CHAR_LEN 20

#endif /* _GATT_DB_H_ */

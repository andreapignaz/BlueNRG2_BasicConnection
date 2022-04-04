/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
 * File Name          : gatt_db.c
 * Author             : IoT-EC
 * Version            : V1.3.0
 * Date               : 20-July-2020
 * Description        : STEVAL-BCN002V1 GATT DB and handle GATT events
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "clock.h"
#include "gp_timer.h"
#include "gatt_db.h"
#include "osal.h"
#include "SDK_EVAL_Config.h"
#include "HWAdvanceFeatures.h"
#include "sensor.h"
#include "sleep.h"
#include "hw_config.h"

#if ENABLE_MOTIONFX
	#include "motion_fx_cm0p.h"			// MotionFX for CortexM0
#endif

#if ENABLE_BLUEVOICE
	#include "bluevoice_adpcm_bnrg1.h"	// BlueVoice for CortexM0
#endif

#if ENABLE_DEBUG
	#include <stdio.h>
	#define PRINTF(...) printf(__VA_ARGS__)
#else
	#define PRINTF(...)
#endif

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

#define STORE_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

#define STORE_BE_32(buf, val)    ( ((buf)[3] =  (uint8_t) (val)     ) , \
                                   ((buf)[2] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[1] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[0] =  (uint8_t) (val>>24) ) )

/* Hardware Characteristics Service */
#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)		COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid_struct)	COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid_struct)	COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid_struct)		COPY_UUID_128(uuid_struct,0x00,0x00,0x04,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_GG_W2ST_CHAR_UUID(uuid_struct)				COPY_UUID_128(uuid_struct,0x00,0x02,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_PROX_W2ST_CHAR_UUID(uuid_struct)			COPY_UUID_128(uuid_struct,0x02,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_LED_W2ST_CHAR_UUID(uuid_struct)			COPY_UUID_128(uuid_struct,0x20,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#if ENABLE_MOTIONFX
	#define COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid_struct)	COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define COPY_ECOMPASS_W2ST_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x40,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif

#if ENABLE_BLUEVOICE
#define COPY_AUDIO_ADPCM_CHAR_UUID(uuid_struct)			COPY_UUID_128(uuid_struct,0x08,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_AUDIO_ADPCM_SYNC_CHAR_UUID(uuid_struct)	COPY_UUID_128(uuid_struct,0x40,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif

/* Configuration Service */
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)			COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_W2ST_CHAR_UUID(uuid_struct)			COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Console Service */
#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)			COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TERM_CHAR_UUID(uuid_struct)				COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_STDERR_CHAR_UUID(uuid_struct)				COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Private variables ------------------------------------------------------------*/
static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t AccGyroMagCharHandle;
static uint16_t AccEventCharHandle;
static uint16_t ProxCharHandle;
static uint16_t LedCharHandle;
static uint16_t GGCharHandle;

#if ENABLE_MOTIONFX
	static uint16_t QuaternionsCharHandle;
	static uint16_t ECompassCharHandle;
#endif

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

/* Size for Environmental BLE characteristic */
static uint8_t EnvironmentalCharSize = 2;

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

extern int32_t pressure;
extern uint16_t humidity;
extern int16_t temperature;

volatile bool FirstConnectionConfig = false;

extern volatile FeaturePresence xFeaturePresence;
extern volatile FeatureNotification xFeatureNotification;
extern volatile HardwareFeaturePresence xHardwareFeaturePresence;

extern volatile uint32_t start_time;
extern volatile int32_t updateTimer;
extern volatile uint32_t ConnectionBleStatus;
extern volatile bool mag_force_calibration;
extern volatile uint16_t sensors_update_rate;
extern volatile bool sensorTimer_expired;
extern bool mag_is_calibrated;
extern uint16_t connection_handle;
extern uint8_t ProxyStatus;

#if ENABLE_BLUEVOICE
	extern BV_ADPCM_BNRG1_Config_t BV_ADPCM_BNRG1_Config;
	BV_ADPCM_BNRG1_ProfileHandle_t TX_handle_BV;
	BV_ADPCM_BNRG1_uuid_t uuid_BV;
	BV_ADPCM_BNRG1_Config_t BV_ADPCM_BNRG1_Config;
#endif

#if ENABLE_MOTIONFX
	QuaternionAxes_t axes_data[1];
#endif

uint8_t BufferToWrite[512];
int32_t BytesToWrite;

uint8_t buff[20];

/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;
Char_Desc_Uuid_t char_desc_uuid;

tBleStatus Add_HW_SW_ServW2ST_Service(void) {
	tBleStatus ret;

	uint8_t uuid[16];

	COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
	Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
	ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, Services_Max_Attribute_Records[HW_SERVICE_INDEX], &HWServW2STHandle);

	COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
	/* Fill the Environmental BLE Characteristic */
	uuid[14] |= 0x04; /* One Temperature value1*/
	EnvironmentalCharSize += 2 * 2;
	uuid[14] |= 0x08; /* Humidity */
	EnvironmentalCharSize += 2;
	uuid[14] |= 0x10; /* Pressure value*/
	EnvironmentalCharSize += 4;

	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, EnvironmentalCharSize, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &EnvironmentalCharHandle);

	COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2 + 3 * 3 * 2, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &AccGyroMagCharHandle);

	COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2 + 3, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &AccEventCharHandle);

	COPY_PROX_W2ST_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2 + 2, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &ProxCharHandle);

	COPY_LED_W2ST_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2 + 1, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &LedCharHandle);

	COPY_GG_W2ST_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2 + 2 + 2 + 2 + 1, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &GGCharHandle);

#if ENABLE_MOTIONFX
	COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2 + 6, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &QuaternionsCharHandle);

	COPY_ECOMPASS_W2ST_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 2 + 2, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &ECompassCharHandle);
#endif

#if ENABLE_BLUEVOICE
	COPY_AUDIO_ADPCM_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_DONT_NOTIFY_EVENTS, 16, 0, &TX_handle_BV.CharAudioHandle);

	COPY_AUDIO_ADPCM_SYNC_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid, 6, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_DONT_NOTIFY_EVENTS, 16, 0, &TX_handle_BV.CharAudioSyncHandle);

	memcpy(&TX_handle_BV.ServiceHandle, &HWServW2STHandle, sizeof(uint16_t));
	BluevoiceADPCM_BNRG1_SetTxHandle(&TX_handle_BV);

	BV_ADPCM_BNRG1_Config.sampling_frequency = FR_8000;
	BV_ADPCM_BNRG1_Config.channel_in = 1;
	BV_ADPCM_BNRG1_Config.channel_tot = 1;
	BluevoiceADPCM_BNRG1_SetConfig(&BV_ADPCM_BNRG1_Config);
#endif

	return ret;

}

tBleStatus Add_ConfigW2ST_Service(void) {
	tBleStatus ret;

	uint8_t uuid[16];

	COPY_CONFIG_SERVICE_UUID(uuid);
	Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
	ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, Services_Max_Attribute_Records[CONFIG_SERVICE_INDEX], &ConfigServW2STHandle);

	if (ret != BLE_STATUS_SUCCESS)
		goto fail;

	COPY_CONFIG_W2ST_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, &char_uuid, 20, CHAR_PROP_NOTIFY | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 1, &ConfigCharHandle);

	if (ret != BLE_STATUS_SUCCESS) {
		goto fail;
	}

	return ret;

	fail: return BLE_STATUS_ERROR;
}

tBleStatus Add_ConsoleW2ST_Service(void) {
	tBleStatus ret;

	uint8_t uuid[16];

	COPY_CONSOLE_SERVICE_UUID(uuid);
	Osal_MemCpy(&service_uuid.Service_UUID_128, uuid, 16);
	ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, Services_Max_Attribute_Records[CONSOLE_SERVICE_INDEX], &ConsoleW2STHandle);

	if (ret != BLE_STATUS_SUCCESS) {
		goto fail;
	}

	COPY_TERM_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, &char_uuid, W2ST_CONSOLE_MAX_CHAR_LEN, CHAR_PROP_NOTIFY | CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 1, &TermCharHandle);

	if (ret != BLE_STATUS_SUCCESS) {
		goto fail;
	}

	COPY_STDERR_CHAR_UUID(uuid);
	Osal_MemCpy(&char_uuid.Char_UUID_128, uuid, 16);
	ret = aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, &char_uuid, W2ST_CONSOLE_MAX_CHAR_LEN, CHAR_PROP_NOTIFY | CHAR_PROP_READ, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 1, &StdErrCharHandle);

	if (ret != BLE_STATUS_SUCCESS) {
		goto fail;
	}

	return ret;

	fail: return BLE_STATUS_ERROR;
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Notify(uint16_t Command, uint8_t dimByte) {
	tBleStatus ret = 0;
	uint8_t buff_2[2 + 2];
	uint8_t buff_3[2 + 3];

	switch (dimByte) {
	case 2:
		STORE_LE_16(buff_2, (getTimestamp()));
		STORE_LE_16(buff_2 + 2, Command);
		ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2 + 2, buff_2);
		break;
	case 3:
		STORE_LE_16(buff_3, (getTimestamp()));
		buff_3[2] = 0;
		STORE_LE_16(buff_3 + 3, Command);
		ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2 + 3, buff_3);
		break;
	}
	return ret;
}

#if ENABLE_MOTIONFX
/**
 * @brief  Update quaternion characteristic value
 * @param  QuaternionAxes_t *data Structure containing the quaternion
 * @retval tBleStatus      Status
 */
tBleStatus Quaternion_Update(QuaternionAxes_t *data) {
	tBleStatus ret = 0;

	STORE_LE_16(buff, (getTimestamp()));
	STORE_LE_16(buff + 2, data[0].AXIS_X);
	STORE_LE_16(buff + 4, data[0].AXIS_Y);
	STORE_LE_16(buff + 6, data[0].AXIS_Z);
	ret = aci_gatt_update_char_value(HWServW2STHandle, QuaternionsCharHandle, 0, 2 + 6, buff);

	if (ret != BLE_STATUS_SUCCESS) {
		return BLE_STATUS_ERROR;
	}
	return ret;
}

#endif

/**
 * @brief  Update Gas Gauge characteristic value
 * @param  None
 * @retval tBleStatus   Status
 */
tBleStatus GasGauge_Update(uint32_t voltage_mV) {
	uint32_t SoC = 0;

	SoC = (voltage_mV - 2000);
	if (SoC > 1000)
		SoC = 1000;

	STORE_LE_16(buff, (getTimestamp()));
	STORE_LE_16(buff + 2, SoC);
	STORE_LE_16(buff + 4, voltage_mV);
	STORE_LE_16(buff + 6, 0x8000); // Not available

	if (SoC < 450) {
		/* if it's < 45% Low Battery*/
		buff[8] = 0x00; /* Low Battery */
	} else {
		static uint32_t Status = 0x01; /* Discharging */
		buff[8] = Status;
	}

	return aci_gatt_update_char_value(HWServW2STHandle, GGCharHandle, 0, 2 + 2 + 2 + 2 + 1, buff);

}

#if ENABLE_MOTIONFX
/**
 * @brief  Update E-Compass characteristic value
 * @param  uint16_t Angle To Magnetic North in cents of degree [0.00 -> 359,99]
 * @retval tBleStatus      Status
 */
tBleStatus ECompass_Update(uint16_t xAngle) {

	STORE_LE_16(buff, (getTimestamp()));
	STORE_LE_16(buff + 2, xAngle);

	return  aci_gatt_update_char_value(HWServW2STHandle, ECompassCharHandle, 0, 2 + 2, buff);

}
#endif


/**
 * @brief  Update Distance Value
 * @param  uint16_t Measured Distance value
 * @retval tBleStatus   Status
 */
tBleStatus FlightSense_Update(uint16_t Distance) {

	STORE_LE_16(buff, (getTimestamp()));
	STORE_LE_16(buff + 2, Distance);

	return aci_gatt_update_char_value(HWServW2STHandle, ProxCharHandle, 0, 2 + 2, buff);

}


/**
 * @brief  Update Led Status
 * @param  uint16_t ENABLE or DISABLE
 * @retval tBleStatus   Status
 */
tBleStatus Led_Update(FunctionalState xState) {
	STORE_LE_16(buff, (getTimestamp()));
	buff[2] = xState;

	return  aci_gatt_update_char_value(HWServW2STHandle, LedCharHandle, 0, 2 + 1, buff);

}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void) {

	 return aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, LastStderrLen, LastStderrBuffer);

}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void) {

	return aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen, LastTermBuffer);

}

/**
 * @brief  Update acceleration/Gryoscope and Magneto characteristics value
 * @param  SensorAxes_t Acc Structure containing acceleration value in mg
 * @param  SensorAxes_t Gyro Structure containing Gyroscope value
 * @param  SensorAxes_t Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyroMag_Update(AxesRaw_t *Acc, AxesRaw_t *Gyro, AxesRaw_t *Mag) {

	STORE_LE_16(buff, (getTimestamp()));

	STORE_LE_16(buff + 2, Acc->AXIS_X);
	STORE_LE_16(buff + 4, Acc->AXIS_Y);
	STORE_LE_16(buff + 6, Acc->AXIS_Z);

	Gyro->AXIS_X /= 10;
	Gyro->AXIS_Y /= 10;
	Gyro->AXIS_Z /= 10;

	STORE_LE_16(buff + 8, Gyro->AXIS_X);
	STORE_LE_16(buff + 10, Gyro->AXIS_Y);
	STORE_LE_16(buff + 12, Gyro->AXIS_Z);

	STORE_LE_16(buff + 14, Mag->AXIS_X);
	STORE_LE_16(buff + 16, Mag->AXIS_Y);
	STORE_LE_16(buff + 18, Mag->AXIS_Z);

	return  aci_gatt_update_char_value(HWServW2STHandle, AccGyroMagCharHandle, 0, 2 + 3 * 3 * 2, buff);

}

/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp1 Temperature in tenths of degree
 * @retval tBleStatus   Status
 */
tBleStatus Environmental_Update(int32_t Press, uint16_t Hum, int16_t Temp) {
	uint8_t BuffPos = 0;

	STORE_LE_16(buff, (getTimestamp()));
	BuffPos = 2;

	STORE_LE_32(buff + BuffPos, Press);
	BuffPos += 4;

	STORE_LE_16(buff + BuffPos, Hum);
	BuffPos += 2;

	STORE_LE_16(buff + BuffPos, Temp);

	return aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize, buff);

}

/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature, uint8_t Command, uint8_t data) {
	STORE_LE_16(buff, (getTimestamp()));
	STORE_BE_32(buff + 2, Feature);
	buff[6] = Command;
	buff[7] = data;

	return aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, 8, buff);

}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint16_t lenght of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data, uint16_t length) {
	uint16_t Offset;
	uint16_t DataToSend;

	/* Split the code in packages */
	for (Offset = 0; Offset < length; Offset += W2ST_CONSOLE_MAX_CHAR_LEN) {
		DataToSend = (length - Offset);
		DataToSend = (DataToSend > W2ST_CONSOLE_MAX_CHAR_LEN) ? W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;
		/* keep a copy */
		memcpy(LastTermBuffer, data + Offset, DataToSend);
		LastTermLen = DataToSend;
		aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend, data + Offset);
	}

	return 0;
}

uint32_t getTimestamp(void){
	return HAL_VTimerDiff_ms_sysT32(HAL_VTimerGetCurrentTime_sysT32(), start_time);
}

/*******************************************************************************
 * Function Name  : Read_Request_CB.
 * Description    : Update the sensor valuse.
 * Input          : Handle of the characteristic to update.
 * Return         : None.
 *******************************************************************************/
void Read_Request_CB(uint16_t handle) {

	if (handle == AccGyroMagCharHandle + 1) {
		// TBD by customer
	} else if (handle == EnvironmentalCharHandle + 1) {
		// TBD by customer
	} else if (handle == AccEventCharHandle + 1) {
		uint16_t StepCount;
		if (xHardwareFeaturePresence.HwPedometer) {
			StepCount = GetStepHWPedometer();
		} else {
			StepCount = 0;
		}
		AccEvent_Notify(StepCount, 2);
	} else if (handle == LedCharHandle + 1) {
		Led_Update((FunctionalState)(SdkEvalLedGetState(LED1)));
	} else if (handle == StdErrCharHandle + 1) {
		// Send again the last packet for StdError
		Stderr_Update_AfterRead();
	} else if (handle == TermCharHandle + 1) {
		// Send again the last packet for Terminal
		Term_Update_AfterRead();
	}

	if (connection_handle != 0) {
		aci_gatt_allow_read(connection_handle);
	}
}

void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length) {

	if (attr_handle == (StdErrCharHandle + 2)) {
		if (att_data[0] == 01)
			W2ST_ON_CONNECTION(W2ST_CONNECT_STD_ERR);
		else if (att_data[0] == 0)
			W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_ERR);
	} else if (attr_handle == (TermCharHandle + 2)) {
		if (att_data[0] == 01)
			W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
		else if (att_data[0] == 0)
			W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);

	} else if (attr_handle == TermCharHandle + 1) {
		uint32_t SendBackData = 1; /* By default Answer with the same message received */
				/* Received one write from Client on Terminal characteristic */
				SendBackData = DebugConsoleCommandParsing(att_data, data_length);
		/* Send it back for testing */
		if (SendBackData) {
			Term_Update(att_data, data_length);
		}

#if ENABLE_MOTIONFX
	} else if ((attr_handle == (AccGyroMagCharHandle + 2)) || (attr_handle == (QuaternionsCharHandle + 2))) {
#else
	} else if (attr_handle == (AccGyroMagCharHandle + 2)) {
#endif
		if (att_data[0] == 01) {
			lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_52Hz);
			lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_52Hz);
			lis2mdl_operating_mode_set(0, LIS2MDL_CONTINUOUS_MODE);
#if ENABLE_MOTIONFX
					Init_SensorFusion(true);
					xFeatureNotification.iNemoEngineNotification = true;
#else
			xFeatureNotification.iNemoEngineNotification = false;
#endif
			xFeatureNotification.MotionNotification = true;
			xFeatureNotification.ProximityNotification = false;
			PRINTF("MotionSensor ON\n\r");
		} else if (att_data[0] == 0) {
#if ENABLE_MOTIONFX
			Init_SensorFusion(false);
			xFeatureNotification.iNemoEngineNotification = false;
#endif
			xFeatureNotification.MotionNotification = false;
			xFeatureNotification.ProximityNotification = false;
			PRINTF("MotionSensor OFF\n\r");
			lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_OFF);
			lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_OFF);
			lis2mdl_operating_mode_set(0, LIS2MDL_POWER_DOWN);
		}

	} else if (attr_handle == (EnvironmentalCharHandle + 2)) {

		if (att_data[0] == 01) {
			if (xFeaturePresence.PressurePresence)
				lps22hh_data_rate_set(0, LPS22HH_50_Hz_LOW_NOISE);
			if (xFeaturePresence.HumidityTemperaturePresence) {
				HTS221_Set_PowerDownMode(0, HTS221_RESET);
				HTS221_Activate(0);
			}
			if (xFeatureNotification.EnvironmentalNotification != true) {
				xFeatureNotification.EnvironmentalNotification = true;
				PRINTF("Env ON\n\r");
			}
		} else if (att_data[0] == 0) {
			if (xFeaturePresence.PressurePresence)
				lps22hh_data_rate_set(0, LPS22HH_POWER_DOWN);
			if (xFeaturePresence.HumidityTemperaturePresence)
				HTS221_Set_PowerDownMode(0, HTS221_SET);
			if (xFeatureNotification.EnvironmentalNotification != false) {
				xFeatureNotification.EnvironmentalNotification = false;
				PRINTF("Env OFF\n\r");
			}
		}

	} else if (attr_handle == (GGCharHandle + 2)) {

		if (att_data[0] == 01) {
			ADC_DMA_Configuration(ADC_Input_BattSensor);
			xFeatureNotification.BatteryMonitorNotification = true;
			PRINTF("Gauge ON\n\r");

		} else if (att_data[0] == 0) {
			xFeatureNotification.BatteryMonitorNotification = false;
			ADC_DMA_Configuration(ADC_Input_None);
			PRINTF("Gauge OFF\n\r");
		}

	} else if (attr_handle == (LedCharHandle + 2)) {

		if (att_data[0] == 01) {
			xFeatureNotification.LedNotification = true;
			PRINTF("Led ON\n\r");
		} else if (att_data[0] == 0) {
			xFeatureNotification.LedNotification = false;
			PRINTF("Led OFF\n\r");
		}
	}

#if ENABLE_MOTIONFX
	else if (attr_handle == (ECompassCharHandle + 2)) {
		if (att_data[0] == 01) {
			lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_52Hz);
			lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_52Hz);
			lis2mdl_operating_mode_set(0, LIS2MDL_CONTINUOUS_MODE);
			Init_SensorFusion(true);
			xFeatureNotification.eCompassNotification = true;
			MotionFX_CM0P_enable_9X(MFX_CM0P_ENGINE_ENABLE);
			PRINTF("Compass ON\n\r");
		} else if (att_data[0] == 0) {
			xFeatureNotification.eCompassNotification = false;
			Init_SensorFusion(false);
			lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_OFF);
			lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_OFF);
			lis2mdl_operating_mode_set(0, LIS2MDL_POWER_DOWN);
			MotionFX_CM0P_enable_9X(MFX_CM0P_ENGINE_DISABLE);
			PRINTF("Compass OFF\n\r");
		}
	}
#endif

	else if (attr_handle == (AccEventCharHandle + 2)) {

		if (xFeatureNotification.MotionNotification == false && xFeatureNotification.iNemoEngineNotification == false) {
			if (att_data[0] == 01) {
				lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_52Hz);
				ResetHWPedometer();
				EnableHWWakeUp();
				GPIO_EXTICmd(GPIO_Pin_13, ENABLE);
				Config_Notify(FEATURE_MASK_ACC_EVENTS, 'w', 1);
				AccEvent_Notify(0, 3);
				xFeatureNotification.AccEventNotification = true;
				PRINTF("AccEvent ON\n\r");
			} else if (att_data[0] == 0) {
				lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_OFF);
				xFeatureNotification.AccEventNotification = false;
				GPIO_EXTICmd(GPIO_Pin_13, DISABLE);
				DisableHWFeatures();
				PRINTF("AccEvent OFF\n\r");
			}
		}

	} else if (attr_handle == (ProxCharHandle + 2)) {

		if (att_data[0] == 01) {
			xFeatureNotification.ProximityNotification = true;
			ProxyStatus = PROXY_OFF;
			PRINTF("Proximity ON\n\r");
		} else if (att_data[0] == 0) {
			xFeatureNotification.ProximityNotification = false;
			PRINTF("Proximity OFF\n\r");
		}

	}

#if ENABLE_MOTIONFX
	else if (attr_handle == (QuaternionsCharHandle + 2)) {

		if (att_data[0] == 01) {
			lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_52Hz);
			lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_52Hz);
			lis2mdl_operating_mode_set(0, LIS2MDL_CONTINUOUS_MODE);
			Init_SensorFusion(true);
			PRINTF("Quaternion ON\n\r");
			xFeatureNotification.iNemoEngineNotification = true;
			xFeatureNotification.ProximityNotification = false;

		} else if (att_data[0] == 0) {
			xFeatureNotification.iNemoEngineNotification = false;
			xFeatureNotification.ProximityNotification = false;
			Init_SensorFusion(false);
			lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_OFF);
			lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_OFF);
			lis2mdl_operating_mode_set(0, LIS2MDL_POWER_DOWN);
			PRINTF("Quaternion OFF\n\r");

		}

	}
#endif

#if ENABLE_BLUEVOICE
	else if (attr_handle == (TX_handle_BV.CharAudioHandle + 2)) {

		BluevoiceADPCM_BNRG1_AttributeModified_CB(attr_handle, data_length, att_data);

		if (att_data[0] == 01) {
			PRINTF("Audio ON\n\r");
			xFeatureNotification.AudioNotification = true;
			ADC_DMA_Configuration(ADC_Input_Microphone);
		} else if (att_data[0] == 0) {
			PRINTF("Audio OFF\n\r");
			xFeatureNotification.AudioNotification = false;
			ADC_DMA_Configuration(ADC_Input_BattSensor);
			ADC_DMA_Configuration(ADC_Input_None);
		}

	} else if (attr_handle == (TX_handle_BV.CharAudioSyncHandle + 2)) {

		BluevoiceADPCM_BNRG1_AttributeModified_CB(attr_handle, data_length, att_data);

		if (att_data[0] == 01) {
			xFeatureNotification.SyncNotification = true;
		} else if (att_data[0] == 0) {
			xFeatureNotification.SyncNotification = false;
		}

	}
#endif
	else if (attr_handle == (ConfigCharHandle + 2)) {

		if (att_data[0] == 01) {
#if ENABLE_MOTIONFX
			Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT, W2ST_COMMAND_CAL_STATUS, mag_is_calibrated ? 100 : 0);
			Config_Notify(FEATURE_MASK_ECOMPASS, W2ST_COMMAND_CAL_STATUS, mag_is_calibrated ? 100 : 0);
#else
			Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT, W2ST_COMMAND_CAL_STATUS, 0);
			Config_Notify(FEATURE_MASK_ECOMPASS, W2ST_COMMAND_CAL_STATUS, 0);
#endif

			FirstConnectionConfig = true;
		} else if (att_data[0] == 0)
			FirstConnectionConfig = false;

	} else if (attr_handle == ConfigCharHandle + 1) {

		/* Received one write command from Client on Configuration characteristic */
		ConfigCommandParsing(att_data, data_length);

	}
}

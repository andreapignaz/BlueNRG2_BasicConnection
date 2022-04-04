/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
 * File Name          : HWAdvanceFeature.c
 * Author             : IoT-EC Application team
 * Version            : V1.3.0
 * Date               : 20-July-2020
 * Description        : STEVAL-BCN002V1 Hardware advance feature file
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

#include <stdio.h>
#include "HWAdvanceFeatures.h"
#include "LSM6DSO_hal.h"
#include "sensor.h"

static uint16_t step_count = 0;
extern volatile HardwareFeaturePresence xHardwareFeaturePresence;
lsm6dso_pin_int1_route_t int1_route;

#if ENABLE_DEBUG
	#include <stdio.h>
	#define PRINTF(...) printf(__VA_ARGS__)
#else
	#define PRINTF(...)
#endif
/* Exported variables ---------------------------------------------------------*/
/* Imported Variables -------------------------------------------------------------*/

/**
 * @brief  This function Reads the default Acceleration Output Data Rate
 * @param  None
 * @retval None
 */
void InitHWFeatures(void) {

	//lsm6dso_int_notification_set(0, LSM6DSO_ALL_INT_PULSED);

}

/**
 * @brief  This function disables all the HW's Features
 * @param  None
 * @retval None
 */
void DisableHWFeatures(void) {

	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.md1_cfg.int1_6d = PROPERTY_DISABLE;
	int1_route.reg.md1_cfg.int1_double_tap = PROPERTY_DISABLE;
	int1_route.reg.md1_cfg.int1_emb_func = PROPERTY_DISABLE;
	int1_route.reg.md1_cfg.int1_ff = PROPERTY_DISABLE;
	int1_route.reg.md1_cfg.int1_single_tap = PROPERTY_DISABLE;
	int1_route.reg.md1_cfg.int1_sleep_change = PROPERTY_DISABLE;
	int1_route.reg.md1_cfg.int1_wu = PROPERTY_DISABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

 }

/**
 * @brief  This function enables the HW's 6D Orientation
 * @param  None
 * @retval None
 */
void EnableHWOrientation6D(void) {
	/* Disable all the HW features before */
	if (!xHardwareFeaturePresence.MultipleEvent)
		DisableHWFeatures();

	lsm6dso_6d_threshold_set(0, LSM6DSO_DEG_50);
	lsm6dso_xl_lp2_on_6d_set(0, PROPERTY_ENABLE);

	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.md1_cfg.int1_6d = PROPERTY_ENABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

}

/**
 * @brief  This function disables the HW's 6D Orientation
 * @param  None
 * @retval None
 */
void DisableHWOrientation6D(void) {

	lsm6dso_xl_lp2_on_6d_set(0, PROPERTY_DISABLE);
	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.md1_cfg.int1_6d = PROPERTY_DISABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);
}

/**
 * @brief  This function eturns the HW's 6D Orientation result
 *
 * @param  None
 * @retval AccEventType 6D Orientation Found
 */
AccEventType GetHWOrientation6D(void) {
	AccEventType OrientationResult = ACC_NOT_USED;

	lsm6dso_all_sources_t all_source;
	lsm6dso_all_sources_get(0, &all_source);

	if (all_source.reg.d6d_src.xh)
		OrientationResult = ACC_6D_OR_BOTTOM;
	if (all_source.reg.d6d_src.xl)
		OrientationResult = ACC_6D_OR_TOP;
	if (all_source.reg.d6d_src.yh)
		OrientationResult = ACC_6D_OR_RIGHT;
	if (all_source.reg.d6d_src.yl)
		OrientationResult = ACC_6D_OR_LEFT;
	if (all_source.reg.d6d_src.zh)
		OrientationResult = ACC_6D_OR_UP;
	if (all_source.reg.d6d_src.zl)
		OrientationResult = ACC_6D_OR_DOWN;

		PRINTF("Event: 6D Orientation, code: %d\r\n", OrientationResult);

	return OrientationResult;
}

/**
 * @brief  This function enables the HW's Tilt Detection
 * @param  None
 * @retval None
 */
void EnableHWTilt(void) {
	/* Disable all the HW features before */
	if (!xHardwareFeaturePresence.MultipleEvent)
		DisableHWFeatures();

	lsm6dso_tilt_sens_set(0, PROPERTY_ENABLE);
	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.emb_func_int1.int1_tilt = PROPERTY_ENABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

}

/**
 * @brief  This function disables the HW's Tilt Detection
 * @param  None
 * @retval None
 */
void DisableHWTilt(void) {

	lsm6dso_tilt_sens_set(0, PROPERTY_DISABLE);
	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.emb_func_int1.int1_tilt = PROPERTY_DISABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

}

/**
 * @brief  This function enables the HW's Wake Up Detection
 * @param  None
 * @retval None
 */
void EnableHWWakeUp(void) {

	DisableHWFeatures();

	lsm6dso_xl_hp_path_internal_set(0, LSM6DSO_USE_SLOPE);
	lsm6dso_wkup_threshold_set(0, 2);
	lsm6dso_wkup_dur_set(0, 1);
	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.md1_cfg.int1_wu = PROPERTY_ENABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

}

/**
 * @brief  This function disables the HW's Wake Up Detection
 * @param  None
 * @retval None
 */
void DisableHWWakeUp(void) {
	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.md1_cfg.int1_wu = PROPERTY_DISABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

}

/**
 * @brief  This function enables the HW's Free Fall Detection
 * @param  None
 * @retval None
 */
void EnableHWFreeFall(void) {

	DisableHWFeatures();

	lsm6dso_ff_dur_set(0, 0x02);
	lsm6dso_ff_threshold_set(0, LSM6DSO_FF_TSH_406mg);
	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.md1_cfg.int1_ff = PROPERTY_ENABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

}

/**
 * @brief  This function disables the HW's Free Fall Detection
 * @param  None
 * @retval None
 */
void DisableHWFreeFall(void) {

	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.md1_cfg.int1_ff = PROPERTY_DISABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

}

/**
 * @brief  This function enables the HW's Single Tap Detection
 * @param  None
 * @retval None
 */
void EnableHWSingleTap(void) {

	DisableHWFeatures();

	lsm6dso_tap_detection_on_z_set(0, PROPERTY_ENABLE);
	lsm6dso_tap_detection_on_y_set(0, PROPERTY_ENABLE);
	lsm6dso_tap_detection_on_x_set(0, PROPERTY_ENABLE);
	lsm6dso_tap_threshold_x_set(0, 0x04);
	lsm6dso_tap_threshold_y_set(0, 0x04);
	lsm6dso_tap_threshold_z_set(0, 0x04);
	lsm6dso_tap_dur_set(0, 0x02);
	lsm6dso_tap_quiet_set(0, 0x02);
	lsm6dso_tap_shock_set(0, 0x00);
	lsm6dso_tap_mode_set(0, LSM6DSO_ONLY_SINGLE);
	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.md1_cfg.int1_wu = PROPERTY_DISABLE;
	int1_route.reg.md1_cfg.int1_double_tap = PROPERTY_DISABLE;
	int1_route.reg.md1_cfg.int1_single_tap = PROPERTY_ENABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

}

/**
 * @brief  This function disables the HW's Single Tap Detection
 * @param  None
 * @retval None
 */
void DisableHWSingleTap(void) {

	lsm6dso_tap_detection_on_z_set(0, PROPERTY_DISABLE);
	lsm6dso_tap_detection_on_y_set(0, PROPERTY_DISABLE);
	lsm6dso_tap_detection_on_x_set(0, PROPERTY_DISABLE);
	lsm6dso_tap_dur_set(0, 0x02);
	lsm6dso_tap_quiet_set(0, 0x02);
	lsm6dso_tap_shock_set(0, 0x00);
	lsm6dso_tap_mode_set(0, LSM6DSO_BOTH_SINGLE_DOUBLE);
	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.md1_cfg.int1_double_tap = PROPERTY_DISABLE;
	int1_route.reg.md1_cfg.int1_single_tap = PROPERTY_DISABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);

}

/**
 * @brief  This function enables the HW's pedometer
 * @param  None
 * @retval None
 */
void EnableHWPedometer(void) {

	DisableHWFeatures();

	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.emb_func_int1.int1_step_detector = PROPERTY_ENABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);
	lsm6dso_pedo_sens_set(0, PROPERTY_ENABLE);
	lsm6dso_pedo_false_step_rejection_set(0, PROPERTY_ENABLE);

}

/**
 * @brief  This function disables the HW's pedometer
 * @param  None
 * @retval None
 */
void DisableHWPedometer(void) {
	lsm6dso_pin_int1_route_get(0, &int1_route);
	int1_route.reg.emb_func_int1.int1_step_detector = PROPERTY_DISABLE;
	lsm6dso_pin_int1_route_set(0, &int1_route);
	lsm6dso_pedo_sens_set(0, PROPERTY_DISABLE);
	lsm6dso_pedo_false_step_rejection_set(0, PROPERTY_DISABLE);

}

/**
 * @brief  This function resets the HW's pedometer steps counter
 * @param  None
 * @retval None
 */
void ResetHWPedometer(void) {
	lsm6dso_steps_reset(0);

}

/**
 * @brief  This function retunrs the HW's pedometer steps counter value
 * @param  None
 * @retval uint16_t Steps Counter
 */
uint16_t GetStepHWPedometer(void) {
	uint8_t steps[2] = { 0, 0 };
	lsm6dso_number_of_steps_get(0, steps);

	step_count = ((uint16_t) steps[0]) + (((uint16_t) steps[1]) << 8);
	return step_count;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

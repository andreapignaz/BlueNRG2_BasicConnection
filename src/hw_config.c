/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
 * File Name          : hw_config.c
 * Author             : IoT-EC Application team
 * Version            : V1.3.0
 * Date               : 20-July-2020
 * Description        : STEVAL-BCN002V1 Hardware configuration file
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

#include "hw_config.h"
#include "SDK_EVAL_Config.h"
#include "sensor.h"
#include "clock.h"

#if ENABLE_MOTIONFX
	#include "motion_fx_cm0p.h"			// MotionFX for CortexM0
#endif

#if ENABLE_OTA
	#include "OTA.h"
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

/* Private variables ---------------------------------------------------------*/

#if ENABLE_BLUEVOICE
	#define ADC_OUT_ADDRESS         (ADC_BASE + 0x16)
	extern volatile int16_t ADC_Buffer[PCM_BUFFER_SIZE];
#endif

ADC_InitType ADC_InitStructure;
DMA_InitType DMA_InitStructure;
NVIC_InitType NVIC_InitStructure;

/* Imported Variables -------------------------------------------------------------*/

void Platform_Init(void) {

	/* Configure I/O communication channel */
	SdkEvalComUartInit(UART_BAUDRATE);
	SdkEvalComUartIrqConfig(ENABLE);

	// Init the systick
	Clock_Init();

	/* GPIO Configuration */
	GPIO_Configuration();

	ADC_DMA_Configuration(ADC_Input_Microphone);

#ifdef ST_OTA_LOWER_APPLICATION
	PRINTF("\n\r # STEVAL-BCN002V1-L #\n\r");
#endif

#ifdef ST_OTA_HIGHER_APPLICATION
	PRINTF("\n\r # STEVAL-BCN002V1-H #\n\r");
#endif

	ADC_DMA_Configuration(ADC_Input_None);

}

/*******************************************************************************
 * Function Name  : GPIO_Configuration.
 * Description    : Configure outputs GPIO pins
 * Input          : None
 * Return         : None
 *******************************************************************************/
void GPIO_Configuration(void) {

	GPIO_InitType GPIO_InitStructure;

	/** GPIO Periph clock enable */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

	/** Init Structure */
	GPIO_StructInit(&GPIO_InitStructure);
	/** Configure GPIO_Pin_7 for Proximity XSHUT */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_Pull = DISABLE;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_WriteBit(GPIO_Pin_7, Bit_RESET);

#if ENABLE_BLUEVOICE
	/* Configure GPIO_Pin_1 and GPIO_Pin_2 as PDM_DATA and PDM_CLOCK */
	GPIO_InitPdmDataPin1()
	;
	GPIO_InitPdmClockPin2()
	;
#endif

}

/*******************************************************************************
 * Function Name  : Interrupts_EXT_IO_Config.
 * Description    : Configure interrupts on GPIO pins
 * Input          : None
 * Return         : None
 *******************************************************************************/
void Interrupts_EXT_IO_Config(void) {

	GPIO_InitType GPIO_InitStructure;
	GPIO_EXTIConfigType exti_config;
	NVIC_InitType NVIC_InitStructure;

	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MED_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Init Structure */
	GPIO_StructInit(&GPIO_InitStructure);

	/* Configure Wakeup IO pin */
	GPIO_InitStructure.GPIO_Mode = GPIO_Input;
	GPIO_InitStructure.GPIO_HighPwr = DISABLE;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Pull = DISABLE;
	GPIO_Init(&GPIO_InitStructure);

	/* Configure the Interrupt */
	exti_config.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	exti_config.GPIO_IrqSense = GPIO_IrqSense_Level;
	exti_config.GPIO_Event = GPIO_Event_Low;
	GPIO_EXTIConfig(&exti_config);

	GPIO_EXTICmd(GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13, DISABLE);

}

/*******************************************************************************
 * Function Name  : ADC_DMA_Configuration.
 * Description    : Init the ADC and the DMA.
 * Input          : Mode for Battery (No DMA) or Microphone (with DMA).
 * Return         : None.
 *******************************************************************************/
void ADC_DMA_Configuration(uint8_t xMode) {

	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_DMA, ENABLE);

	ADC_DmaCmd(DISABLE);
	DMA_Cmd(DMA_CH0, DISABLE);
	ADC_Cmd(DISABLE);
	NVIC_DisableIRQ(DMA_IRQn);
	DMA_ClearFlag(DMA_FLAG_HT0);
	DMA_ClearFlag(DMA_FLAG_TC0);
	ADC_DeInit();
	DMA_DeInit(DMA_CH0);
	DMA_SelectAdcChannel(DMA_ADC_CHANNEL0, DISABLE);

	/* Enable auto offset correction */
	ADC_Calibration(ENABLE);
	ADC_AutoOffsetUpdate(ENABLE);

	switch (xMode) {
	case ADC_Input_None:
		break;
	case ADC_Input_BattSensor:
		ADC_InitStructure.ADC_OSR = ADC_OSR_100;
		ADC_InitStructure.ADC_Input = ADC_Input_BattSensor;
		ADC_InitStructure.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6;
		ADC_InitStructure.ADC_Attenuation = ADC_Attenuation_9dB54;
		ADC_InitStructure.ADC_ConversionMode = ADC_ConversionMode_Single;
		ADC_Init(&ADC_InitStructure);

		/* Start conversion */
		ADC_Cmd(ENABLE);
		break;

	case ADC_Input_Microphone:
#if ENABLE_BLUEVOICE

		ADC_InitStructure.ADC_OSR = ADC_OSR_200;
		ADC_InitStructure.ADC_Input = ADC_Input_Microphone;
		ADC_InitStructure.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6;
		ADC_InitStructure.ADC_Attenuation = ADC_Attenuation_0dB;
		ADC_InitStructure.ADC_ConversionMode = ADC_ConversionMode_Continuous;
		ADC_Init(&ADC_InitStructure);
		/* Configure DMA TX channel */
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC_OUT_ADDRESS;
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_Buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize = (uint32_t) (PCM_BUFFER_SIZE);
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(DMA_CH0, &DMA_InitStructure);

		/* Enable DMA_CH0 Transfer and half transfer Complete interrupt */
		DMA_FlagConfig(DMA_CH0, DMA_FLAG_TC | DMA_FLAG_HT, ENABLE);
		/* Select DMA ADC CHANNEL 0 */
		DMA_SelectAdcChannel(DMA_ADC_CHANNEL0, ENABLE);

		/* Enable the DMA Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = DMA_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = HIGH_PRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		NVIC_EnableIRQ(DMA_IRQn);

		/* Start conversion */
		ADC_Cmd(ENABLE);
		DMA_Cmd(DMA_CH0, ENABLE);
		ADC_DmaCmd(ENABLE);
#endif
		break;

	default:
		break;
	}

}

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/

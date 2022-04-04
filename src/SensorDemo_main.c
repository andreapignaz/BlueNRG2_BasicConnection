//Interrupt definitions: BlueNRG1_it.h
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "sleep.h"
#include "SensorDemo_config.h"
#include "hw_config.h"
#include "osal.h"
#include "clock.h"

/*

SUGGESTIONS
1. Read the ST documentation on the BLUENRG2, in particular the Programmming Guidelines.
It has a lot of explaination on Bluetooth and GATT with nice programming examples.

2. Use the nRF connect app for Android to debug Bluetooth data for the device. 
It can list all the devices with their addresses. It also shows all the GATT services and characteristics.

*/

/* Set the Application Service Max number of attributes records with init parameters coming from application *.config.h file */
//uint8_t Services_Max_Attribute_Records[NUM_APP_GATT_SERVICES] = { MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_1, MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_2, MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_3 };
uint16_t led_count, system_count;

void Platform_Init(void);

int main(void) {
	
	/* System Init */
	SystemInit();

	/* Identify BlueNRG1 platform */
	SdkEvalIdentification();

	/* Init the Hardware platform */
	Platform_Init();
	SdkEvalLedInit(LED1); // Init Red LED
	SdkEvalLedInit(LED2); // Init Green LED
	SdkEvalLedInit(LED3); // Init Blue LED
	SdkEvalLedOn(LED3);   // Turn on Blue LED

	/* BlueNRG-2 stack Initialization + Public Address Definition */
	BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
	uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
	aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
	
	/* Variables */
 	uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
	uint8_t device_name[] = { 'B', 'C', 'N', '-', '0', '0', '2' };
	uint32_t number_of_executions;
	
	/*Power level configuration */
	aci_hal_set_tx_power_level(1, 0x07);
	
	/*GATT initialization*/
	aci_gatt_init();
	
	/*GAP initialization: peripheral role + handle(s) configuration*/
	aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	
	/*GAP device name setting*/
	aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name), device_name);
	
	/*Security setup: PIN is 123456*/
	aci_gap_set_authentication_requirement(BONDING, MITM_PROTECTION_REQUIRED, SC_IS_NOT_SUPPORTED, KEYPRESS_IS_NOT_SUPPORTED, 7, 16, USE_FIXED_PIN_FOR_PAIRING, 123456, 0x00);

	/*Service definition: define the UUID, copy it in the required structure, add the service and obtain the handle.
	  The handle is then used by the rest of the program.  */
	static uint16_t ServiceHandle;
	uint8_t string_service_uuid[16] = {0x1b,0xc5,0xd5,0xa5,0x02,0xb4,0x9a,0xe1,0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x00};
	Service_UUID_t service_uuid;
	Osal_MemCpy(&service_uuid.Service_UUID_128, string_service_uuid, 16);
	aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 6, &ServiceHandle);
	
	/*Characteristic definition: defined as service*/
	static uint16_t CharHandle;
	uint8_t string_char_uuid[16] = {0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,0xe1,0x11,0x01,0x00,0x00,0x00,0xE0,0x00};
	Char_UUID_t char_uuid;
	Osal_MemCpy(&char_uuid.Char_UUID_128, string_char_uuid, 16);
	aci_gatt_add_char(ServiceHandle, UUID_TYPE_128, &char_uuid, 2 + 3 * 3 * 2, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP, 16, 0, &CharHandle);

	/* Set device connectable (advertising) */
	/*Choose the name*/
	uint8_t local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'B','T','n','a','m','e' };
	/*Power level*/
	aci_hal_set_tx_power_level(1, 0x04);
	/*Set reponse data to scan*/
	hci_le_set_scan_response_data(0,NULL);
	/*Set discoverable with the public address and the nome defined before*/
	aci_gap_set_discoverable(ADV_IND, 0x160, 0x160, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);

	number_of_executions = 0;
	
	while (1) {
		/* BLE Stack Tick: let the Bluetooth FSM proceed with one tick */
		BTLE_StackTick();
		
		//Define a buffer that will contain the data to be written on the GATT characteristic
		uint8_t output_buffer[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		
		//Here a stupid example is used: the number of executions is saved, divided and published.
		number_of_executions = (number_of_executions + 1) % 3000;
			
		output_buffer[0] = number_of_executions;
	  
		aci_gatt_update_char_value(ServiceHandle, CharHandle, 0, 19, output_buffer);

	}/* while (1) */
}



/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/* Operations performed when connection with a device completes. */
void hci_le_connection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Role, uint8_t Peer_Address_Type, uint8_t Peer_Address[6], uint16_t Conn_Interval, uint16_t Conn_Latency, uint16_t Supervision_Timeout, uint8_t Master_Clock_Accuracy) {

	SdkEvalLedOff(LED1);
	SdkEvalLedOff(LED2);
	SdkEvalLedOff(LED3);
	
}

/*Operations performed when DISconnection from a device completes.*/
void hci_disconnection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Reason) {

	SdkEvalLedOff(LED1);
	SdkEvalLedOff(LED2);
	SdkEvalLedOn(LED3);

	/* Set device connectable (copied from main) */
	uint8_t local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'P','i','g','n','a' };
	aci_hal_set_tx_power_level(1, 0x04);
	hci_le_set_scan_response_data(0,NULL);
	aci_gap_set_discoverable(ADV_IND, 0x160, 0x160, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);

}



/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode) {
	if (SdkEvalComIOTxFifoNotEmpty())
		return SLEEPMODE_RUNNING;
	return sleepMode;
}

/***************************************************************************************/

void Platform_Init(void) {

	/* Configure I/O communication channel */
	SdkEvalComUartInit(UART_BAUDRATE);
	SdkEvalComUartIrqConfig(ENABLE);

	// Init the systick
	Clock_Init();

	/* GPIO Configuration */
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
}


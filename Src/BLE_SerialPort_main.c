/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : BLE_SerialPort_main.c
* Author             : RF Application Team
* Version            : V3.0.0
* Date               : 23-January-2010
* Description        : BlueNRG-LP main file for Serial Port demo (previously  
*                      known as Serial Port demo)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP Serial Port demo \see BLE_SerialPort_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "app_state.h"
#include "serial_port.h"
#include "SerialPort_config.h"
#include "OTA_btl.h" 
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "clock.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"
#include "packetHandler.h"
#if SERVER
#include "att_pwrq.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_SERIAL_PORT_VERSION_STRING "2.0.0" 
	 
#define TEST_DATA_SIZE 		2000//512 ZAO
	 
//#define ENABLE_RADIO_ACTIVITY_EVENT	 
	 
//LEDS
//BLUE - PA6
#define LED_BLUE_OFF() LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_HIGH)
#define LED_BLUE_ON() LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_LOW)
//GREEN - PB8
#define LED_GREEN_OFF()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_HIGH)	
#define LED_GREEN_ON()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_LOW)	
//RED -  PB9
#define LED_RED_OFF()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_HIGH)	
#define LED_RED_ON()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_LOW)	 
	 
uint8_t testDataArray[TEST_DATA_SIZE];
extern uint8_t enableDataSendFlag;
uint8_t packetSendDoneFlag = 0;
uint16_t globalConnectionHandle = 0;
extern uint16_t TXCharHandle, RXCharHandle;

volatile uint32_t sysTick;


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

#if USE_LONG_WRITE
uint32_t queued_write_buffer[QUEUED_WRITE_BUFFER_SIZE>>2];
#endif
   
/* Private function prototypes -----------------------------------------------*/
void InitTestDataArray(uint8_t dataArray[], uint16_t arraySize);
void SendTestDataArray(void);
/* Private functions ---------------------------------------------------------*/

void ModulesInit(void)
{
  uint8_t ret;
  BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;
  
  //enable clocking Public Key Accelerator and Rundom Number Generator
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG); 

  //Initialize Bluetooth Controller
  BLECNTR_InitGlobal();
  
  //Initialize Virtual Timer (many sw timers based on one hw timer)
  //HS_STARTUP_TIME = 780uS, INITIAL_CALIBRATION = FALSE, CALIBRATION_INTERVAL =v0
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  BLEPLAT_Init();  
  
  //initial PKA (Public Key Accelerator)
    if (PKAMGR_Init() == PKAMGR_ERROR)
  {
      while(1);
  }
  if (RNGMGR_Init() != RNGMGR_SUCCESS)
  {
      while(1);
  }
  
  /* Init the AES block */
  AESMGR_Init();
  
#if SERVER && USE_LONG_WRITE
  ATT_pwrq_init(sizeof(queued_write_buffer), (uint8_t *)queued_write_buffer);
#endif
  
  /* BlueNRG-LP stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
    while(1);
// init output pins

  }
	//===================
//PB0 --- generate notify
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
		LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS0, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS0, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_PUSHPULL);
//PA13 --- confirmation event
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
		LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS13, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS13, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_PUSHPULL);

//PA5 --- Pool Avalable for notification
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
		LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS5, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS5, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOA, GPIO_BSRR_BS5, LL_GPIO_OUTPUT_PUSHPULL);
		
//LED		
//PA6 --- BLUE
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
		LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS6, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS6, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_PUSHPULL);
		
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_HIGH);
//PB8 --- GREEN
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
		LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS8, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS8, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_PUSHPULL);
		
		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_HIGH);		
//PB9 --- RED
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
		LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS9, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS9, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_PUSHPULL);
		
		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_HIGH);	
		
//buttons
//PUSH 1 --- PA10
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
		LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS10, LL_GPIO_MODE_INPUT);
		LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS10, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinPull(GPIOA, GPIO_BSRR_BS10, LL_GPIO_PULL_NO);
//		LL_GPIO_SetPinOutputType(GPIOA, GPIO_BSRR_BS10, LL_GPIO_INPUT);		
		
//PUSH 2 --- PB6		
		LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
		LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS9, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS9, LL_GPIO_SPEED_FREQ_HIGH);
		LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_PUSHPULL);		
		
		
#ifdef ENABLE_RADIO_ACTIVITY_EVENT
				uint8_t status = aci_hal_set_radio_activity_mask(0x00FF);
				if(status == BLE_STATUS_SUCCESS)
				{
					printf("=========Radio activity event enable!!!\r\n");
				}
#endif
		
		
		
		
	InitTestDataArray(testDataArray, TEST_DATA_SIZE);
	InitTestDataArray(txBuffer, 32999);
	
		uint16_t bleStackTotalBufferSize = DYNAMIC_MEMORY_SIZE;
	printf("TotalBuffersize: \t\t%06d\r\n",BLE_STACK_InitParams.TotalBufferSize);
	printf("NumAttrRecords: \t\t%06d\r\n",BLE_STACK_InitParams.NumAttrRecords);
	printf("MaxNumOfClientProcs: \t%06d\r\n",BLE_STACK_InitParams.MaxNumOfClientProcs);
	printf("NumOfLinks: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfLinks);
	printf("NumOfEATTChannels: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfEATTChannels);
	printf("NumBlockCount: \t\t%06d\r\n",BLE_STACK_InitParams.NumBlockCount);
	printf("ATT_MTU: \t\t\t%06d\r\n",BLE_STACK_InitParams.ATT_MTU);
	printf("MaxConnEventLength: \t%06d\r\n",BLE_STACK_InitParams.MaxConnEventLength);
	printf("SleepClockAccuracy: \t%06d\r\n",BLE_STACK_InitParams.SleepClockAccuracy);
	printf("NumOfAdvDataSet: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfAdvDataSet);
	printf("NumOfAuxScanSlots: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfAuxScanSlots);
	printf("WhiteListSizeLog2: \t\t%06d\r\n",BLE_STACK_InitParams.WhiteListSizeLog2);
	printf("L2CAP_MPS: \t\t%06d\r\n",BLE_STACK_InitParams.L2CAP_MPS);
	printf("L2CAP_NumChannels: \t\t%06d\r\n",BLE_STACK_InitParams.L2CAP_NumChannels);	
	printf("NumOfSyncSlots: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfSyncSlots);
	printf("CTE_MaxNumAntennaIDs: \t%06d\r\n",BLE_STACK_InitParams.CTE_MaxNumAntennaIDs);
	printf("CTE_MaxNumIQSamples: \t%06d\r\n",BLE_STACK_InitParams.CTE_MaxNumIQSamples);
	printf("isr0_fifo_size: \t\t%06d\r\n",BLE_STACK_InitParams.isr0_fifo_size);
	printf("isr1_fifo_size: \t\t%06d\r\n",BLE_STACK_InitParams.isr1_fifo_size);
	printf("user_fifo_size: \t\t%06d\r\n",BLE_STACK_InitParams.user_fifo_size);
}


void ModulesTick(void)
{
  /* Timer tick */
  HAL_VTIMER_Tick();
  
  /* Bluetooth stack tick */
  BLE_STACK_Tick();
  
  /* NVM manager tick */
  NVMDB_Tick();
}

int main(void)
{
  uint8_t ret;

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();

  /* Init Clock */
  Clock_Init();

  /* Configure I/O communication channel */
  BSP_COM_Init(Process_InputData);

  ModulesInit();
	LED_GREEN_OFF();
	LED_RED_ON();
	
//ZAO - start test section
	uint8_t status;
	uint16_t suggestedMaxTxOctets = 251;// bytes
	uint16_t suggestedMaxTxTime = (251 + 14)*8;//microsecunds
	status = hci_le_write_suggested_default_data_length(suggestedMaxTxOctets,
                                                      suggestedMaxTxTime);
	if(status != BLE_STATUS_SUCCESS)
	{
		printf("====Function: hci_le_write_suggested_default_data_length()\r\n");
		printf("return error code: 0x%02x\r\n", status);
	}
	else
	{
		printf("====Function: hci_le_write_suggested_default_data_length()\r\n");
		printf("return OK\r\n");		
	}
//ZAO - end test section		

#if SERVER
  printf("BlueNRG-LP BLE Serial Port Server Application (version: %s)\r\n", BLE_SERIAL_PORT_VERSION_STRING);
#else
  printf("BlueNRG-LP BLE Serial Port Client Application (version: %s)\r\n", BLE_SERIAL_PORT_VERSION_STRING); 
#endif

  /* Init Serial port Device */
  ret = Serial_port_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Serial_port_DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }
  
  printf("BLE Stack Initialized \n");
  
#if CONFIG_OTA_USE_SERVICE_MANAGER
  /* Initialize the button */
  BSP_PB_Init(USER_BUTTON, BUTTON_MODE_GPIO); 
#endif /* CONFIG_OTA_USE_SERVICE_MANAGER */
   
  while(1) {
    
		if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_10) == 1)
		{
			LED_BLUE_ON();
		}
		else
		{
			LED_BLUE_OFF();
		}

		
		
    ModulesTick();
    PacketHandler();
#ifdef PERIODIC_SEND_ENABLE
		PeriodicNotificationSend(periodicSendDataEnable);
#endif //PERIODIC_SEND_ENABLE
		
    /* Application tick */
//		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS1, LL_GPIO_OUTPUT_HIGH);
    APP_Tick();
//		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS1, LL_GPIO_OUTPUT_LOW);
    SendTestDataArray();
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    /* Check if the OTA firmware upgrade session has been completed */
    if (OTA_Tick() == 1)
    {
      /* Jump to the new application */
      OTA_Jump_To_New_Application();
    }
#endif  /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */

#if CONFIG_OTA_USE_SERVICE_MANAGER
    if (BSP_PB_GetState(USER_BUTTON) == SET) 
    {
      OTA_Jump_To_Service_Manager_Application();
    }
#endif /* CONFIG_OTA_USE_SERVICE_MANAGER */
  }
  
} /* end main() */


/* Event used to notify the Host that a hardware failure has occurred in the Controller. 
   See bluenrg_lp_events.h. */
void hci_hardware_error_event(uint8_t Hardware_Code)
{
  if (Hardware_Code <= 0x03)
  {
    NVIC_SystemReset();
  }
}

/**
  * This event is generated to report firmware error informations.
  * FW_Error_Type possible values: 
  * Values:
  - 0x01: L2CAP recombination failure
  - 0x02: GATT unexpected response
  - 0x03: GATT unexpected request
    After this event with error type (0x01, 0x02, 0x3) it is recommended to disconnect. 
*/
void aci_hal_fw_error_event(uint8_t FW_Error_Type,
                            uint8_t Data_Length,
                            uint8_t Data[])
{
  if (FW_Error_Type <= 0x03)
  {
    uint16_t connHandle;
    
    /* Data field is the connection handle where error has occurred */
    connHandle = LE_TO_HOST_16(Data);
    
    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER); 
  }
}

//=============================================================================
void InitTestDataArray(uint8_t dataArray[], uint16_t arraySize)
{
	uint8_t value = 0;
	for(uint16_t i = 0; i <= arraySize; i++)
	{

		dataArray[i] = value;
		value++;
	}
}
//=============================================================================
void SendTestDataArray(void)
{
	if(enableDataSendFlag == 0)
		return;
	static uint16_t repeatCount = 0;
	//check packetSendDoneFlag from callback event
	if(packetSendDoneFlag == 1)
	{
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS1, LL_GPIO_OUTPUT_HIGH);
		packetSendDoneFlag = 0;
		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_HIGH);
		uint8_t status;
		status = aci_gatt_srv_notify(globalConnectionHandle, TXCharHandle + 1, 0x01, 1000,
                                            (uint8_t *)(testDataArray));
		if(status == BLE_STATUS_SUCCESS)
		{
			printf("Data in buffer\r\n");
		} else
		{
			printf("Error code: 0x%02x", status);
		}
//TEST SECTION START

		while ( (status = aci_gatt_srv_notify(globalConnectionHandle, TXCharHandle + 1, 0x01, 1000,(uint8_t *)(testDataArray + 1000)))  == 0x0C)
		status = aci_gatt_srv_notify(globalConnectionHandle, TXCharHandle + 1, 0x01, 1000,(uint8_t *)(testDataArray + 1000));	
				if(status == BLE_STATUS_SUCCESS)
		{
			printf("Data in buffer\r\n");
		} else
		{
			printf("Error code: 0x%02x", status);
		}
//TEST SECTION END

		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_LOW);		
		repeatCount++;
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS1, LL_GPIO_OUTPUT_LOW);
	}
	
//stop send data - all data has been send	
	if(repeatCount >= 65 )
	{
		repeatCount = 0;
		enableDataSendFlag = 0;	
	}
	
}
//=============================================================================
#if SERVER
#if USE_LONG_WRITE

#endif//USE_LONG_WRITE
#endif//SERVER	

#ifdef ENABLE_RADIO_ACTIVITY_EVENT
void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_HIGH);
		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_LOW);
}
#endif
//=============================================================================

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */

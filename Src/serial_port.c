/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : serial_port.c
* Author             : AMS - VMA RF  Application team
* Version            : V3.0.0
* Date               : 23-January-2020
* Description        : This file handles bytes received from USB and the init
*                      function.
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
#include "gp_timer.h" 
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "app_state.h"
#include "osal.h"
#include "gatt_db.h"
#include "serial_port.h"
#include "gap_profile.h"
#if defined (CONFIG_OTA_LOWER) || defined (CONFIG_OTA_HIGHER)
#include "OTA_btl.h" 
#endif
#include "bluenrg_lp_evb_com.h"
#if SERVER
#include "att_pwrq.h"
#endif
#include "packetHandler.h"
/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define CMD_BUFF_SIZE 1020//ZAO 512

#define ADV_INTERVAL_MIN    ((uint32_t)(100/0.625))     // 100 ms
#define ADV_INTERVAL_MAX    ((uint32_t)(100/0.625))     // 100 ms
#define SCAN_INTERVAL       ((uint16_t)(400/0.625))     // 400 ms
#define SCAN_WINDOW         ((uint16_t)(100/0.625))     // 100 ms
#define CONN_INTERVAL_MIN   ((uint16_t)(7.5/1.25))       // 20 ms
#define CONN_INTERVAL_MAX   ((uint16_t)(7.5/1.25))       // 20 ms
#define SUPERVISION_TIMEOUT ((uint16_t)(1000/10))       // 1000 ms
#define CE_LENGTH           ((uint16_t)(40/0.625))      // 20 ms

#define BD_ADDR_MASTER      0xa1, 0x00, 0x00, 0xE1, 0x80, 0x02
#define BD_ADDR_SLAVE       0xa2, 0x00, 0x00, 0xE1, 0x80, 0x02

//write additional information to the printf() function
#define ZAO_PRINT_ADDITIONAL_INFO

#define LED_BLUE_OFF() LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_HIGH)
#define LED_BLUE_ON() LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_LOW)
//GREEN - PB8
#define LED_GREEN_OFF()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_HIGH)	
#define LED_GREEN_ON()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_LOW)	
//RED -  PB9
#define LED_RED_OFF()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_HIGH)	
#define LED_RED_ON()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_LOW)

#define PERIODIC_NOTIFICATION_TIMER 15000// millisec

uint8_t txBuffer[TX_BUFFER_SIZE];
uint8_t periodicSendDataEnable = 0;
/*
*Template for additiional information
*
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====()====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
*/

#ifdef SERVER

#define LOCAL_NAME  'S','E','R','V','E','R','-','-'
#else
#define LOCAL_NAME  'C','L','I','E','N','T','-','-'
#endif //SERVER//



#define MANUF_DATA_SIZE (30)

extern uint16_t globalConnectionHandle;
extern uint8_t enableDataSendFlag;
extern uint8_t packetSendDoneFlag;

packetHandlerSettings_t phSettings;

void TxSendDone(void);






/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


volatile int app_flags = SET_CONNECTABLE;
static  uint16_t connection_handle = 0;
#if SERVER && USE_LONG_WRITE
static uint8_t rx_char_val_buff[RX_CHR_BUFFER_SIZE];
#endif

/** 
  * @brief  Handle of TX,RX  Characteristics.
  */ 
#if CLIENT
uint16_t tx_handle;
uint16_t rx_handle;
#endif 

/* UUIDs */
UUID_t UUID_Tx;
UUID_t UUID_Rx;

static char cmd[CMD_BUFF_SIZE];
static uint16_t cmd_buff_end = 0;
//static uint8_t att_mtu = BLE_STACK_DEFAULT_ATT_MTU;
#if !(USE_LONG_WRITE && CLIENT)
static uint16_t cmd_buff_start = 0;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Serial_port_DeviceInit.
* Description    : Init the Serial_port device.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t Serial_port_DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t name[] = {'B', 'l', 'u', 'e', 'N', 'R', 'G', 'L','P'};
  
#if SERVER
  uint8_t role = GAP_PERIPHERAL_ROLE;
  uint8_t bdaddr[] = {BD_ADDR_SLAVE};
#else
  uint8_t role = GAP_CENTRAL_ROLE;
  uint8_t bdaddr[] = {BD_ADDR_MASTER};
#endif 
  
  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_hal_write_config_data()====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  if(ret != BLE_STATUS_SUCCESS){
    printf("Setting BD_ADDR failed: 0x%02x\r\n", ret);
    return ret;
  }

  /* Set the TX power to 0 dBm */
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_halset_tx_power_level(0,24)====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  aci_hal_set_tx_power_level(1, 31);//ZAO it was 0,24

  /* GATT Init */
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gatt_srv_init()====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO	
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in aci_gatt_srv_init(): 0x%02x\r\n", ret);
    return ret;
  } else {
    printf ("aci_gatt_srv_init() --> SUCCESS\r\n");
  }
  
  /* GAP Init */
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gap_init()====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  ret = aci_gap_init(role, 0x00, 0x09, PUBLIC_ADDR, &service_handle,  
                     &dev_name_char_handle, &appearance_char_handle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in aci_gap_init() 0x%02x\r\n", ret);
    return ret;
  } else {
    printf ("aci_gap_init() --> SUCCESS\r\n");
  }

  /* Set the device name */
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====Gap_profile_set_dev_name()====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  ret = Gap_profile_set_dev_name(0, sizeof(name), name);
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in Gap_profile_set_dev_name 0x%02x\r\n", ret);
    return ret;
  } else {
    printf ("Gap_profile_set_dev_name() --> SUCCESS\r\n");
  }
  
#if SERVER
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====Add_Serial_port_Service()====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  ret = Add_Serial_port_Service();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in Add_Serial_port_Service 0x%02x\r\n", ret);
    return ret;
  } else {
    printf("Add_Serial_port_Service() --> SUCCESS\r\n");
  }
  
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT     
  ret = OTA_Add_Btl_Service();
  if(ret == BLE_STATUS_SUCCESS)
    printf("OTA service added successfully.\n");
  else
    printf("Error while adding OTA service.\n");
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
  #ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gap_set_advertising_configuration()====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                              ADV_INTERVAL_MIN, ADV_INTERVAL_MAX,
                                              ADV_CH_ALL,
                                              0,NULL,
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  printf("Advertising configuration %02X\n", ret);
  
  static uint8_t manuf_data[MANUF_DATA_SIZE] = { 
    0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
    2,                      /* Length of AD type Transmission Power */
    0x0A, 0x00,             /* Transmission Power = 0 dBm */ 
    9,                      /* Length of AD type Complete Local Name */
    0x09,                   /* AD type Complete Local Name */ 
    LOCAL_NAME,             /* Local Name */            
    13,                     /* Length of AD type Manufacturer info */
    0xFF,                   /* AD type Manufacturer info */
    0x01,                   /* Protocol version */
    0x05,		      /* Device ID: 0x05 = STEVAL-BCN002V1 Board */
    0x00,                   /* Feature Mask byte#1 */
    0x00,                   /* Feature Mask byte#2 */
    0x00,                   /* Feature Mask byte#3 */
    0x00,                   /* Feature Mask byte#4 */
    0x00,                   /* BLE MAC start */
    0x00, 
    0x00, 
    0x00, 
    0x00, 
    0x00                    /* BLE MAC stop */
  };
  
  for (int var = 0; var < 6; ++var) {
    manuf_data[MANUF_DATA_SIZE - 1 - var] = bdaddr[var];
  }
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gap_set_advertising_data()====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(manuf_data), manuf_data);
  
  printf("Set advertising data %02X\n", ret);  
  
#endif /* SERVER */
  
#if CLIENT
  #ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gap_set_scan_configuration() [client function]====");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED,SCAN_ACCEPT_ALL, LE_1M_PHY_BIT, PASSIVE_SCAN, SCAN_INTERVAL, SCAN_WINDOW);
  
  printf("Scan configuration %02X\n", ret);
  #ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gap_set_connection_configuration() [client functon]====");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  ret = aci_gap_set_connection_configuration(LE_1M_PHY_BIT, CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, 0, SUPERVISION_TIMEOUT, CE_LENGTH, CE_LENGTH);
  
  printf("Connection configuration %02X\n", ret);
  
#endif /* CLIENT */
  
  return BLE_STATUS_SUCCESS;
}

#if (USE_LONG_WRITE && CLIENT)

void Send_Data_Over_BLE(void)
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====Send_Data_Over_BLE() [function begin]====");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  uint8_t ret;
  static ble_gatt_clt_write_ops_t write_op;
      
  if(!APP_FLAG(SEND_DATA) || APP_FLAG(SENDING_DATA))
    return;
  
  if(cmd_buff_end <= att_mtu - 3)
  {
    ret = aci_gatt_clt_write(connection_handle, rx_handle + 1, cmd_buff_end, (uint8_t *)cmd);
  }
  else
  {
    write_op.attr_h = rx_handle + 1;
    write_op.attr_offset = 0;
    write_op.data_len = cmd_buff_end;
    write_op.data_p = (uint8_t *)cmd;
    ret = aci_gatt_clt_write_long(connection_handle, &write_op);
  }
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf("Error to send data (0x%2x\n)", ret);
    NVIC_EnableIRQ(BSP_UART_IRQn); 
  }
  else 
  {
    /* Wait for procedure to end */
    APP_FLAG_SET(SENDING_DATA);
  }
  
  APP_FLAG_CLEAR(SEND_DATA);
  cmd_buff_end = 0;
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====Send_Data_Over_BLE() [function end]====");
	#endif//ZAO_PRINT_ADDITONAL_INFO
}

#else /* !(USE_LONG_WRITE && CLIENT) */

void Send_Data_Over_BLE(void)
{

    uint8_t ret;
// exit if tx buffer full or no data to send
  if(!APP_FLAG(SEND_DATA) || APP_FLAG(TX_BUFFER_FULL))
    return;
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====Send_Data_Over_BLE()[client function begin]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO  
  while(cmd_buff_start < cmd_buff_end)
  {
    uint32_t len = MIN(1020, cmd_buff_end - cmd_buff_start);//ZAO 20 to 1020
    
#if SERVER
		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_HIGH);//trigger		
//    ret = aci_gatt_srv_notify(connection_handle, TXCharHandle + 1, 0, len,
 //                                           (uint8_t *)(cmd + cmd_buff_start));
// indication 
		printf("Message lenght: %04d", len);
    ret = aci_gatt_srv_notify(connection_handle, TXCharHandle + 1, 0x02, len,
                                            (uint8_t *)(cmd + cmd_buff_start));		
		
		
		
		
		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_LOW);//trigger				
		
		
#elif CLIENT
    ret = aci_gatt_clt_write_without_resp(connection_handle, rx_handle + 1, len,
                                          (uint8_t *)(cmd + cmd_buff_start));
#else
#error "Define SERVER or CLIENT"
#endif
    if (ret != BLE_STATUS_SUCCESS)
    {
        printf("Error to send ");
#if SERVER
        printf("notify ");
#else
        printf("cmd ");
#endif
        printf("with error code = 0x%2x\n", ret);
    }
    if(ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
    {
      APP_FLAG_SET(TX_BUFFER_FULL);
      return;
    }
    cmd_buff_start += len;
  }
  
  // All data from buffer have been sent.
  APP_FLAG_CLEAR(SEND_DATA);
  cmd_buff_end = 0;
  NVIC_EnableIRQ(BSP_UART_IRQn);
		#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====Send_Data_Over_BLE()[client function end]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
	
}

#endif /* end of #if USE_LONG_WRITE && CLIENT */

/*******************************************************************************
* Function Name  : Process_InputData.
* Description    : Process a command. It should be called when data are received.
* Input          : data_buffer: data address.
*	           Nb_bytes: number of received bytes.
* Return         : none.
*******************************************************************************/
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  uint8_t i;
// check 'T\r\n 'sequence
	
// if got this sequence call 


	//===========================
  for (i = 0; i < Nb_bytes; i++)
  {
		// ring buffer
    if(cmd_buff_end >= CMD_BUFF_SIZE-1){
      cmd_buff_end = 0;
    }
    
    cmd[cmd_buff_end] = data_buffer[i];
    BSP_COM_Write(&data_buffer[i], 1); 
    cmd_buff_end++;
//enter 'EOS' symbol if read \n or \r    
    if((cmd[cmd_buff_end-1] == '\n') || (cmd[cmd_buff_end-1] == '\r')){
      if(cmd_buff_end != 1){
        
        cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.
        
        // Set flag to send data. Disable UART IRQ to avoid overwriting buffer with new incoming data
        APP_FLAG_SET(SEND_DATA);
        NVIC_DisableIRQ(BSP_UART_IRQn);
#if !(USE_LONG_WRITE && CLIENT)
        cmd_buff_start = 0;        
#endif        
      }
      else {
        cmd_buff_end = 0; // Discard
      }
    }
  }
}


/*******************************************************************************
* Function Name  : Make_Connection.
* Description    : If the device is a Client create the connection. Otherwise puts
*                  the device in discoverable mode.
* Input          : none.
* Return         : none.
*******************************************************************************/
void Make_Connection(void)
{  
  tBleStatus ret;
  
#if CLIENT
  tBDAddr bdaddr = {BD_ADDR_SLAVE}; 
  
  printf("Connecting...\n");
  
  ret = aci_gap_create_connection(LE_1M_PHY_BIT, PUBLIC_ADDR, bdaddr);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf("Error while starting connection: 0x%04x\r\n", ret);
    Clock_Wait(100);        
  }
  
#else
  
  static Advertising_Set_Parameters_t Advertising_Set_Parameters[1];
  
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  ret = aci_gap_set_scan_response_data(0,18,BTLServiceUUID4Scan);
  if(ret != BLE_STATUS_SUCCESS)
  {
    printf("aci_gap_set_scan_response_data() failed: 0x%02x\r\n",ret);
  }
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */  
  
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  if (ret != BLE_STATUS_SUCCESS)
    printf ("Error in aci_gap_set_advertising_enable(): 0x%02x\r\n", ret);
  else
    printf ("aci_gap_set_advertising_enable() --> SUCCESS\r\n");
#endif
}

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Tick to run the application state machine.
* Input          : none.
* Return         : none.
*******************************************************************************/
void APP_Tick(void)
{
#if CLIENT
  tBleStatus ret;
#endif
  
  if(APP_FLAG(SET_CONNECTABLE))
  {
    Make_Connection();
    APP_FLAG_CLEAR(SET_CONNECTABLE);
  }
  
  Send_Data_Over_BLE();

#if REQUEST_CONN_PARAM_UPDATE    
  if(APP_FLAG(CONNECTED) && !APP_FLAG(L2CAP_PARAM_UPD_SENT) && Timer_Expired(&l2cap_req_timer))
  {
    aci_l2cap_connection_parameter_update_req(connection_handle, 8, 16, 0, 600);
    APP_FLAG_SET(L2CAP_PARAM_UPD_SENT);
  }
#endif
  
#if CLIENT
  /* Start TX handle Characteristic discovery if not yet done */
  if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
  {
    if (!APP_FLAG(START_READ_TX_CHAR_HANDLE))
    {
      /* Discovery TX characteristic handle by UUID 128 bits */
        UUID_t uuid = {.UUID_128 = {TX_CHR_UUID}};

        ret = aci_gatt_clt_disc_char_by_uuid(connection_handle, 0x0001, 0xFFFF, 2, &uuid);
      if (ret != 0) 
        {
            printf ("Error in aci_gatt_clt_disc_char_by_uuid() for TX characteristic: 0x%04xr\n", ret);
        }
      else
        {
            printf ("aci_gatt_clt_disc_char_by_uuid() for TX characteristic --> SUCCESS\r\n");
        }
      APP_FLAG_SET(START_READ_TX_CHAR_HANDLE);
    }
  }
  /* Start RX handle Characteristic discovery if not yet done */
  else if (APP_FLAG(CONNECTED) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
  {
    /* Discovery RX characteristic handle by UUID 128 bits */
    if (!APP_FLAG(START_READ_RX_CHAR_HANDLE))
    {
        /* Discovery RX characteristic handle by UUID 128 bits */
        UUID_t uuid = {.UUID_128 = {RX_CHR_UUID}};

        ret = aci_gatt_clt_disc_char_by_uuid(connection_handle, 0x0001, 0xFFFF, 2, &uuid);
      if (ret != 0) 
        {
            printf ("Error in aci_gatt_clt_disc_char_by_uuid() for RX characteristic: 0x%04xr\n", ret);
        }
      else
        {
            printf ("aci_gatt_clt_disc_char_by_uuid() for RX characteristic --> SUCCESS\r\n");
        }
      APP_FLAG_SET(START_READ_RX_CHAR_HANDLE);
    }
  }
  
  if(APP_FLAG(CONNECTED) && APP_FLAG(END_READ_TX_CHAR_HANDLE) && APP_FLAG(END_READ_RX_CHAR_HANDLE) && !APP_FLAG(NOTIFICATIONS_ENABLED))
  {
    static uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
    
    printf("subscribe to notification\n");
    
    ret = aci_gatt_clt_write(connection_handle, tx_handle + 2,
                                 sizeof(client_char_conf_data),
                                 client_char_conf_data);
    if (ret != 0)
    {
      printf("Error: not able to enable notifications: 0x%02xr\n", ret);
    }
    else
    {
      printf("Notifications enabled\n");
    }
    
    APP_FLAG_SET(NOTIFICATIONS_ENABLED);
  }
#endif
  
}/* end APP_Tick() */


void Data_Received(uint16_t length, uint8_t *data)
{
  for(uint16_t i = 0U; i < length; i++)
  {
    printf("%c", data[i]);
  }  
}


/* ***************** BlueNRG-LP Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====hci_le_connection_complete_event() [callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
	LED_GREEN_ON();
	LED_RED_OFF();
	periodicSendDataEnable = 1;
  if(Status != BLE_STATUS_SUCCESS)
    return;
  
  connection_handle = Connection_Handle;
	globalConnectionHandle = connection_handle;
	
//START TEST CODE
	
	phSettings.cltOrSrv = SERVERS;
	phSettings.notificationType = NOTIFICATION;
	PacketHandlerInit(connection_handle,
										TXCharHandle,
										phSettings.cltOrSrv,
										phSettings.notificationType,
										0);
	PacketHandlerTxDoneSetCallback(&TxSendDone);

	
//END TEST CODE	
	
  
  APP_FLAG_SET(CONNECTED);
  
  printf("Connection, Status: 0x%04X\n", Status); 
  
#if REQUEST_CONN_PARAM_UPDATE
  APP_FLAG_CLEAR(L2CAP_PARAM_UPD_SENT);
  Timer_Set(&l2cap_req_timer, CLOCK_SECOND*2);
#endif
}/* end hci_le_connection_complete_event() */




/*******************************************************************************
 * Function Name  : hci_le_enhanced_connection_complete_event.
 * Description    : This event indicates that a new connection has been created
 * Input          : See file bluenrg_lp_events.h
 * Output         : See file bluenrg_lp_events.h
 * Return         : See file bluenrg_lp_events.h
 *******************************************************************************/
void hci_le_enhanced_connection_complete_event(uint8_t Status,
                                               uint16_t Connection_Handle,
                                               uint8_t Role,
                                               uint8_t Peer_Address_Type,
                                               uint8_t Peer_Address[6],
                                               uint8_t Local_Resolvable_Private_Address[6],
                                               uint8_t Peer_Resolvable_Private_Address[6],
                                               uint16_t Conn_Interval,
                                               uint16_t Conn_Latency,
                                               uint16_t Supervision_Timeout,
                                               uint8_t Master_Clock_Accuracy)

{
  #ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====hci_le_connection_complete_event() [functon call from callback function hci_le_enhanced_connection_conplete_event()]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  hci_le_connection_complete_event(Status,
                                   Connection_Handle,
                                   Role,
                                   Peer_Address_Type,
                                   Peer_Address,
                                   Conn_Interval,
                                   Conn_Latency,
                                   Supervision_Timeout,
                                   Master_Clock_Accuracy);
}

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====hci_disconnection_complete_event()[callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
	LED_GREEN_OFF();
	LED_RED_ON();
	periodicSendDataEnable = 0;
  APP_FLAG_CLEAR(CONNECTED);
  /* Make the device connectable again. */
  APP_FLAG_SET(SET_CONNECTABLE);
  APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
  APP_FLAG_CLEAR(TX_BUFFER_FULL);

  APP_FLAG_CLEAR(START_READ_TX_CHAR_HANDLE);
  APP_FLAG_CLEAR(END_READ_TX_CHAR_HANDLE);
  APP_FLAG_CLEAR(START_READ_RX_CHAR_HANDLE); 
  APP_FLAG_CLEAR(END_READ_RX_CHAR_HANDLE);
  
  printf("Disconnection, Reason: 0x%04X\n", Reason); 

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  OTA_terminate_connection();
#endif 
  
}/* end hci_disconnection_complete_event() */


/*******************************************************************************
 * Function Name  : aci_gatt_srv_attribute_modified_event.
 * Description    : This event occurs when an attribute is modified.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_srv_attribute_modified_event(uint16_t Connection_Handle,
                                           uint16_t Attr_Handle,
                                           uint16_t Attr_Data_Length,
                                           uint8_t Attr_Data[])
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gatt_srv_attribute_modified_event()[callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    OTA_Write_Request_CB(Connection_Handle, Attr_Handle, Attr_Data_Length, Attr_Data);
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
  
    if(Attr_Handle == TXCharHandle + 2)
    {
        if(Attr_Data[0] == 0x01)
        {
            APP_FLAG_SET(NOTIFICATIONS_ENABLED);
        }
    }
    else
    {
        printf("Received data from not recognized attribute handle 0x%4X\n",
               Attr_Handle);
    }
}

void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t Server_RX_MTU)
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_att_exchange_mtu_resp_event()[callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  OTA_att_exchange_mtu_resp_CB(Connection_Handle, Server_RX_MTU);
#endif
//  att_mtu = Server_RX_MTU;
  printf("ATT MTU = %05d\n", Server_RX_MTU);
}

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
  OTA_data_length_change_CB(Connection_Handle);  
}
#endif

#if CLIENT

/*******************************************************************************
 * Function Name  : aci_gatt_clt_notification_event.
 * Description    : This event occurs when a notification is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_clt_notification_event(uint16_t Connection_Handle,
                                     uint16_t Attribute_Handle,
                                     uint16_t Attribute_Value_Length,
                                     uint8_t Attribute_Value[])
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gatt_clt_notification_event()[clent callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  uint16_t attr_handle;
 
  attr_handle = Attribute_Handle;
    if(attr_handle == tx_handle+1)
    {
      for(int i = 0; i < Attribute_Value_Length; i++) 
          printf("%c", Attribute_Value[i]);
    }
    else
    {
        printf("Received data from not recognized attribute handle 0x%4X\n", Attribute_Handle);
    }
}

/*******************************************************************************
 * Function Name  : aci_gatt_clt_disc_read_char_by_uuid_resp_event.
 * Description    : This event occurs when a discovery read characteristic by UUID response.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_clt_disc_read_char_by_uuid_resp_event(uint16_t Connection_Handle,
                                                uint16_t Attribute_Handle,
                                                uint8_t Attribute_Value_Length,
                                                uint8_t Attribute_Value[])
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gatt_clt_disc_read_char_by_uuid_resp_event()[client callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  printf("aci_gatt_clt_disc_read_char_by_uuid_resp_event, Connection Handle: 0x%04X\n", Connection_Handle);
  if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
  {
    tx_handle = Attribute_Handle;
    printf("TX Char Handle 0x%04X\n", tx_handle);
  }
  else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
  {
    rx_handle = Attribute_Handle;
    printf("RX Char Handle 0x%04X\n", rx_handle);
  }
}

/*******************************************************************************
 * Function Name  : aci_gatt_clt_proc_complete_event.
 * Description    : This event occurs when a GATT procedure complete is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_clt_proc_complete_event(uint16_t Connection_Handle,
                                      uint8_t Error_Code)
{ 
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gatt_clt_proc_complete_event()[client callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
  if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
  {
    printf("aci_GATT_PROCEDURE_COMPLETE_Event for START_READ_TX_CHAR_HANDLE \r\n");
    APP_FLAG_SET(END_READ_TX_CHAR_HANDLE);
  }
  else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
  {
    printf("aci_GATT_PROCEDURE_COMPLETE_Event for START_READ_RX_CHAR_HANDLE \r\n");
    APP_FLAG_SET(END_READ_RX_CHAR_HANDLE);
  }
  else if (APP_FLAG(SENDING_DATA))
  {
    APP_FLAG_CLEAR(SENDING_DATA);
    NVIC_EnableIRQ(BSP_UART_IRQn);
  }
}

#endif /* CLIENT */

/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : This event occurs when a TX pool available is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
//void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
//                                      uint16_t Available_Buffers)
//{
//	#ifdef ZAO_PRINT_ADDITIONAL_INFO
//	printf("====aci_gatt_tx_pool_available_event()[callback] Avl_Buff: %05d\n",
//				 Available_Buffers);
//	#endif//ZAO_PRINT_ADDITONAL_INFO
//  /* It allows to notify when at least 2 GATT TX buffers are available */
//  APP_FLAG_CLEAR(TX_BUFFER_FULL);
//} 
//==============================================================================

//void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
//                                         uint8_t Next_State,
//                                         uint32_t Next_State_SysTime)
//{
//	#ifdef ZAO_PRINT_ADDITIONAL_INFO
//	printf("====aci_hal_end_of_radio_activity_event()[callback]====\n");
//	#endif//ZAO_PRINT_ADDITONAL_INFO
//#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
//  if (Next_State == 0x02) /* 0x02: Connection event slave */
//  {
//    OTA_Radio_Activity(Next_State_SysTime);  
//  }
//#endif 
//}

//==============================================================================
#if SERVER
void aci_gatt_srv_write_event(uint16_t Connection_Handle,
                                 uint8_t Resp_Needed,
                                 uint16_t Attribute_Handle,
                                 uint16_t Data_Length,
                                 uint8_t Data[])
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gatt_srv_write_event()[server callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
    uint8_t att_error = BLE_ATT_ERR_NONE;

    if(Attribute_Handle == RXCharHandle + 1)
    {
        Data_Received(Data_Length, Data);
        att_error = BLE_ATT_ERR_NONE;
    }
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  OTA_Write_Request_CB(Connection_Handle, Attribute_Handle, Data_Length, Data); 
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */ 
   
    if (Resp_Needed == 1U)
    {
        aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_error, 0,  NULL);
    }
}

void aci_gatt_srv_read_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Data_Offset)
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_gatt_srv_read_event()[server callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT 
    OTA_Read_Char(Connection_Handle, Attribute_Handle,Data_Offset); 
#endif 
}

#if USE_LONG_WRITE
void aci_att_srv_prepare_write_req_event(uint16_t Connection_Handle,
                                         uint16_t Attribute_Handle,
                                         uint16_t Data_Offset,
                                         uint16_t Data_Length,
                                         uint8_t Data[])
{
	#ifdef ZAO_PRINT_ADDITIONAL_INFO
	printf("====aci_att_srv_prepare_write_req_event()[server callback]====\n");
	#endif//ZAO_PRINT_ADDITONAL_INFO
    uint8_t att_err = BLE_ATT_ERR_INVALID_HANDLE;
    tBleStatus ret;
    
    if(Attribute_Handle == RXCharHandle + 1)
    {
      /**
       * Push the received prepare write in the queue.
       */
      ret = ATT_pwrq_push(Connection_Handle, Attribute_Handle,
                          Data_Offset, Data_Length, Data);
      if (ret != BLE_STATUS_SUCCESS)
      {
        att_err = BLE_ATT_ERR_PREP_QUEUE_FULL;
      }
      else
      {
        att_err = BLE_ATT_ERR_NONE;
      }
    }
    
    /**
     * Send response.
     */
    aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_err,
                      Data_Length, Data);
}

struct attribute{
  uint16_t handle;
  BOOL is_variable; /**< Indicate if the value has or not a variable length. */
  uint16_t length; /**< Current attribute length. */
  uint16_t value_buff_length;
  uint8_t  *value_buff;
};

static struct attribute * find_attribute(uint16_t attr_handle, uint8_t num_attr, struct attribute * attr)
{
  for(uint8_t i = 0; i < num_attr; i++)
  {
    if(attr[i].handle == attr_handle)
    {
      return &attr[i];
    }      
  }
  
  return NULL;  
}

static uint8_t write_queue(uint16_t conn_handle, uint8_t num_attr, struct attribute * attr)
{
  tBleStatus ret;
  uint16_t attr_h;
  ble_gatt_clt_write_ops_t wr_ops;
  struct attribute *attr_p;
  
  /**
  * Set attribute handle to 0x0000 to extract the first found entry.
  */
  attr_h = 0x0000U;
  attr_p = NULL;
  while (TRUE)
  {
    /**
    * Extract queued write.
    */
    ret = ATT_pwrq_pop(conn_handle, attr_h, &wr_ops);
    if (ret == BLE_STATUS_SUCCESS)
    {
      if (attr_h == 0x0000U)
      {
        /**
        * Select the first extracted attribute handle for the next
        * search.
        */
        attr_h = wr_ops.attr_h;
        
        /**
        * Search for attribute definition node.
        */
        
        attr_p = find_attribute(wr_ops.attr_h, num_attr, attr);
        if (attr_p == NULL)
        {
          /**
          * Invalid attribute handle.
          */
          return BLE_ATT_ERR_UNLIKELY;
        }
      }
      
      if (attr_p == NULL)
      {
        return BLE_ATT_ERR_ATTR_NOT_FOUND;
      }
      
      /**
      * Check write offset.
      */
      if (wr_ops.attr_offset > attr_p->value_buff_length)
      {
        return BLE_ATT_ERR_INVALID_OFFSET;
      }
      
      /**
      * Check write length.
      */
      if ((wr_ops.attr_offset + wr_ops.data_len) > attr_p->value_buff_length)
      {
        return BLE_ATT_ERR_INVAL_ATTR_VALUE_LEN;
      }
      
      /**
      * Write dato into attribute value buffer.
      */
      (void)Osal_MemCpy(&(attr_p->value_buff[wr_ops.attr_offset]),
                        wr_ops.data_p,
                        wr_ops.data_len);
      
      if (attr_p->is_variable == TRUE)
      {
        /**
        * Update attribute value length.
        */
        attr_p->length =  wr_ops.attr_offset + wr_ops.data_len;
      }
    }
    else
    {
      if (attr_h != 0x0000U)
      {
        /**
        * Reset attribute handle to extract the next first found entry.
        */
        attr_h = 0x0000U;
      }
      else
      {
        /**
        * No more entries are present for the selected connection handle.
        */
        break;
      }
    }
  }
  
  return BLE_ATT_ERR_NONE;
}
 //============================================================================== 
void aci_att_srv_exec_write_req_event(uint16_t Connection_Handle,
                                      uint8_t Flags)
{
  uint8_t att_error = BLE_ATT_ERR_NONE;
  
  struct attribute attr = {
    .handle = RXCharHandle + 1,
    .is_variable = TRUE,
    .length = 0,
    .value_buff_length = RX_CHR_BUFFER_SIZE,
    .value_buff = rx_char_val_buff,
  };
  
  if(Flags == 1)
  {
    att_error = write_queue(Connection_Handle, 1, &attr);
    Data_Received(attr.length, attr.value_buff);
  }
  
  /**
  * Flush the queue from all prepared write received by this connection handle.
  */
  ATT_pwrq_flush(Connection_Handle);
  
  /**
  * Return to the stack to send the Execute Write Response or in case the
  * error response if the att_error is not zero.
  */
  aci_gatt_srv_resp(Connection_Handle, 0U, att_error, 0U, NULL);
	
}
//==============================================================================	
void aci_gatt_srv_confirmation_event(uint16_t Connection_Handle)
{
	LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_HIGH);
	if(enableDataSendFlag == 1)
		{
			packetSendDoneFlag = 1;
		}
	LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_LOW);
#ifdef PRINTF_DISABLE
	printf("====aci_gatt_srv_confirmation_event() callback");
	LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_LOW);
#endif //PRINTF_DISABLE
}
//==============================================================================	
void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
	printf("====Functon begin: hci_le_data_length_change_event()\r\n");
	printf("Connection handle: 0x%04x\r\n", Connection_Handle);
	printf("MaxTxOctets: %05d\r\n", MaxTxOctets);
	printf("MaxTxTime: %05d\r\n", MaxTxTime);
	printf("MaxRxOctets: %05d\r\n", MaxRxOctets);
	printf("MaxRxTime: %05d\r\n", MaxRxTime);	
	printf("====Functon end: hci_le_data_length_change_event()\r\n");
}
//==============================================================================
void hci_le_phy_update_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t TX_PHY,
                                      uint8_t RX_PHY)
{                                    
  printf("phy_update_complete_event: Status = 0x%02X, Connection= 0x%02X, TX_PHY = 0x%0X , RX_PHY = 0x%0X  \n",
				 Status, Connection_Handle,TX_PHY,RX_PHY);                                     
}
//==============================================================================



//==============================================================================
void PeriodicNotificationSend(uint8_t sendEnable)
{
	static uint8_t dataIsSent = 0;
	static uint32_t time;
	if(sendEnable == 1)
	{
		if(dataIsSent == 0)
		{
			time = Clock_Time();
			PacketHandlerSend(txBuffer, TX_BUFFER_SIZE, 1000);
			dataIsSent = 1;
		}
		else if(Clock_Time() >= time + PERIODIC_NOTIFICATION_TIMER)
		{
			dataIsSent = 0;
		}
	}
}

//==============================================================================
#endif /* USE_LONG_WRITE */
#endif /* SERVER */

void TxSendDone(void)
{
	printf("All data has been sent\r\n");
	
}
//==============================================================================
void PeriodicNotificationSend(uint8_t sendEnable)
{
	static uint8_t dataIsSent = 0;
	static uint32_t time;
	if(sendEnable == 1)
	{
		if(dataIsSent == 0)
		{
			time = Clock_Time();
			PacketHandlerSend(txBuffer, TX_BUFFER_SIZE, 1000);
			dataIsSent = 1;
		}
		else if(Clock_Time() >= time + PERIODIC_NOTIFICATION_TIMER)
		{
			dataIsSent = 0;
		}
	}
}
//==============================================================================
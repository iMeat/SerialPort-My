
#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_
  
/*
UUIDs:
D973F2E0-B19E-11E2-9E96-0800200C9A66
D973F2E1-B19E-11E2-9E96-0800200C9A66
D973F2E2-B19E-11E2-9E96-0800200C9A66
*/
//global defines
#define SRVC_UUID           0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9
#define TX_CHR_UUID         0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9
#define RX_CHR_UUID         0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9
#if USE_LONG_WRITE
#define RX_CHR_BUFFER_SIZE  (1020)//(512)
#else
#define RX_CHR_BUFFER_SIZE  (20)
#endif
#define PERIODIC_SEND_ENABLE
#define TX_BUFFER_SIZE 32000

//global variables
extern uint8_t txBuffer[TX_BUFFER_SIZE];
extern uint8_t periodicSendDataEnable;

uint8_t Serial_port_DeviceInit(void);
void APP_Tick(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
void PeriodicNotificationSend(uint8_t SendEnable);

#endif // _SERIAL_PORT_H_

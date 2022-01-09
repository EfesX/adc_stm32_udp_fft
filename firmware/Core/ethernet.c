#include "main.h"
#include "stdlib.h"


uint8_t my_ip[4]  = {192, 168, 1, 71};
uint8_t mymask[4] = {255, 255, 255, 0};
uint8_t mygate[4] = {192, 168, 1, 1};
uint8_t mymac[6]  = {0x40, 0xa3, 0x6b, 0xc4, 0x90, 0x91};


void FreeRTOS_ENC28J60Init(void){
	//spi_dma_init();
	FreeRTOS_IPInit(my_ip, mymask, mygate, NULL, mymac);
}

//void vApplicationStackOverflowHook(TaskHandle_t  pxCurrentTCB, char *pcTaskName ){
//	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//}
void vApplicationIPNetworkEventHook(eIPCallbackEvent_t ev){
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);

	vRegisterCLICommands();
	vStartTelnetServer();
}

void vLoggingPrintf( const char *pcFormatString, ... ){
	//CDC_Transmit_FS(pcFormatString, strlen(pcFormatString));
}
BaseType_t xApplicationGetRandomNumber( uint32_t * pulNumber ){
	return rand();
}
uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress, uint16_t usSourcePort, uint32_t ulDestinationAddress, uint16_t usDestinationPort ){
	return 0xdeadbaba;
}

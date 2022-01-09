#include "main.h"
#include "cmsis_os.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOSIPConfig.h"

/* Remove the whole file if FreeRTOSIPConfig.h is set to exclude TCP. */
#if( ipconfigUSE_TCP == 1 )


static void prvTelnetServer( void *pvParameters );

osThreadId_t StartTelnetServerHandle;

const osThreadAttr_t TelnetServer_attributes = {
		.name = "telnet_server",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityNormal,
};


uint8_t close_flag = 0;

const char *const tHeader = "************************************************\r\n"
			"TELNET SERVER ON STM32 + ENC28J60 + FreeRTOS\r\n"
			"************************************************\r\n"
			">> \0";

static char cRxedData[ipconfigTCP_MSS] = {0};
static char cTxData[ipconfigTCP_MSS] = {0};

BaseType_t lBytesReceived;
portBASE_TYPE xMoreDataToFollow;
BaseType_t lSent = 0;

static struct freertos_sockaddr xClient, xBindAddress;
static Socket_t xListeningSocket, xConnectedSocket;
static socklen_t xSize = sizeof( xClient );
static const TickType_t xReceiveTimeOut = portMAX_DELAY;
static const BaseType_t xBacklog = 20;
char *indx = NULL;

void vStartTelnetServer(void){
	StartTelnetServerHandle = osThreadNew(prvTelnetServer, NULL, &TelnetServer_attributes);
}

static void prvTelnetServer(void *pvParameters){
	/* Attempt to open the socket. */
	xListeningSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP );

	/* Check the socket was created. */
	configASSERT( xListeningSocket != FREERTOS_INVALID_SOCKET );

	FreeRTOS_setsockopt( xListeningSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof( xReceiveTimeOut ) );

	xBindAddress.sin_port = ( uint16_t ) 23;
	xBindAddress.sin_port = FreeRTOS_htons( xBindAddress.sin_port );

	/* Bind the socket to the port that the client RTOS task will send to. */
	FreeRTOS_bind( xListeningSocket, &xBindAddress, sizeof( xBindAddress ) );
	FreeRTOS_listen( xListeningSocket, xBacklog );

	while(1){
		xConnectedSocket = FreeRTOS_accept( xListeningSocket, &xClient, &xSize );
		if(xConnectedSocket != FREERTOS_INVALID_SOCKET){
			while (lSent != strlen(tHeader)) lSent = FreeRTOS_send(xConnectedSocket, tHeader, strlen(tHeader), FREERTOS_MSG_DONTWAIT);
			while(1){
				lBytesReceived = FreeRTOS_recv( xConnectedSocket, cRxedData, ipconfigTCP_MSS, 0 );
				if( lBytesReceived > 0 ){

					indx = strstr(cRxedData, "\r\n");

					if(indx != NULL){
						if(indx == &cRxedData[0]){
							continue;
						}else if(indx == &cRxedData[lBytesReceived - 2]){
							cRxedData[lBytesReceived - 2] = 0x00;
							cRxedData[lBytesReceived - 1] = 0x00;
						}
					}else cRxedData[lBytesReceived] = 0x00;

					cTxData[0] = 0x00;
				    do{
				    	xMoreDataToFollow = FreeRTOS_CLIProcessCommand(cRxedData, cTxData + strlen(cTxData), ipconfigTCP_MSS - strlen(cTxData));
				    }while(xMoreDataToFollow != pdFALSE);

				    lSent = FreeRTOS_send(xConnectedSocket, cTxData, strlen(cTxData), 0);
				    if(lSent < 0) break;

				    lSent = FreeRTOS_send(xConnectedSocket, ">> \0", strlen(">> \0"), FREERTOS_MSG_DONTWAIT);
				    if(lSent < 0) break;
				}
				else if( lBytesReceived == 0 ){
					break;
				}
				else{
					FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );
					break;
				}
			}
			while( FreeRTOS_recv( xConnectedSocket, cRxedData, ipconfigTCP_MSS, 0 ) >= 0 ){
				for(uint8_t i = 0; i < 100; i++){}
		    }
		    FreeRTOS_closesocket( xConnectedSocket );

		}
	}
}

#endif

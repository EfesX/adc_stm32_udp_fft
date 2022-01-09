/*
 * FreeRTOS+TCP V2.3.2
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* FreeRTOS includes. */
#include "main.h"
#include "cmsis_os.h"

#include "FreeRTOS.h" 
//#include "task.h"
//#include "list.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
//#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
//#include "FreeRTOS_DNS.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
//#include "FreeRTOSIPConfig.h"
//#include "FreeRTOSIPConfigDefaults.h"

#include "enc.h"

/* If ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES is set to 1, then the Ethernet
 * driver will filter incoming packets and only pass the stack those packets it
 * considers need processing. */
#if ( ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES == 0 )
    #define ipCONSIDER_FRAME_FOR_PROCESSING( pucEthernetBuffer )    eProcessBuffer
#else
    #define ipCONSIDER_FRAME_FOR_PROCESSING( pucEthernetBuffer )    eConsiderFrameForProcessing( ( pucEthernetBuffer ) )
#endif

#define BUFFER_SIZE ( ipTOTAL_ETHERNET_FRAME_SIZE + ipBUFFER_PADDING )
#define BUFFER_SIZE_ROUNDED_UP ( ( BUFFER_SIZE + 7 ) & ~0x07UL )
static uint8_t ucBuffers[ ipconfigNUM_NETWORK_BUFFERS ][ BUFFER_SIZE_ROUNDED_UP ];



volatile uint8_t rx_buffer[1500] __attribute__ ((aligned(4)));

size_t xBytesReceived;

osThreadId_t processDataPacketHandle;
const osThreadAttr_t processDataPacket_attributes = {
		.name = "eth_phy",
		.stack_size = 1024 * 2,
		.priority = (osPriority_t) osPriorityNormal - 2,
};

BaseType_t xHigherPriorityTaskWoken = pdFALSE;

QueueHandle_t xEncQueue;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  //if (GPIO_Pin == ENC_INT_Pin){
	 //vTaskNotifyGiveFromISR(processDataPacketHandle, (BaseType_t *) pdFALSE);
	// xQueueSendToBackFromISR(xEncQueue, 1, (BaseType_t *) pdFALSE);
  //}
}


void process_data_packet(void *p){

	NetworkBufferDescriptor_t *pxBufferDescriptor;
	IPStackEvent_t xRxEvent;

	while(1){
		//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		xBytesReceived = eth_rx_packet(ipconfigTCP_MSS, rx_buffer);

		 if(xBytesReceived > 0){
			 pxBufferDescriptor = pxGetNetworkBufferWithDescriptor( xBytesReceived, 0 );
			 if( pxBufferDescriptor != NULL ){
				 memcpy(pxBufferDescriptor->pucEthernetBuffer, rx_buffer, xBytesReceived);
				 pxBufferDescriptor->xDataLength = xBytesReceived;

				 if(eConsiderFrameForProcessing(pxBufferDescriptor->pucEthernetBuffer) == eProcessBuffer){
					 xRxEvent.eEventType = eNetworkRxEvent;
					 xRxEvent.pvData = ( void * ) pxBufferDescriptor;
					 if( xSendEventStructToIPTask( &xRxEvent, 0 ) == pdFALSE ){
						 vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );
						 iptraceETHERNET_RX_EVENT_LOST();
					 }else iptraceNETWORK_INTERFACE_RECEIVE();
				 }else vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );
			 }else iptraceETHERNET_RX_EVENT_LOST();
		 }else{
			 vTaskDelay(pdMS_TO_TICKS(2));
		 }
	}
}

BaseType_t xNetworkInterfaceInitialise(void){
	eth_init();
	xEncQueue = xQueueCreate(32, sizeof(uint8_t));
	processDataPacketHandle = osThreadNew(process_data_packet, NULL, &processDataPacket_attributes);
    return pdTRUE;
}

void vNetworkInterfaceAllocateRAMToBuffers( NetworkBufferDescriptor_t pxNetworkBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ] ){
    /* FIX ME. */
	BaseType_t x;

	for( x = 0; x < ipconfigNUM_NETWORK_BUFFERS; x++ ){
		/* pucEthernetBuffer is set to point ipBUFFER_PADDING bytes in from the
	    beginning of the allocated buffer. */
	    pxNetworkBuffers[ x ].pucEthernetBuffer = &( ucBuffers[ x ][ ipBUFFER_PADDING ] );
	    /* The following line is also required, but will not be required in
	    future versions. */
	    *( ( uint32_t * ) &ucBuffers[ x ][ 0 ] ) = ( uint32_t ) &( pxNetworkBuffers[ x ] );
	}
}

BaseType_t xGetPhyLinkStatus( void ){
    /* FIX ME. */
    return pdTRUE;
}

void vNetworkNotifyIFDown(){

}

void vNetworkNotifyIFUp(void){

}

BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxNetworkBuffer, BaseType_t xReleaseAfterSend ){
	enc_tx_packet(pxNetworkBuffer->xDataLength, pxNetworkBuffer->pucEthernetBuffer);
	iptraceNETWORK_INTERFACE_TRANSMIT();

	if( xReleaseAfterSend != pdFALSE ){
		vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
	}
    return pdTRUE;
}

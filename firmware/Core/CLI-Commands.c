/*
 * CLI-Commands.c
 *
 *  Created on: Sep 11, 2021
 *      Author: root
 */

#include "stdio.h"


/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"



extern uint8_t close_flag;

static BaseType_t prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvTaskTimeStatCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvTaskNetbufCount( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvCloseConnect( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static BaseType_t prvMemStat( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/* Structure that defines the "task-stats" command line command. */
static const CLI_Command_Definition_t xTaskStats =
{
	"tasks", /* The command string to type. */
	"tasks:\r\n Displays a table showing the state of each FreeRTOS task\r\n",
	prvTaskStatsCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t xTaskTimeStats =
{
	"runtimes", /* The command string to type. */
	"runtimes:\r\n Displays time statistic for each task\r\n",
	prvTaskTimeStatCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
static const CLI_Command_Definition_t xNetBuf =
{
	"netbuf", /* The command string to type. */
	"netbuf:\r\n Displays count of network buffers\r\n",
	prvTaskNetbufCount, /* The function to run. */
	0 /* No parameters are expected. */
};
static const CLI_Command_Definition_t xMemStat =
{
	"memstat", /* The command string to type. */
	"memstat:\r\n Displays free memory\r\n",
	prvMemStat, /* The function to run. */
	0 /* No parameters are expected. */
};

static BaseType_t prvTaskNetbufCount( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *const pcHeader = "Network Buffers #\r\n************************************************\r\n";
	strcpy(pcWriteBuffer, pcHeader);
	//xBoundTCPSocketsList

	sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "NumberOfFreeNetworkBuffers: %d\r\n", (int) uxGetNumberOfFreeNetworkBuffers());
	sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "MinimumFreeNetworkBuffers:  %d\r\n", (int) uxGetMinimumFreeNetworkBuffers());

	return pdFALSE;
}

static BaseType_t prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *const pcHeader = "Task State  Priority  Stack #\r\n************************************************\r\n";
	strcpy(pcWriteBuffer, pcHeader);

	vTaskList(pcWriteBuffer + strlen(pcWriteBuffer));

	return pdFALSE;
}

static BaseType_t prvTaskTimeStatCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ){
	const char *const pcHeader = "Tasks Time Statistic:\r\n************************************************\r\n";
	strcpy( pcWriteBuffer, pcHeader );
	vTaskGetRunTimeStats(pcWriteBuffer + strlen(pcWriteBuffer));
	return pdFALSE;
}

static BaseType_t prvMemStat( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ){
	const char *const pcHeader = "Mem Statistics: #\r\n************************************************\r\n";
	strcpy(pcWriteBuffer, pcHeader);

	sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "FreeHeap: %d\r\n", 		(uint16_t) xPortGetFreeHeapSize());
	sprintf(pcWriteBuffer + strlen(pcWriteBuffer), "MinFreeHeap:  %d\r\n", 	(uint16_t) xPortGetMinimumEverFreeHeapSize());

	return pdFALSE;
}

void vRegisterCLICommands( void ){
	/* Register all the command line commands defined immediately above. */
	FreeRTOS_CLIRegisterCommand( &xTaskStats );
	FreeRTOS_CLIRegisterCommand( &xTaskTimeStats );
	FreeRTOS_CLIRegisterCommand( &xNetBuf );
	FreeRTOS_CLIRegisterCommand( &xMemStat );
}



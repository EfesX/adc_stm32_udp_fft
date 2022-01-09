

#include "main.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#define ARM_MATH_CM4 1U
#include "arm_math.h"

#include "task.h"

#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOSIPConfig.h"

extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;
extern TIM_HandleTypeDef htim3;


#define FFT_SIZE 512
uint16_t adc_data[FFT_SIZE << 1];
//uint16_t *adc_data_1;
//uint16_t *adc_data_2;

//q15_t adc_data_1_fft[FFT_SIZE << 1] = {0};
//q15_t adc_data_2_fft[FFT_SIZE << 1] = {0};

arm_rfft_fast_instance_f32 S;

float32_t fft_in [FFT_SIZE << 1] = {0};
float32_t fft_out[FFT_SIZE << 1] = {0};
float32_t fft_mag[FFT_SIZE >> 1] = {0};


osThreadId_t fftTaskHandle_1, fftTaskHandle_2, fftSendTaskHandle;

const osThreadAttr_t fftTask_attributes_1 = {
  .name = "fft_task_1",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityHigh,
};
const osThreadAttr_t fftTask_attributes_2 = {
  .name = "fft_task_2",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityHigh,
};
const osThreadAttr_t fftSendTask_attributes = {
  .name = "fft_send_task",
  .stack_size = 512,
  .priority = (osPriority_t) 20,
};


void ADCFFTTask_1(void *argument);
void ADCFFTTask_2(void *argument);
void FFTSendTask(void *argument);


void adc_dma_init(void){
	hdma_adc3.Instance = DMA2_Channel5;
	hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc3.Init.Mode = DMA_CIRCULAR;
	hdma_adc3.Init.Priority = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&hdma_adc3);
}

void adc_fft_start(void){
	//adc_data_1 = &adc_data[0];
	//adc_data_2 = &adc_data[FFT_SIZE];


	fftTaskHandle_1 = osThreadNew(ADCFFTTask_1, NULL, &fftTask_attributes_1);
	//fftTaskHandle_2 = osThreadNew(ADCFFTTask_2, NULL, &fftTask_attributes_2);
	fftSendTaskHandle = osThreadNew(FFTSendTask, NULL, &fftSendTask_attributes);

	adc_dma_init();

	HAL_ADCEx_Calibration_Start(&hadc3, 0);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc_data, FFT_SIZE << 1);
}

float32_t mean = 0;

void ADCFFTTask_1(void *argument){

	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		for(uint16_t i = 0; i < 512; i++){
			fft_in[i] = (float32_t) adc_data[i] - 1928;
		}

		//arm_mean_f32(fft_in, 512, &mean);
		arm_rfft_fast_init_f32(&S, FFT_SIZE);
		arm_rfft_fast_f32(&S, fft_in, fft_out, 0);
		arm_cmplx_mag_f32(fft_out, fft_mag, FFT_SIZE);

		xTaskNotifyGive(fftSendTaskHandle);

	}
}




void ADCFFTTask_2(void *argument){

	while(1){

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

		for(uint16_t i = 0; i < FFT_SIZE; i++){
			fft_in[FFT_SIZE + i] = (float32_t) adc_data[i] - 1953;
		}

		//arm_mean_f32(fft_in, 512, &mean);
		arm_rfft_fast_init_f32(&S, FFT_SIZE);
		arm_rfft_fast_f32(&S, &fft_in[FFT_SIZE], &fft_out[FFT_SIZE], 0);
		arm_cmplx_mag_f32(&fft_out[FFT_SIZE], &fft_mag[FFT_SIZE], FFT_SIZE);

	}
}

void FFTSendTask(void *argument){

	Socket_t xSocket;
	struct freertos_sockaddr xDestinationAddress;
	const TickType_t x10ms = 10UL / portTICK_PERIOD_MS;

	xDestinationAddress.sin_addr = FreeRTOS_inet_addr( "192.168.1.166" );
	xDestinationAddress.sin_port = FreeRTOS_htons( 10000 );

	xSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP );
	configASSERT( xSocket != FREERTOS_INVALID_SOCKET );

	while(1){
		vTaskDelay(pdMS_TO_TICKS(300));
		//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//FreeRTOS_sendto( xSocket, (char *) fft_in, 1024, 0, &xDestinationAddress, sizeof( xDestinationAddress ) );
		FreeRTOS_sendto( xSocket, (char *) fft_mag, 1024, 0, &xDestinationAddress, sizeof( xDestinationAddress ) );
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(hadc->Instance == ADC3){
		vTaskNotifyGiveFromISR(fftTaskHandle_1, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(hadc->Instance == ADC3){
		//vTaskNotifyGiveFromISR(fftTaskHandle_2, &xHigherPriorityTaskWoken);
		//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}
}



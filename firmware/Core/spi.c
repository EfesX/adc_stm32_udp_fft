#include "main.h"
#include "spi.h"

//void NSS_DOWN(void){
//	HAL_GPIO_WritePin(ENC_NSS_GPIO_Port, ENC_NSS_Pin, GPIO_PIN_RESET);
//}

//void NSS_UP(void){
//	HAL_GPIO_WritePin(ENC_NSS_GPIO_Port, ENC_NSS_Pin, GPIO_PIN_SET);
//}


void spi_dma_init(void){
	NSS_UP();

	hdma_spi1_tx.Instance = DMA1_Channel3;
	hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_spi1_tx.Init.Mode = DMA_NORMAL;
	hdma_spi1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	HAL_DMA_Init(&hdma_spi1_tx);
}



//HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi){
//
//}

HAL_StatusTypeDef spi_transmit(uint8_t *pdata, uint16_t len, uint8_t nss_release){
	HAL_StatusTypeDef res = HAL_ERROR;

	NSS_DOWN();
	res = HAL_SPI_Transmit(&hspi1, pdata, len, 1000);
	if(nss_release)	NSS_UP();
	return res;
}

HAL_StatusTypeDef spi_receive(uint8_t *pdata, uint16_t len, uint8_t nss_release){
	HAL_StatusTypeDef res = HAL_ERROR;

	NSS_DOWN();
	res = HAL_SPI_Receive(&hspi1, pdata, len, 1000);
	if(nss_release)	NSS_UP();
	return res;

}

HAL_StatusTypeDef spi_transmit_dma(uint8_t *pdata, uint16_t len){
	HAL_StatusTypeDef res = HAL_ERROR;

	NSS_DOWN();
	res = HAL_SPI_Transmit_DMA(&hspi1, pdata, len);
	return res;
}





HAL_StatusTypeDef spi_txrx(uint8_t *ptxdata, uint8_t *prxdata, uint16_t len, uint8_t nss_release){
	HAL_StatusTypeDef res = HAL_ERROR;
	NSS_DOWN();
	res =  HAL_SPI_TransmitReceive(&hspi1, ptxdata, prxdata, len, 1000);
	if(nss_release)	NSS_UP();
	return res;
}




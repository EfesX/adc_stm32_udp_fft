#ifndef __SPI_H
#define __SPI_H

#define NSS_DOWN() HAL_GPIO_WritePin(ENC_NSS_GPIO_Port, ENC_NSS_Pin, GPIO_PIN_RESET);
#define NSS_UP()   HAL_GPIO_WritePin(ENC_NSS_GPIO_Port, ENC_NSS_Pin, GPIO_PIN_SET);


extern SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;


void spi_dma_init(void);
HAL_StatusTypeDef spi_transmit(uint8_t *pdata, uint16_t len, uint8_t nss_release);
HAL_StatusTypeDef spi_receive(uint8_t *pdata, uint16_t len, uint8_t nss_release);
HAL_StatusTypeDef spi_transmit_dma(uint8_t *pdata, uint16_t len);
HAL_StatusTypeDef spi_txrx(uint8_t *ptxdata, uint8_t *prxdata, uint16_t len, uint8_t nss_release);




#endif /* __SPI_H */

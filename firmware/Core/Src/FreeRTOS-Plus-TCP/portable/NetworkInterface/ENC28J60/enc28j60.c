#include "enc.h"

__uint8_t current_bank = 0;
extern unsigned char mymac[6];
static uint16_t NextPacketPtr;

void ENC_RESET_ENABLE(void){
	HAL_GPIO_WritePin(ENC_RESET_GPIO_Port, ENC_RESET_Pin, GPIO_PIN_RESET);
}

void ENC_RESET_DISABLE(void){
	HAL_GPIO_WritePin(ENC_RESET_GPIO_Port, ENC_RESET_Pin, GPIO_PIN_SET);
}

void enc_delay(int x){
	for(uint16_t i = 0; i < x; i++){}
}

void eth_set_bit(uint8_t addr, uint8_t mask){
	uint8_t w[2] = {ENC28J60_BIT_FIELD_SET | addr, mask };
	spi_transmit((uint8_t *) &w, 2, 1);
}
void eth_clear_bit(uint8_t addr, uint8_t mask){
	uint8_t w[2] = { ENC28J60_BIT_FIELD_CLR | addr, mask };
	spi_transmit((uint8_t *) &w, 2, 1);
}
void eth_set_bank(uint8_t addr){
	if (current_bank != addr){
		eth_clear_bit(ECON1, 0x03);
		eth_set_bit(ECON1, (uint8_t)((addr >> 5) & 0x03));
		current_bank = addr;
	}
}

uint8_t eth_read_ctrl_8(uint8_t addr){
	uint8_t address = addr;
	eth_set_bank(address);
	uint8_t r[3];
	uint8_t w[3] = { ENC28J60_READ_CTRL_REG | (address & 0x1f), 0, 0 };
	if(address & 0x80){
		spi_txrx(w, r, 3, 1);
		return r[2];
	} else {
		spi_txrx(w, r, 2, 1);
		return r[1];
	}
}
uint16_t eth_read_ctrl_16(uint8_t addr){
	eth_set_bank(addr);
	uint16_t rl = (uint16_t)eth_read_ctrl_8(addr);
	uint16_t rh = (uint16_t)eth_read_ctrl_8(addr + 1);
	uint16_t r = (rh << 8) | rl;
	return r;
}
void eth_write_ctrl_8(uint8_t addr, uint8_t data){
	eth_set_bank(addr);
	uint8_t w[2] = {ENC28J60_WRITE_CTRL_REG | (addr & 0x1f), data};
	spi_transmit(w, 2, 1);
}
void eth_write_ctrl_16(uint8_t addr, uint16_t data){
	eth_set_bank(addr);
	uint8_t w[3] = {ENC28J60_WRITE_CTRL_REG | (addr & 0x1f), (uint8_t)(data & 0x00ff), (uint8_t)((data & 0xff00) >> 8)};
	spi_transmit(w, 3, 1);
}
void eth_write_phy(uint8_t addr, uint16_t data){
	eth_write_ctrl_8(MIREGADR, addr);
	eth_write_ctrl_16(MIWRL, data);
	while(eth_read_ctrl_8(MISTAT) & MISTAT_BUSY);
}
uint16_t eth_read_phy(uint8_t addr){
	eth_write_ctrl_8(MIREGADR, addr);
	eth_write_ctrl_8(MICMD, MICMD_MIIRD);
	while(eth_read_ctrl_8(MISTAT) & MISTAT_BUSY);
	eth_write_ctrl_8(MICMD, 0);
	return eth_read_ctrl_16(MIRDL);
}
void eth_soft_reset(void){
	uint8_t w = ENC28J60_SOFT_RESET;
	spi_transmit(&w, 1, 1);
}
uint16_t eth_get_pktcnt(void){
	return eth_read_ctrl_8(EPKTCNT);
}


//******************************************************************************
uint16_t eth_read_buf_16(void){
	uint8_t t[3] = {0};
	uint8_t w = ENC28J60_READ_BUF_MEM;
	spi_transmit(&w, 1, 0);
	spi_receive(t, 2, 1);

	uint16_t data = 0;
	data  = t[0];
	data |= t[1] << 8;
	return data;
}
uint8_t eth_read_buf_8(void){
	uint8_t w = ENC28J60_READ_BUF_MEM;
	uint8_t r[2] = {0};
	spi_transmit(w, 1, 0);
	spi_receive(r, 1, 1);
	return r[0];
}
void eth_read_buf(uint16_t len, uint8_t *data){
	uint8_t w = ENC28J60_READ_BUF_MEM;
	spi_transmit(&w, 1, 0);
	spi_receive(data, len, 1);
}
uint16_t enc_get_buf_len(){
	eth_read_buf_8();
	eth_read_buf_8();

	uint8_t len = eth_read_buf_8();
	len |= eth_read_buf_8() << 8;
	return len;
}
//********************************************************************************
uint16_t eth_rx_packet(uint16_t maxlen, uint8_t *packet){
   unsigned int rxstat;
   unsigned int len;

   if (eth_read_ctrl_8(EPKTCNT) == 0) return(0);

   // Set the read pointer to the start of the received packet
   eth_write_ctrl_16(ERDPTL, NextPacketPtr);

   // read the next packet pointer
	NextPacketPtr = eth_read_buf_16();

   // read the packet length (see datasheet page 43)
	len = eth_read_buf_16();

   len -= 4; //remove the CRC count
   // read the receive status (see datasheet page 43)
   rxstat = eth_read_buf_16();

   // limit retrieve length
   if (len > maxlen - 1) len = maxlen - 1;

   // check CRC and symbol errors (see datasheet page 44, table 7-3):
   // The ERXFCON.CRCEN is set by default. Normally we should not
   // need to check this.
   if ((rxstat & 0x80) == 0){
      // invalid
      len = 0;
   } else {
      // copy the packet from the receive buffer
	   eth_read_buf(len, packet);
   }
   // Move the RX read pointer to the start of the next received packet
   // This frees the memory we just read out
   eth_write_ctrl_16(ERXRDPTL, (NextPacketPtr));

   // decrement the packet counter indicate we are done with this packet
   uint8_t econ2 = eth_read_ctrl_8(ECON2);
   econ2 |= ECON2_PKTDEC;
   eth_write_ctrl_8(ECON2, econ2);
   return(len);
}

extern uint8_t dma_flag;

void enc_tx_packet(uint16_t len, uint8_t *packet){

	while (eth_read_ctrl_8(ECON1) & ECON1_TXRTS);

	// Reset the transmit logic problem. Unblock stall in the transmit logic.
	// See Rev. B4 Silicon Errata point 12.
	if( (eth_read_ctrl_8(EIR) & EIR_TXERIF) ) {
	  	eth_write_ctrl_8(ECON1, 0x84); //rst set
	   	eth_write_ctrl_8(ECON1, 0x04); //rst clear
	   	eth_write_ctrl_8(EIR,   0x00);
    }
	// Set the write pointer to start of transmit buffer area
	eth_write_ctrl_16(EWRPTL, TXSTART_INIT);

	// Set the TXND pointer to correspond to the packet size given
	eth_write_ctrl_8(ETXNDL, (TXSTART_INIT + len));

	// write per-packet control byte (0x00 means use macon3 settings)
	uint8_t w[2];
	w[0] = ENC28J60_WRITE_BUF_MEM;
	w[1] = 0x00;
	spi_transmit(w, 2, 1);
	//alt_avalon_spi_command(SPI_MASTER_BASE, 0, 2, w, 0, NULL, 0);
	// copy the packet into the transmit buffer
	w[0] = ENC28J60_WRITE_BUF_MEM;
	spi_transmit(&w[0], 1, 0);
	spi_transmit(packet, len, 1);
	//alt_avalon_spi_command(SPI_MASTER_BASE, 0, len + 1, packet - 1, 0, NULL, 0);

	// send the contents of the transmit buffer onto the network
	eth_write_ctrl_8(ECON1, 0x0c); //start set
}

void eth_init(void){
	ENC_RESET_ENABLE();
	ENC_RESET_DISABLE();
	HAL_Delay(10);
	eth_soft_reset();
	HAL_Delay(10);

	eth_clear_bit(ECON1, 0x03);

	NextPacketPtr = RXSTART_INIT;

	eth_write_ctrl_16(ERXSTL,   RXSTART_INIT);
	eth_write_ctrl_16(ERXRDPTL, RXSTART_INIT);
	eth_write_ctrl_16(ERXNDL,   RXSTOP_INIT);
	eth_write_ctrl_16(ETXSTL,   TXSTART_INIT);
	eth_write_ctrl_16(ETXNDL,   TXSTOP_INIT);

	eth_write_ctrl_8(ERXFCON,   ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_PMEN |ERXFCON_BCEN);
	eth_write_ctrl_8(EPMM0,  0x3f);
	eth_write_ctrl_8(EPMM1,  0x30);
	eth_write_ctrl_8(EPMCSL, 0xf9);
	eth_write_ctrl_8(EPMCSH, 0xf7);
	eth_write_ctrl_8(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
	eth_write_ctrl_8(MACON2,  0x00);
	uint8_t macon3 = eth_read_ctrl_8(MACON3);
	macon3 |= MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN | MACON3_FULDPX;
	eth_write_ctrl_8(MACON3, macon3);
	eth_write_ctrl_8(MAIPGL,  0x12);
	eth_write_ctrl_8(MAIPGH,  0x0C);
	eth_write_ctrl_8(MABBIPG, 0x12);
	eth_write_ctrl_16(MAMXFLL, MAX_FRAMELEN);
	eth_write_ctrl_8(MAADR5, mymac[0]);
	eth_write_ctrl_8(MAADR4, mymac[1]);
	eth_write_ctrl_8(MAADR3, mymac[2]);
	eth_write_ctrl_8(MAADR2, mymac[3]);
	eth_write_ctrl_8(MAADR1, mymac[4]);
	eth_write_ctrl_8(MAADR0, mymac[5]);

	eth_write_phy(PHCON1, PHCON1_PDPXMD);
	eth_write_phy(PHCON2, PHCON2_HDLDIS);

	eth_write_ctrl_8(EIE, EIE_INTIE | EIE_PKTIE);

	eth_write_ctrl_8(ECON1, ECON1_RXEN);
}

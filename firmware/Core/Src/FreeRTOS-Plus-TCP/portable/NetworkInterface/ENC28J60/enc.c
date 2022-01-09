/*
 * enc.c
 *
 *  Created on: Aug 30, 2021
 *      Author: root
 */

#include "enc.h"


__uint8_t current_bank = 0;


void enc_delay(int x){
	for(int i = 0; i < x; i++){

	}
}


extern unsigned char mymac[6];

static alt_u16 NextPacketPtr;

void eth_set_bit(alt_u8 addr, alt_u8 mask){
	alt_u8 w[2];
	w[0] = ENC28J60_BIT_FIELD_SET | addr;
	w[1] = mask;
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, 2, w, 0, NULL, 0);
}
void eth_clear_bit(alt_u8 addr, alt_u8 mask){
	alt_u8 w[2];
	w[0] = ENC28J60_BIT_FIELD_CLR | addr;
	w[1] = mask;
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, 2, w, 0, NULL, 0);
}
void eth_set_bank(alt_u8 addr){
	if (current_bank != addr){
		eth_clear_bit(ECON1, 0x03);
		eth_set_bit(ECON1, (alt_u8)((addr >> 5) & 0x03));
		current_bank = addr;
	}
}

alt_u8 eth_read_ctrl_8(alt_u8 addr){
	alt_u8 address = addr;
	eth_set_bank(address);
	//enc_delay(100);
	alt_u8 r[2];
	alt_u8 w = ENC28J60_READ_CTRL_REG | (address & 0x1f);
	if(address & 0x80){
		alt_avalon_spi_command(SPI_MASTER_BASE, 0, 1, &w, 2, &r, 0);
		return r[1];
	} else {
		alt_avalon_spi_command(SPI_MASTER_BASE, 0, 1, &w, 1, &r, 0);
		return r[0];
	}
	return r[0];
}
alt_u16 eth_read_ctrl_16(alt_u8 addr){
	eth_set_bank(addr);
	alt_u16 rl = (alt_u16)eth_read_ctrl_8(addr);
	alt_u16 rh = (alt_u16)eth_read_ctrl_8(addr + 1);
	alt_u16 r = (rh << 8) | rl;
	return r;
}
void eth_write_ctrl_8(alt_u8 addr, alt_u8 data){
	eth_set_bank(addr);
	alt_u8 w[2];
	w[0] = ENC28J60_WRITE_CTRL_REG | (addr & 0x1f);
	w[1] = data;
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, 2, w, 0, NULL, 0);
}
void eth_write_ctrl_16(alt_u8 addr, alt_u16 data){
	eth_set_bank(addr);
	alt_u8 w[3];
	w[0] = ENC28J60_WRITE_CTRL_REG | (addr & 0x1f);
	w[1] = (alt_u8)(data & 0x00ff);
	w[2] = (alt_u8)((data & 0xff00) >> 8);
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, 3, w, 0, NULL, 0);
}
void eth_write_phy(alt_u8 addr, alt_u16 data){
	eth_write_ctrl_8(MIREGADR, addr);
	eth_write_ctrl_16(MIWRL, data);
	while(eth_read_ctrl_8(MISTAT) & MISTAT_BUSY);
}
alt_u16 eth_read_phy(alt_u8 addr){
	eth_write_ctrl_8(MIREGADR, addr);
	eth_write_ctrl_8(MICMD, MICMD_MIIRD);
	while(eth_read_ctrl_8(MISTAT) & MISTAT_BUSY);
	eth_write_ctrl_8(MICMD, 0);
	return eth_read_ctrl_16(MIRDL);
}

void eth_soft_reset(void){
	alt_u8 w = ENC28J60_SOFT_RESET;
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, 1, &w, 0, NULL, 0);
}
alt_u16 eth_get_pktcnt(void){
	return eth_read_ctrl_8(EPKTCNT);
}
void eth_init(/*alt_isr_func func_isr*/void){
	eth_soft_reset();
	eth_clear_bit(ECON1, 0x03);

	NextPacketPtr = RXSTART_INIT;

	eth_write_ctrl_16(ERXSTL,   RXSTART_INIT);
	eth_write_ctrl_16(ERXRDPTL, RXSTART_INIT);
	eth_write_ctrl_16(ERXNDL,   RXSTOP_INIT);
	eth_write_ctrl_16(ETXSTL,   TXSTART_INIT);
	eth_write_ctrl_16(ETXNDL,   TXSTOP_INIT);

	eth_write_ctrl_8(ERXFCON,   ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_PMEN |ERXFCON_BCEN);
	eth_write_ctrl_8(EPMM0, 0x3f);
	eth_write_ctrl_8(EPMM1, 0x30);
	eth_write_ctrl_8(EPMCSL, 0xf9);
	eth_write_ctrl_8(EPMCSH, 0xf7);
	eth_write_ctrl_8(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
	eth_write_ctrl_8(MACON2, 0x00);
	alt_u8 macon3 = eth_read_ctrl_8(MACON3);
	macon3 |= MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN | MACON3_FULDPX;
	eth_write_ctrl_8(MACON3, macon3);
	eth_write_ctrl_8(MAIPGL, 0x12);
	eth_write_ctrl_8(MAIPGH, 0x0C);
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
	//alt_ic_isr_register(PIO_1_IRQ_INTERRUPT_CONTROLLER_ID, PIO_1_IRQ, func_isr, NULL, NULL);

	eth_write_ctrl_8(ECON1, ECON1_RXEN);
}

alt_u16 eth_read_buf_16(){
	alt_u8 t[2] = {0, 0};
	alt_u8 w = ENC28J60_READ_BUF_MEM;
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, 1, &w, 2, &t, 0);
	alt_u16 data = 0;
	data  = t[0];
	data |= t[1] << 8;

	return data;
}

alt_u8 eth_read_buf_8(void){
	alt_u8 w = ENC28J60_READ_BUF_MEM;
	alt_u8 r = 0;
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, 1, &w, 1, &r, 0);
	return r;
}
void eth_read_buf(alt_u16 len, alt_u8 *data){
	alt_u8 w = ENC28J60_READ_BUF_MEM;
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, 1, &w, len, data, 0);
}


alt_u16 enc_get_buf_len(){
	eth_read_buf_8();
	eth_read_buf_8();

	alt_u8 len = eth_read_buf_8();
	len |= eth_read_buf_8() << 8;

	return len;
}



alt_u16 eth_rx_packet(alt_u16 maxlen, unsigned char* packet){
   unsigned int rxstat;
   unsigned int len;
   alt_u8 temp_buf[2];

   // check if a packet has been received and buffered
   //if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
   // The above does not work. See Rev. B4 Silicon Errata point 6.
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
   alt_u8 econ2 = eth_read_ctrl_8(ECON2);
   econ2 |= ECON2_PKTDEC;
   eth_write_ctrl_8(ECON2, econ2);
   return(len);
}

void enc_tx_packet(alt_u16 len, alt_u8 *packet){

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
	alt_u8 w[2];
	w[0] = ENC28J60_WRITE_BUF_MEM;
	w[1] = 0x00;
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, 2, w, 0, NULL, 0);
	// copy the packet into the transmit buffer
	*(packet - 1) = ENC28J60_WRITE_BUF_MEM;
	alt_avalon_spi_command(SPI_MASTER_BASE, 0, len + 1, packet - 1, 0, NULL, 0);
	// send the contents of the transmit buffer onto the network
	eth_write_ctrl_8(ECON1, 0x0c); //start set
}

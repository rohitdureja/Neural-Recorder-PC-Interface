/*
 * nrf24l01.c
 *
 *  Created on: Jul 29, 2014
 *      Author: Rohit Dureja
 */

#include "nrf24l01.h"

// initialize the RF module
void RFInit(uint32_t ui32Mode)
{
	SPIInit();

	if(ui32Mode == 0) // RX Mode
	{
		SPISetCELow(); // disable all communication

		// ----- //
		RFWriteRegister(WRITE_REG + SETUP_AW, 0x01); // Set address width to three bytes
		// set RX pipe 0 address
		SPISetCSNLow();
		SPIDataWrite(WRITE_REG + RX_ADDR_P0);
		SPIDataRead();
		SPIDataWrite(0x2C); // LSB
		SPIDataRead();
		SPIDataWrite(0x3E);
		SPIDataRead();
		SPIDataWrite(0x3E); // MSB
		SPIDataRead();
		SPISetCSNHigh();
		RFWriteRegister(WRITE_REG + EN_AA, 0x01); // enable ACK on RX pipe 0
		RFWriteRegister(WRITE_REG + EN_RXADDR, 0x01); // enable data pipe 0
		RFWriteRegister(WRITE_REG + RF_CH, 20); // set RF channel
		RFWriteRegister(WRITE_REG + RF_SETUP, 0x0F); // set data rate at 2mbps and power at 0dBm
		RFWriteRegister(WRITE_REG + DYNPD, 0x01); // enable dynamic payload length for RX pipe 0
		RFWriteRegister(WRITE_REG + FEATURE, 0x06); // enable dynamic payload length
		RFWriteRegister(WRITE_REG + CONFIG, 0x3F); // RX_DR interrupt on IRQ pin and RX mode on

		// Flush SPI RX FIFO to remove residual data
		SPIRXFlush();
		// ----- //

		SPISetCEHigh(); // enable all communication
	}
	else if(ui32Mode == 1) // TX Mode
	{
		SPISetCELow(); // disable all communication

		// ----- //
		RFWriteRegister(WRITE_REG + SETUP_AW, 0x01); // Set address width to three bytes
		// set TX address
		SPISetCSNLow();
		SPIDataWrite(WRITE_REG + TX_ADDR);
		SPIDataRead();
		SPIDataWrite(0x2C); // LSB
		SPIDataRead();
		SPIDataWrite(0x3E);
		SPIDataRead();
		SPIDataWrite(0x3E); // MSB
		SPIDataRead();
		SPISetCSNHigh();
		// set RX pipe 0 address
		SPISetCSNLow();
		SPIDataWrite(WRITE_REG + RX_ADDR_P0);
		SPIDataRead();
		SPIDataWrite(0x2C); // LSB
		SPIDataRead();
		SPIDataWrite(0x3E);
		SPIDataRead();
		SPIDataWrite(0x3E); // MSB
		SPIDataRead();
		SPISetCSNHigh();
		RFWriteRegister(WRITE_REG + EN_AA, 0x01);
		RFWriteRegister(WRITE_REG + EN_RXADDR, 0x01);
		RFWriteRegister(WRITE_REG + SETUP_RETR, 0x12); // set retries to 5 and delay to 500us
		RFWriteRegister(WRITE_REG + RF_CH, 20); // set RF channel
		RFWriteRegister(WRITE_REG + DYNPD, 0x01);
		RFWriteRegister(WRITE_REG + FEATURE, 0x04); // enable dynamic payload length
		RFWriteRegister(WRITE_REG + RF_SETUP, 0x0F); // set data rate at 2mbps and power at 0dBm
		RFWriteRegister(WRITE_REG + CONFIG, 0x6E); // MAX_RT interrupt on IRQ and TX mode on

		// Flush SPI RX FIFO to remove residual data
		SPIRXFlush();
		// ----- //

		SPISetCEHigh(); // enable all communication
	}

	RFWriteRegister(WRITE_REG + STATUSREG, 0x40); // Clear RX_DR flag

	//Flush TX buffer
	SPISetCSNLow();
	SPIDataWrite(FLUSH_TX);
	SPIDataRead();
	SPISetCSNHigh();

	//Flush RX buffer
	SPISetCSNLow();
	SPIDataWrite(FLUSH_RX);
	SPIDataRead();
	SPISetCSNHigh();
}

// write into a register. returns status
uint32_t RFWriteRegister(uint32_t ui32Register, uint32_t ui32Value)
{
	uint32_t ui32Status;
	SPISetCSNLow();
	SPIDataWrite(ui32Register); // select register to write to
	ui32Status = SPIDataRead();
	SPIDataWrite(ui32Value); // write value in register
	SPIDataRead();
	SPISetCSNHigh();
	return ui32Status;
}

// read from a RF register. returns read value
uint32_t RFReadRegister(uint32_t ui32Register)
{
	uint32_t ui32Value;
	SPISetCSNLow();
	SPIDataWrite(ui32Register); // select register to read from
	ui32Value = SPIDataRead();
	SPIDataWrite(0xFF); // push dummy bits to extract value
	ui32Value = SPIDataRead();
	SPISetCSNHigh();
	return ui32Value;
}

// write to send buffer. Returns numbers of bytes written
uint32_t RFWriteSendBuffer(uint32_t *ui32Data, uint32_t ui32Bytes)
{
	uint32_t i;

	//Flush TX buffer
	SPISetCSNLow();
	SPIDataWrite(FLUSH_TX);
	SPIDataRead();
	SPISetCSNHigh();

	SPISetCELow(); // disable all communications
	SPISetCSNLow();
	SPIDataWrite(WR_TX_PLOAD);  // choose TX payload register
	SPIDataRead();
	for(i = 0 ; i < ui32Bytes ; ++i)
	{
		SPIDataWrite(ui32Data[i]); // push bytes into TX payload
		SPIDataRead();
	}
	SPISetCSNHigh();
	SPISetCEHigh(); // enable all communication

	// Flush SPI RX FIFO to remove residual data
	SPIRXFlush();
	return i;
}

// read from recive buffer. Returns number of bytes read
uint32_t RFReadRecieveBuffer(uint8_t *ui32Data)
{
	uint32_t ui32Bytes = 32;
	uint32_t i;
	// Find number of bytes to read
	SPISetCSNLow();
	SPIDataWrite(R_RX_PL_WID);
	ui32Bytes = SPIDataRead();
	SPIDataWrite(0xFF);
	ui32Bytes = SPIDataRead();
	SPISetCSNHigh();

	// if bytes > 32. Some error has occurred.
	if(ui32Bytes > 32)
	{
		// Flush RX FIFO
		SPISetCSNLow();
		SPIDataWrite(FLUSH_RX);
		SPIDataRead();
		SPISetCSNHigh();
		return 0;
	}
	else
	{
		SPISetCSNLow();
		SPIDataWrite(RD_RX_PLOAD);
		SPIDataRead(); // first bytes not important contains status
		for(i = 0 ; i < ui32Bytes ; ++i)
		{
			SPIDataWrite(0xFF);
			ui32Data[i] = SPIDataRead();
		}
		SPISetCSNHigh();
		return ui32Bytes;
	}
}

/*
 * rc522deneme.c
 *
 *  Created on: Sep 6, 2025
 *      Author: Enes
 */

#include "rc522deneme.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"

typedef enum {
    RC522_CMD_IDLE              = 0x00, // no action, cancels current command
    RC522_CMD_MEM               = 0x01, // stores 25 bytes into the internal buffer
    RC522_CMD_GENERATE_RANDOMID = 0x02, // generates a 10-byte random ID number
    RC522_CMD_CALC_CRC          = 0x03, // activates CRC coprocessor / self test
    RC522_CMD_TRANSMIT          = 0x04, // transmits data from FIFO buffer
    RC522_CMD_NOCMDCHANGE       = 0x07, // no command change, only update CommandReg bits
    RC522_CMD_RECEIVE           = 0x08, // activates receiver circuits
    RC522_CMD_TRANSCEIVE        = 0x0C, // transmit FIFO data and auto-activate receiver
    RC522_CMD_RESERVED          = 0x0D, // reserved for future use
    RC522_CMD_MFAUTHENT         = 0x0E, // MIFARE authentication as reader
    RC522_CMD_SOFTRESET         = 0x0F  // reset MFRC522
} rc522_command_t;

typedef enum {
    // Page 0: Command and Status
    RC522_REG_RESERVED_00      = 0x00,
    RC522_REG_COMMAND          = 0x01,
    RC522_REG_COM_IEN          = 0x02,
    RC522_REG_DIV_IEN          = 0x03,
    RC522_REG_COM_IRQ          = 0x04,
    RC522_REG_DIV_IRQ          = 0x05,
    RC522_REG_ERROR            = 0x06,
    RC522_REG_STATUS1          = 0x07,
    RC522_REG_STATUS2          = 0x08,
    RC522_REG_FIFO_DATA        = 0x09,
    RC522_REG_FIFO_LEVEL       = 0x0A,
    RC522_REG_WATER_LEVEL      = 0x0B,
    RC522_REG_CONTROL          = 0x0C,
    RC522_REG_BIT_FRAMING      = 0x0D,
    RC522_REG_COLL             = 0x0E,
    RC522_REG_RESERVED_0F      = 0x0F,

    // Page 1: Command
    RC522_REG_RESERVED_10      = 0x10,
    RC522_REG_MODE             = 0x11,
    RC522_REG_TX_MODE          = 0x12,
    RC522_REG_RX_MODE          = 0x13,
    RC522_REG_TX_CONTROL       = 0x14,
    RC522_REG_TX_ASK           = 0x15,
    RC522_REG_TX_SEL           = 0x16,
    RC522_REG_RX_SEL           = 0x17,
    RC522_REG_RX_THRESHOLD     = 0x18,
    RC522_REG_DEMOD            = 0x19,
    RC522_REG_RESERVED_1A      = 0x1A,
    RC522_REG_RESERVED_1B      = 0x1B,
    RC522_REG_MF_TX            = 0x1C,
    RC522_REG_MF_RX            = 0x1D,
    RC522_REG_RESERVED_1E      = 0x1E,
    RC522_REG_SERIAL_SPEED     = 0x1F,

    // Page 2: Configuration
    RC522_REG_RESERVED_20      = 0x20,
    RC522_REG_CRC_RESULT       = 0x21, // MSB + LSB
    RC522_REG_CRC_RESULT_L     = 0x22,
    RC522_REG_RESERVED_23      = 0x23,
    RC522_REG_MOD_WIDTH        = 0x24,
    RC522_REG_RESERVED_25      = 0x25,
    RC522_REG_RF_CFG           = 0x26,
    RC522_REG_GS_N             = 0x27,
    RC522_REG_CW_GSP           = 0x28,
    RC522_REG_MOD_GSP          = 0x29,
    RC522_REG_TMODE            = 0x2A,
    RC522_REG_TPRESCALER       = 0x2B,
    RC522_REG_TRELOAD_HI       = 0x2C, // 16-bit
    RC522_REG_TRELOAD_LO       = 0x2D,
    RC522_REG_TCOUNTER_VAL     = 0x2E, // 16-bit
    RC522_REG_TCOUNTER_VAL_L   = 0x2F,

    // Page 3: Test Register
    RC522_REG_RESERVED_30      = 0x30,
    RC522_REG_TEST_SEL1        = 0x31,
    RC522_REG_TEST_SEL2        = 0x32,
    RC522_REG_TEST_PIN_EN      = 0x33,
    RC522_REG_TEST_PIN_VALUE   = 0x34,
    RC522_REG_TEST_BUS         = 0x35,
    RC522_REG_AUTO_TEST        = 0x36,
    RC522_REG_VERSION          = 0x37,
    RC522_REG_ANALOG_TEST      = 0x38,
    RC522_REG_TEST_DAC1        = 0x39,
    RC522_REG_TEST_DAC2        = 0x3A,
    RC522_REG_TEST_ADC         = 0x3B,
    RC522_REG_RESERVED_3C      = 0x3C,
    RC522_REG_RESERVED_3D      = 0x3D,
    RC522_REG_RESERVED_3E      = 0x3E,
    RC522_REG_RESERVED_3F      = 0x3F
} rc522_register_map_t;

struct rc522_t{
	SPI_HandleTypeDef* spi_handle;
	GPIO_TypeDef* rst_port;
	GPIO_TypeDef* cs_port;
	GPIO_TypeDef* irq_port;
	uint16_t rst_pin;
	uint16_t cs_pin;
	uint16_t irq_pin;
	uint32_t max_delay;
	uint8_t wait_irq;
	uint8_t irq_en;
	uint8_t command;
};

rc522_return_status_t static write_register_poll(rc522_handle dev, uint8_t reg_addr, uint8_t* txdata, uint16_t size){
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	uint8_t register_address = (reg_addr<<1)&0x7E;
	if(HAL_SPI_Transmit(dev->spi_handle, &register_address, 1, dev->max_delay)!=HAL_OK){
		return rc522_fail;
	}
	if(HAL_SPI_Transmit(dev->spi_handle, txdata, size, dev->max_delay)!=HAL_OK){
		return rc522_fail;
	}
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return rc522_ok;
}

rc522_return_status_t static write_register_dma(rc522_handle dev, uint8_t reg_addr, uint8_t* txdata, uint16_t size){
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	uint8_t buffer[size+1];
	buffer[0]=(reg_addr<<1)&0x7E;
	for(uint8_t i=0; i<size; i++){
		buffer[i+1]=txdata[i];
	}
	if(HAL_SPI_Transmit_DMA(dev->spi_handle, buffer, size+1)!=HAL_OK){
		return rc522_fail;
	}
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return rc522_ok;
}

rc522_return_status_t static read_register_poll(rc522_handle dev, uint8_t reg_addr, uint8_t* rxdata, uint16_t size){
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	uint8_t register_address = ((reg_addr<<1)&0x7E)|0x80;
	if(HAL_SPI_Transmit(dev->spi_handle, &register_address, 1, dev->max_delay)!=HAL_OK){
		return rc522_fail;
	}
	if(HAL_SPI_Receive(dev->spi_handle, rxdata, size, dev->max_delay)!=HAL_OK){
		return rc522_fail;
	}
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return rc522_ok;
}

rc522_return_status_t static read_register_dma(rc522_handle dev, uint8_t reg_addr, uint8_t* rxdata, uint16_t size){
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	uint8_t register_address = ((reg_addr<<1)&0x7E)|0x80;
	if(HAL_SPI_Transmit(dev->spi_handle, &register_address, 1, dev->max_delay)!=HAL_OK){
		return rc522_fail;
	}
	if(HAL_SPI_Receive_DMA(dev->spi_handle, rxdata, size)!=HAL_OK){
		return rc522_fail;
	}
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return rc522_ok;
}

void static set_bit_mask(rc522_handle dev, uint8_t reg_addr, uint8_t mask){
	uint8_t temp;
	uint8_t buffer;
	read_register_poll(dev, reg_addr, &temp, 1);
	buffer = temp|mask;
	write_register_poll(dev, reg_addr, &buffer, 1);
}

void static clear_bit_mask(rc522_handle dev, uint8_t reg_addr, uint8_t mask){
	uint8_t temp;
	uint8_t buffer;
	read_register_poll(dev, reg_addr, &temp, 1);
	buffer = temp & ~mask;
	write_register_poll(dev, reg_addr, &buffer, 1);
}

void static antenna_on(rc522_handle dev){
	set_bit_mask(dev, RC522_REG_TX_CONTROL, 0x03);
}

void static antenna_off(rc522_handle dev){
	clear_bit_mask(dev, RC522_REG_TX_CONTROL, 0x03);
}

void static reset(rc522_handle dev){
	uint8_t buffer = RC522_CMD_SOFTRESET;
	write_register_poll(dev, RC522_REG_COMMAND, &buffer, 1);
}

rc522_handle rc522_init(rc522_config_t* config){
	rc522_handle dev = (rc522_handle)malloc(sizeof(struct rc522_t));
	if(dev==NULL) return NULL;
	dev->cs_pin=config->cs_pin;
	dev->cs_port=config->cs_port;
	dev->irq_pin=config->irq_pin;
	dev->irq_port=config->irq_port;
	dev->max_delay=config->max_delay;
	dev->rst_pin=config->rst_pin;
	dev->rst_port=config->rst_port;
	dev->spi_handle=config->spi_handle;
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);
	reset(dev);
	return dev;
}

rc522_return_status_t rc522_configure(rc522_handle dev, rc522_config_t* config){
	if(write_register_poll(dev, RC522_REG_TMODE, &(config->tmode.raw), 1)!=rc522_ok){
		return rc522_write_fail;
	}
	if(write_register_poll(dev, RC522_REG_TPRESCALER, &(config->tprescaler.raw), 1)!=rc522_ok){
		return rc522_write_fail;
	}
	if(write_register_poll(dev, RC522_REG_TRELOAD_LO, &(config->treloadlo.raw), 1)!=rc522_ok){
		return rc522_write_fail;
	}
	if(write_register_poll(dev, RC522_REG_TRELOAD_HI, &(config->treloadhi.raw), 1)!=rc522_ok){
		return rc522_write_fail;
	}
	if(write_register_poll(dev, RC522_REG_TX_ASK, &(config->txask.raw), 1)!=rc522_ok){
		return rc522_write_fail;
	}
	if(write_register_poll(dev, RC522_REG_MODE, &(config->mode.raw), 1)!=rc522_ok){
		return rc522_write_fail;
	}
	if(write_register_poll(dev, RC522_REG_TX_MODE, &(config->txmode.raw), 1)!=rc522_ok){
		return rc522_write_fail;
	}
	if(write_register_poll(dev, RC522_REG_RX_MODE, &(config->rxmode.raw), 1)!=rc522_ok){
		return rc522_write_fail;
	}
	if(write_register_poll(dev, RC522_REG_RF_CFG, &(config->rfcfg.raw), 1)!=rc522_ok){
		return rc522_write_fail;
	}
	antenna_on(dev);
	return rc522_ok;
}

rc522_return_status_t static card_command_start(rc522_handle dev, uint8_t* senddata, uint8_t sendlen){
	switch(dev->command){
	case RC522_CMD_MFAUTHENT:
	{
		dev->irq_en=0x12;
		dev->wait_irq=0x10;
		break;
	}
	case RC522_CMD_TRANSCEIVE:
	{
		dev->irq_en=0x63;
		dev->wait_irq=0x30; // sonra incelenecek
		break;
	}
	default:
	{
		dev->irq_en=0;
		dev->wait_irq=0;
		break;
	}
	}
	uint8_t buffer = dev->irq_en | 0x80;
	write_register_poll(dev, RC522_REG_COM_IEN, &buffer, 1);
	clear_bit_mask(dev, RC522_REG_COM_IRQ, 0x80);
	set_bit_mask(dev, RC522_REG_FIFO_LEVEL, 0x80);
	buffer = RC522_CMD_IDLE;
	write_register_poll(dev, RC522_REG_COMMAND, &buffer, 1);
	write_register_poll(dev, RC522_REG_FIFO_DATA, senddata, sendlen);
	buffer = dev->command;
	write_register_poll(dev, RC522_REG_COMMAND, &buffer, 1);
	if(dev->command==RC522_CMD_TRANSCEIVE){
		set_bit_mask(dev, RC522_REG_BIT_FRAMING, 0x80);
	}
	return rc522_ok;
}

rc522_return_status_t static card_command_finish(rc522_handle dev, uint8_t* backdata, uint16_t* backlen){
	if(dev->command==RC522_CMD_TRANSCEIVE){
		clear_bit_mask(dev, RC522_REG_BIT_FRAMING, 0x80);
	}
	uint8_t irq_status, error_status;
	read_register_poll(dev, RC522_REG_COM_IRQ, &irq_status, 1);
	if(irq_status&0x01){
		return rc522_timeout;
	}
	read_register_poll(dev, RC522_REG_ERROR, &error_status, 1);
	if(error_status&0x1B){
		return rc522_fail;
	}
	if(irq_status&0x01&dev->irq_en){
		return rc522_fail;
	}
	if(dev->command==RC522_CMD_TRANSCEIVE){
		uint8_t fifo_level, last_bits;
		read_register_poll(dev, RC522_REG_FIFO_LEVEL, &fifo_level, 1);
		read_register_poll(dev, RC522_REG_CONTROL, &last_bits, 1);
		last_bits = last_bits & 0x07;
		if(last_bits){
			*backlen = (fifo_level-1)*8 + last_bits;
		}else{
			*backlen = fifo_level*8;
		}
		if(fifo_level==0) fifo_level=1;
		if(fifo_level>16) fifo_level=16;
		read_register_poll(dev, RC522_REG_FIFO_DATA, backdata, fifo_level);
	}
	return rc522_ok;
}

rc522_return_status_t rc522_request_start(rc522_handle dev, uint8_t reqmode){
	uint8_t buffer = 0x07;
	dev->command=RC522_CMD_TRANSCEIVE;
	write_register_poll(dev, RC522_REG_BIT_FRAMING, &buffer, 1);
	card_command_start(dev, &reqmode, 1);
	return rc522_ok;
}

rc522_return_status_t rc522_request_finish(rc522_handle dev, uint8_t* tagtype){
	uint16_t backlen;
	card_command_finish(dev, tagtype, &backlen);
	if(backlen!=0x10){
		return rc522_fail;
	}
	return rc522_ok;
}

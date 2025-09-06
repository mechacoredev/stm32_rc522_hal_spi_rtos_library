/*
 * rc522deneme.h
 *
 *  Created on: Sep 6, 2025
 *      Author: Enes
 */

#ifndef INC_RC522DENEME_H_
#define INC_RC522DENEME_H_

#include "stm32f4xx_hal.h"

struct rc522_t;
typedef struct rc522_t* rc522_handle;

typedef enum {
    RC522_PICC_CMD_REQIDL      = 0x26, // idle kartları tarar (sadece uyuyan kartları)
    RC522_PICC_CMD_REQALL      = 0x52, // tüm kartları tarar (uyuyan + aktif)
    RC522_PICC_CMD_ANTICOLL    = 0x93, // çakışma önleme (UID'in ilk 5 byte’ını alır)
    RC522_PICC_CMD_SELECTTAG   = 0x93, // kart seçme komutu (ANTICOLL sonrası)
    RC522_PICC_CMD_AUTHENT1A   = 0x60, // oturum açma (Key A ile)
    RC522_PICC_CMD_AUTHENT1B   = 0x61, // oturum açma (Key B ile)
    RC522_PICC_CMD_READ        = 0x30, // 16 byte blok okuma
    RC522_PICC_CMD_WRITE       = 0xA0, // 16 byte blok yazma
    RC522_PICC_CMD_DECREMENT   = 0xC0, // değer bloklarından çıkarma
    RC522_PICC_CMD_INCREMENT   = 0xC1, // değer bloklarına ekleme
    RC522_PICC_CMD_RESTORE     = 0xC2, // değer bloklarını geri yükleme
    RC522_PICC_CMD_TRANSFER    = 0xB0, // değer bloklarını transfer etme
    RC522_PICC_CMD_HALT        = 0x50  // kartı durdur (HALT state)
} rc522_picc_command_t;

typedef enum{
	rc522_ok,
	rc522_fail,
	rc522_read_fail,
	rc522_write_fail,
	rc522_mask_fail,
	rc522_timeout,
}rc522_return_status_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t tprescaler_hi: 4;
		uint8_t tauto_restart: 1;
		uint8_t tgated: 2;
		uint8_t tauto: 1;
	}bits;
}rc522_tmodereg;

typedef union{
	uint8_t raw;
	struct{
		uint8_t tprescaler_lo: 8;
	}bits;
}rc522_tprescalerreg;

typedef union{
	uint8_t raw;
	struct{
		uint8_t treloadval_hi: 8;
	}bits;
}rc522_treloadreghi;

typedef union{
	uint8_t raw;
	struct{
		uint8_t treloadval_lo: 8;
	}bits;
}rc522_treloadreglo;

typedef union{
	uint8_t raw;
	struct{
		uint8_t crcpreset: 2;
		uint8_t reserved: 1;
		uint8_t polmfin: 1;
		uint8_t reserved2: 1;
		uint8_t txwaitrf: 1;
		uint8_t reserved3: 1;
		uint8_t msbfirst: 1;
	}bits;
}rc522_modereg;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 3;
		uint8_t invmod: 1;
		uint8_t txspeed: 3;
		uint8_t txcrcen: 1;
	}bits;
}rc522_txmodereg;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 2;
		uint8_t rxmultiple: 1;
		uint8_t rxnoerr: 1;
		uint8_t rxspeed: 3;
		uint8_t rxcrcen: 1;
	}bits;
}rc522_rxmodereg;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 6;
		uint8_t force100ask: 1;
		uint8_t reserved2: 1;
	}bits;
}rc522_txaskreg;

typedef union{
	uint8_t raw;
	struct{
		uint8_t reserved: 4;
		uint8_t rxgain: 3;
		uint8_t reserved2: 1;
	}bits;
}rc522_rfcfgreg;

typedef struct{
	SPI_HandleTypeDef* spi_handle;
	GPIO_TypeDef* rst_port;
	GPIO_TypeDef* cs_port;
	GPIO_TypeDef* irq_port;
	uint16_t rst_pin;
	uint16_t cs_pin;
	uint16_t irq_pin;
	uint32_t max_delay;
	rc522_tmodereg tmode;
	rc522_tprescalerreg tprescaler;
	rc522_treloadreghi treloadhi;
	rc522_treloadreglo treloadlo;
	rc522_modereg mode;
	rc522_txmodereg txmode;
	rc522_rxmodereg rxmode;
	rc522_txaskreg txask;
	rc522_rfcfgreg rfcfg;
}rc522_config_t;

rc522_handle rc522_init(rc522_config_t* config);
rc522_return_status_t rc522_configure(rc522_handle dev, rc522_config_t* config);
rc522_return_status_t rc522_request_start(rc522_handle dev, uint8_t reqmode);
rc522_return_status_t rc522_request_finish(rc522_handle dev, uint8_t* tagtype);
rc522_return_status_t rc522_request_start_interrupt(rc522_handle dev);
rc522_return_status_t rc522_request_finish_interrupt(rc522_handle dev, uint8_t* tagtype);

#endif /* INC_RC522DENEME_H_ */

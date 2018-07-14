/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2018  Flirc Inc.
 * Written by Jason Kotzin <jasonkotzin@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <libopencm3/sam/d/port.h>
#include <libopencm3/usb/usbd.h>

#include "timing.h"
#include "version.h"

//#define PLATFORM_HAS_DEBUG
//#define USBUART_DEBUG

#define BOARD_IDENT             "Black Magic Probe (SAMD), (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_DFU		"Black Magic (Upgrade) for Launchpad, (Firmware " FIRMWARE_VERSION ")"
#define DFU_IDENT               "Black Magic Firmware Upgrade (SAMD)"
#define DFU_IFACE_STRING	"hid"

extern uint8_t running_status;

#ifdef DEBUG_ME

#define LED_PORT	PORTA
#define LED_IDLE_RUN	GPIO11
#define LED_ERROR	GPIO10

#define TMS_PORT	PORTA
#define TMS_PIN		GPIO7

#define TCK_PORT	PORTA
#define TCK_PIN		GPIO5

#define TDI_PORT	PORTA
#define TDI_PIN		GPIO3

#define TDO_PORT	PORTA
#define TDO_PIN		GPIO2

#define SWO_PORT	PORTA
#define SWO_PIN		GPIO6

#define SWDIO_PORT	PORTA
#define SWDIO_PIN	TMS_PIN
#define SWDIO_PIN_NUM	7

#define SWCLK_PORT	PORTA
#define SWCLK_PIN	TCK_PIN

#define SRST_PORT	PORTA
#define SRST_PIN	GPIO6

#define LED_PORT_UART	PORTA
#define LED_UART	GPIO12

#else

/* Hardware definitions... */
#define JTAG_PORT 	PORTA
#define TDI_PORT	JTAG_PORT
#define TMS_DIR_PORT	JTAG_PORT
#define TMS_PORT	JTAG_PORT
#define TCK_PORT	JTAG_PORT
#define TDO_PORT	JTAG_PORT
#define TMS_DIR_PIN	GPIO15
#define TMS_PIN		GPIO31
#define TCK_PIN		GPIO30
#define TDI_PIN		GPIO5
#define TDO_PIN		GPIO4

#define SWDIO_DIR_PORT	JTAG_PORT
#define SWDIO_PORT 	JTAG_PORT
#define SWCLK_PORT 	JTAG_PORT
#define SWDIO_DIR_PIN	TMS_DIR_PIN
#define SWDIO_PIN	TMS_PIN
#define SWDIO_PIN_NUM	31
#define SWCLK_PIN	TCK_PIN

#define TRST_PORT	PORTA
#define TRST_PIN	GPIO27
#define PWR_BR_PORT	GPIOA
#define PWR_BR_PIN	GPIO28
#define SRST_PORT	GPIOA
#define SRST_PIN	GPIO8
#define SRST_SENSE_PORT	GPIOA
#define SRST_SENSE_PIN	GPIO9
#define TRGT_SENSE	GPIO2

#define LED_PORT	GPIOA
#define LED_PORT_UART	PORTA
#define LED_0		GPIO10
#define LED_1		GPIO11
#define LED_2		GPIO14
#define LED_UART	LED_1	/* Orange */
#define LED_IDLE_RUN	LED_0	/* Yellow */
#define LED_ERROR	LED_2	/* Red */

#define UART_TX_PIN	GPIO8
#define UART_RX_PIN	GPIO9
#define UART_PERIPH	SOC_GPIO_PERIPH_C

#define LED_PORT	PORTA
#define LED_IDLE_RUN	GPIO11

#define SWO_PORT	JTAG_PORT
#define SWO_PIN		SWD_PIN

#endif

#define TMS_SET_MODE()	{ \
	gpio_config_output(TMS_PORT, TMS_PIN, 0); \
}

#define SWDIO_MODE_FLOAT() do { \
	PORT_DIRCLR(SWDIO_PORT) = SWDIO_PIN; \
	gpio_set(SWDIO_PORT, SWDIO_PIN); \
} while(0)
#define SWDIO_MODE_DRIVE() do { \
	PORT_DIRSET(SWDIO_PORT) = SWDIO_PIN; \
} while(0)

/* extern usbd_driver samd21_usb_driver; */
#define USB_DRIVER	samd21_usb_driver
#define USB_IRQ		NVIC_USB_IRQ
#define USB_ISR		usb_isr

#define IRQ_PRI_USB	(2 << 4)

#define INLINE_GPIO

#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		_gpio_set((port), (pin));	\
	else					\
		_gpio_clear((port), (pin));	\
} while(0)

#ifdef INLINE_GPIO
static inline void _gpio_set(uint32_t gpioport, uint32_t gpios)
{
	PORT_OUTSET(gpioport) = gpios;
}
#define gpio_set _gpio_set

static inline void _gpio_clear(uint32_t gpioport, uint32_t gpios)
{
	PORT_OUTCLR(gpioport) = gpios;
}
#define gpio_clear _gpio_clear

static inline uint16_t _gpio_get(uint32_t gpioport, uint32_t gpios)
{
	return (uint32_t)PORT_IN(gpioport) & gpios;
}
#define gpio_get _gpio_get
#endif

#define DEBUG(...)

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(state)	{gpio_set_val(LED_PORT, LED_ERROR, state);}

static inline int platform_hwversion(void)
{
	        return 0;
}

void uart_pop(void);
#endif

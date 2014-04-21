/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2013 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>

#include "usbdfu.h"

__attribute__((__section__(".serialno"))) const volatile char board_serial_no[16] = "SERIAL NUM HERE";
uint32_t app_address = 0x08002000;

struct platform_output_pins_t {
	const uint32_t port;
	const uint16_t pin;
	const uint8_t cnf;
};

struct platform_uart_t {
	uint32_t usart;
	struct platform_output_pins_t tx;
	struct platform_output_pins_t rts;
	struct platform_output_pins_t dtr;
};

const struct platform_uart_t uarts[] = {{
	.usart = USART1,
	.tx =  {	// TX
		.port = GPIO_BANK_USART1_TX,
		.pin = GPIO_USART1_TX,
		.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	},
	.rts = {	// RTS
		.port = GPIOC,
		.pin = GPIO9,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
	.dtr = {	// DTR
		.port = GPIOC,
		.pin = GPIO12,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
},{
	.usart = USART2,
	.tx =  {	// TX
		.port = GPIO_BANK_USART2_TX,
		.pin = GPIO_USART2_TX,
		.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	},
	.rts = {	// RTS
		.port = GPIO_BANK_USART2_RTS,
		.pin = GPIO_USART2_RTS,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
	.dtr = {	// DTR
		.port = GPIOC,
		.pin = GPIO2,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
},{
	.usart = USART3,
	.tx =  {	// TX
		.port = GPIO_BANK_USART3_TX,
		.pin = GPIO_USART3_TX,
		.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	},
	.rts = {	// RTS
		.port = GPIO_BANK_USART3_RTS,
		.pin = GPIO_USART3_RTS,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
	.dtr = {	// DTR
		.port = GPIOC,
		.pin = GPIO5,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
},{
	.usart = UART4,
	.tx =  {	// TX
		.port = GPIO_BANK_UART4_TX,
		.pin = GPIO_UART4_TX,
		.cnf = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
	},
	.rts = {	// RTS
		.port = GPIOB,
		.pin = GPIO6,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
	.dtr = {	// DTR
		.port = GPIOB,
		.pin = GPIO5,
		.cnf = GPIO_CNF_OUTPUT_PUSHPULL,
	},
}};

void dfu_detach(void)
{
	dfu_jump_app_if_valid();

        /* USB device must detach, we just reset... */
	scb_reset_system();
}

int main(void)
{
	uint8_t i;
	
	/* ensure serialno is linked in */
	if( board_serial_no[0] ) {
	}
	
	dfu_protect(DFU_MODE);

	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);
	
	// Enable clocks for (GPIOA, GPIOB, GPIOC, GPIOD, USART1, AFIO)
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
	    RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
	    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN |
	    RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN);
	// Enable clocks all four uarts (USART2, USART3, UART4)
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
		RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN |
		RCC_APB1ENR_UART4EN);

	// Set all port pins to input pull-down
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_ALL);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_ALL);
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_ALL);
	gpio_set_mode(GPIOD, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO2);
	gpio_clear(GPIOA, GPIO_ALL);
	gpio_clear(GPIOB, GPIO_ALL);
	gpio_clear(GPIOC, GPIO_ALL);
	gpio_clear(GPIOD, GPIO2);

	// 31.4.3 - Internal pull-up and pull-down on JTAG pins
	// ● NJTRST: Input pull-up
	gpio_set_mode(GPIO_BANK_JNTRST, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_JNTRST);
	gpio_set(GPIO_BANK_JNTRST, GPIO_JNTRST);
	// ● JTDI: Input pull-up
	gpio_set_mode(GPIO_BANK_JTDI, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_JTDI);
	gpio_set(GPIO_BANK_JTDI, GPIO_JTDI);
	// ● JTMS/SWDIO: Input pull-up
	gpio_set_mode(GPIO_BANK_JTMS_SWDIO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_JTMS_SWDIO);
	gpio_set(GPIO_BANK_JTMS_SWDIO, GPIO_JTMS_SWDIO);
	// ● JTCK/SWCLK: Input pull-down
	gpio_set_mode(GPIO_BANK_JTCK_SWCLK, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_JTCK_SWCLK);
	gpio_clear(GPIO_BANK_JTCK_SWCLK, GPIO_JTCK_SWCLK);

	rcc_periph_clock_enable(RCC_USB);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, 0, GPIO8);

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO2 | GPIO10);

	//UART Setup
	for(i=0; i<4; i++) {
		gpio_set_mode(uarts[i].tx.port, GPIO_MODE_OUTPUT_50_MHZ, uarts[i].tx.cnf, uarts[i].tx.pin);
		gpio_set_mode(uarts[i].rts.port, GPIO_MODE_OUTPUT_50_MHZ, uarts[i].rts.cnf, uarts[i].rts.pin);
		gpio_set_mode(uarts[i].dtr.port, GPIO_MODE_OUTPUT_50_MHZ, uarts[i].dtr.cnf, uarts[i].dtr.pin);
		usart_set_baudrate(uarts[i].usart, 9600);
		usart_set_mode(uarts[i].usart, USART_MODE_TX_RX);
		usart_enable(uarts[i].usart);
	}

	dfu_init(&stm32f103_usb_driver, DFU_MODE);

	gpio_set(GPIOA, GPIO8);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

	dfu_main();
}


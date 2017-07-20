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
#include "i2c_tools.h"

const volatile char __attribute__((__section__(".serialno"))) board_serial_no[16] = "SERIAL NUM HERE";
uint32_t app_address = 0x08002000;

#define SERIAL_NUM_NUM_CHARS 3
const char serial_board_serial_numbers[][SERIAL_NUM_NUM_CHARS] = {
	"124",
	"129",
	"134",
};
const char i2c_board_serial_numbers[][SERIAL_NUM_NUM_CHARS] = {
	"817",	// 16 Channel Digital Input
	"818",	// 16 Channel Digital Output
	"839",	// 8 Channel Digital Input, 8 Channel Digital Output
	"830",	// 16 Channel Analogue Input
};

struct platform_output_pins_t {
	const uint32_t port;
	const uint16_t pin;
};

struct platform_uart_t {
	struct platform_output_pins_t tx;
	struct platform_output_pins_t rts;
	struct platform_output_pins_t dtr;
};

const struct platform_uart_t uarts[] = {{
	// USART1,
	.tx =  {	// TX
		.port = GPIO_BANK_USART1_TX,
		.pin = GPIO_USART1_TX,
	},
	.rts = {	// RTS
		.port = GPIOC,
		.pin = GPIO9,
	},
	.dtr = {	// DTR
		.port = GPIOC,
		.pin = GPIO12,
	},
},{
	// USART2,
	.tx =  {	// TX
		.port = GPIO_BANK_USART2_TX,
		.pin = GPIO_USART2_TX,
	},
	.rts = {	// RTS
		.port = GPIO_BANK_USART2_RTS,
		.pin = GPIO_USART2_RTS,
	},
	.dtr = {	// DTR
		.port = GPIOC,
		.pin = GPIO2,
	},
},{
	// USART3,
	.tx =  {	// TX
		.port = GPIO_BANK_USART3_TX,
		.pin = GPIO_USART3_TX,
	},
	.rts = {	// RTS
		.port = GPIO_BANK_USART3_RTS,
		.pin = GPIO_USART3_RTS,
	},
	.dtr = {	// DTR
		.port = GPIOC,
		.pin = GPIO5,
	},
},{
	// UART4,
	.tx =  {	// TX
		.port = GPIO_BANK_UART4_TX,
		.pin = GPIO_UART4_TX,
	},
	.rts = {	// RTS
		.port = GPIOB,
		.pin = GPIO6,
	},
	.dtr = {	// DTR
		.port = GPIOB,
		.pin = GPIO5,
	},
}};

void init_ports(void) {
	// Enable clocks for (GPIOA, GPIOB, GPIOC, GPIOD)
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
	    RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
	    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN);

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
};

void init_serial_ports(void) {
	uint8_t serial_num;
	uint8_t uart_num;

	// *** Special pin setup for UARTS and UART LEDs ***
	for(serial_num=0; serial_num<sizeof(serial_board_serial_numbers)/(SERIAL_NUM_NUM_CHARS*sizeof(char)); serial_num++) {
		if ((board_serial_no[0] == serial_board_serial_numbers[serial_num][0]) &&
			(board_serial_no[1] == serial_board_serial_numbers[serial_num][1]) &&
			(board_serial_no[2] == serial_board_serial_numbers[serial_num][2]))
		{
			for(uart_num=0; uart_num<4; uart_num++) {
				gpio_set_mode(uarts[uart_num].tx.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, uarts[uart_num].tx.pin);
				gpio_set_mode(uarts[uart_num].rts.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, uarts[uart_num].rts.pin);
				gpio_set_mode(uarts[uart_num].dtr.port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, uarts[uart_num].dtr.pin);
				gpio_set(uarts[uart_num].tx.port, uarts[uart_num].tx.pin);
				gpio_set(uarts[uart_num].rts.port, uarts[uart_num].rts.pin);
				gpio_set(uarts[uart_num].dtr.port, uarts[uart_num].dtr.pin);
			}
			break;
		}
	}
}

void init_i2c_devices(void) {
	uint8_t serial_num;
	// *** Special pin setup for I2C ***
	for(serial_num=0; serial_num<sizeof(i2c_board_serial_numbers)/(SERIAL_NUM_NUM_CHARS*sizeof(char)); serial_num++) {
		if ((board_serial_no[0] == i2c_board_serial_numbers[serial_num][0]) &&
			(board_serial_no[1] == i2c_board_serial_numbers[serial_num][1]) &&
			(board_serial_no[2] == i2c_board_serial_numbers[serial_num][2]))
		{
			char init_data[] = {0xFF, 0xFF};
			init_i2c_ports();
			i2c_write(I2C2, PCA8575_LED_ADDRESS, init_data, sizeof(init_data));
			I2C_SR1(I2C2);
			I2C_SR2(I2C2);
			i2c_peripheral_disable(I2C2);
			break;
		}
	}
}

void dfu_detach(void)
{
	dfu_jump_app_if_valid();

        /* USB device must detach, we just reset... */
	scb_reset_system();
}

int main(void)
{
	dfu_protect(DFU_MODE);

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	// set initial state of port pins
	init_ports();

	// set initial state of serial port pins
	init_serial_ports();

	// setup initial state of I2C devices
	init_i2c_devices();

	// setup systic
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);
	
	// Setup USB hardware
	rcc_periph_clock_enable(RCC_USB);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, 0, GPIO8);

	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO2 | GPIO10);

	dfu_init(&stm32f103_usb_driver, DFU_MODE);

	gpio_set(GPIOA, GPIO8);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

	dfu_main();
}

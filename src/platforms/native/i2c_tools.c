/***
 * Code copied from:
 *    git@github.com:yohanes-erwin/stm32f103-keil.git
 *    STM32F10x_StdPeriph_Lib_V3.5.0
 **/

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

#include "i2c_tools.h"

void restart_systick_count(void) {
	/* verify counter is stopped */
	systick_counter_disable();
	/* 9000000/(89999 + 1) = 100 overflows per second
	   - every 10ms one overflow and reset.
	   SysTick interrupt every N clock pulses: set reload to N-1
	 **/
	systick_set_reload(89999);
	/* Start the SysTick counter */
	systick_counter_enable();
}

void stop_systick(void) {
	systick_counter_disable();
	systick_interrupt_disable();
	systick_set_reload(0);
}

void init_systick(void) {
	// make certain SysTick is disabled
	stop_systick();
	// setup SysTick
	/* 72MHz / 8 => 9000000 counts per second */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* restart SysTick counter */
	restart_systick_count();
}

void reset_i2c_pins(uint32_t I2Cx) {
	// reseting I2C pins [errata_sheet CD00197763.pdf section 2.13.7]
	const uint16_t gpios[] = {
		GPIO_I2C2_SCL | GPIO_I2C2_SDA,
		GPIO_I2C2_SDA,
		GPIO_I2C2_SCL,
	};
	i2c_peripheral_disable(I2Cx);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, gpios[0]);
	for(uint8_t index_num=0; index_num<3; index_num++) {
		gpio_set(GPIOB, gpios[index_num]);
		restart_systick_count();
		while (!systick_get_countflag() &&
			!gpio_get(GPIOB, gpios[index_num]));
		gpio_clear(GPIOB, gpios[index_num]);
		restart_systick_count();
		while (!systick_get_countflag() &&
			gpio_get(GPIOB, gpios[index_num]));
	}
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, gpios[0]);
	I2C_CR1(I2Cx) |= I2C_CR1_SWRST;
	I2C_CR1(I2Cx) &= ~I2C_CR1_SWRST;
}

/**
  * @brief  Checks whether the specified I2C flag is set or not.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to check. 
  *   This parameter can be one of the following values:
  *     @arg I2C_FLAG_DUALF: Dual flag (Slave mode)
  *     @arg I2C_FLAG_SMBHOST: SMBus host header (Slave mode)
  *     @arg I2C_FLAG_SMBDEFAULT: SMBus default header (Slave mode)
  *     @arg I2C_FLAG_GENCALL: General call header flag (Slave mode)
  *     @arg I2C_FLAG_TRA: Transmitter/Receiver flag
  *     @arg I2C_FLAG_BUSY: Bus busy flag
  *     @arg I2C_FLAG_MSL: Master/Slave flag
  *     @arg I2C_FLAG_SMBALERT: SMBus Alert flag
  *     @arg I2C_FLAG_TIMEOUT: Timeout or Tlow error flag
  *     @arg I2C_FLAG_PECERR: PEC error in reception flag
  *     @arg I2C_FLAG_OVR: Overrun/Underrun flag (Slave mode)
  *     @arg I2C_FLAG_AF: Acknowledge failure flag
  *     @arg I2C_FLAG_ARLO: Arbitration lost flag (Master mode)
  *     @arg I2C_FLAG_BERR: Bus error flag
  *     @arg I2C_FLAG_TXE: Data register empty flag (Transmitter)
  *     @arg I2C_FLAG_RXNE: Data register not empty (Receiver) flag
  *     @arg I2C_FLAG_STOPF: Stop detection flag (Slave mode)
  *     @arg I2C_FLAG_ADD10: 10-bit header sent flag (Master mode)
  *     @arg I2C_FLAG_BTF: Byte transfer finished flag
  *     @arg I2C_FLAG_ADDR: Address sent flag (Master mode) "ADSL"
  *   Address matched flag (Slave mode)"ENDA"
  *     @arg I2C_FLAG_SB: Start bit flag (Master mode)
  * @retval The new state of I2C_FLAG (SET or RESET).
  */
FlagStatus I2C_GetFlagStatus(uint32_t I2Cx, uint32_t I2C_FLAG) {
	FlagStatus bitstatus = RESET;
	uint32_t status_reg = 0;

	/* Get bit[23:0] of the flag */
	if ((I2C_FLAG >> 28) != 0) {
		/* Get the I2Cx SR1 register */
		status_reg = I2C_SR1(I2Cx) & (I2C_FLAG & FLAG_Mask);
	} else {
		/* Flag in I2Cx SR2 Register */
		I2C_FLAG = (uint32_t)((I2C_FLAG & FLAG_Mask) >> 16);
		/* Get the I2Cx SR2 register */
		status_reg = I2C_SR2(I2Cx) & I2C_FLAG;
	}
	/* check the register */
	if (status_reg != (uint32_t)RESET) {
		/* I2C_FLAG is set */
		bitstatus = SET;
	}
	/* Return the I2C_FLAG status */
	return  bitstatus;
}

/**
  * @brief  Checks whether the last I2Cx Event is equal to the one passed
  *   as parameter.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_EVENT: specifies the event to be checked. 
  *   This parameter can be one of the following values:
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED           : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED              : EV1
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED     : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED        : EV1
  *     @arg I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED            : EV1
  *     @arg I2C_EVENT_SLAVE_BYTE_RECEIVED                         : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF)      : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL)    : EV2
  *     @arg I2C_EVENT_SLAVE_BYTE_TRANSMITTED                      : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF)   : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL) : EV3
  *     @arg I2C_EVENT_SLAVE_ACK_FAILURE                           : EV3_2
  *     @arg I2C_EVENT_SLAVE_STOP_DETECTED                         : EV4
  *     @arg I2C_EVENT_MASTER_MODE_SELECT                          : EV5
  *     @arg I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6     
  *     @arg I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
  *     @arg I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
  *     @arg I2C_EVENT_MASTER_MODE_ADDRESS10                       : EV9
  *     
  * @note: For detailed description of Events, please refer to section 
  *    I2C_Events in stm32f10x_i2c.h file.
  *    
  * @retval An ErrorStatus enumeration value:
  * - SUCCESS: Last event is equal to the I2C_EVENT
  * - ERROR: Last event is different from the I2C_EVENT
  */
ErrorStatus I2C_CheckEvent(uint32_t I2Cx, uint32_t I2C_EVENT) {
	uint32_t lastevent = 0;
	uint32_t status_reg = 0;
	ErrorStatus status = ERROR;

    /* Read the I2Cx status register */
	status_reg = I2C_SR1(I2Cx) | (I2C_SR2(I2Cx) << 16);

	/* Get the last event value from I2C status register */
	lastevent = status_reg & FLAG_Mask;

	/* Check whether the last event contains the I2C_EVENT */
	if ((lastevent & I2C_EVENT) == I2C_EVENT) {
		status = SUCCESS;
	}

	/* Return status */
	return status;
}

void i2c_start(uint32_t I2Cx) {
	// Wait until I2Cx is not busy anymore
	restart_systick_count();
	while (!systick_get_countflag() &&
		I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	// Generate start condition
	i2c_send_start(I2Cx);
	// Wait for I2C EV5. 
	// It means that the start condition has been correctly released 
	// on the I2C bus (the bus is free, no other devices is communicating))
	restart_systick_count();
	while (!systick_get_countflag() &&
		!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
}

void i2c_stop(uint32_t I2Cx) {
	// Generate I2C stop condition
	i2c_send_stop(I2Cx);
	// Wait until I2C stop condition is finished
	restart_systick_count();
	while (!systick_get_countflag() &&
		I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
}

void i2c_address_direction(uint32_t I2Cx, uint8_t address, uint8_t direction) {
	// Send slave address
	i2c_send_7bit_address(I2Cx, address, direction);
	// Wait for I2C EV6
	// It means that a slave acknowledges his address
	if (direction == I2C_WRITE) {
		restart_systick_count();
		while (!systick_get_countflag() &&
			!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	} else if (direction == I2C_READ) {
		restart_systick_count();
		while (!systick_get_countflag() &&
			!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

void i2c_transmit(uint32_t I2Cx, uint8_t byte) {
	// Send data byte
	i2c_send_data(I2Cx, byte);
	// Wait for I2C EV8_2
	// It means the data has been physically shifted out and
	// output on the bus
	restart_systick_count();
	while (!systick_get_countflag() &&
		!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

void i2c_write(uint32_t I2Cx, uint8_t address, uint8_t *data, uint8_t num_char) {
	uint8_t char_num;

	i2c_start(I2Cx);
	i2c_address_direction(I2Cx, address, I2C_WRITE);
	for(char_num=0; char_num<(num_char*sizeof(char)); char_num++) {
		i2c_transmit(I2Cx, data[char_num]);
	}
	i2c_stop(I2Cx);
}

void init_i2c_ports(void) {
	// Enable I2C2 peripheral clock
	rcc_periph_clock_enable(RCC_I2C2);
	// Code copied from libopencm3-examples:examples/stm32/f1/other/i2c_stts75_sensor
	reset_i2c_pins(I2C2);
	// This is the slave address when not transmitting data
	i2c_set_own_7bit_slave_address(I2C2, 0x32);
	// do not respond to the specified slave address
	i2c_disable_ack(I2C2);
	// APB1 is running at 36MHz = T(PCLK1) = 1/36000000 sec.
	i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_36MHZ);
	// Set up the hardware for the particular speed
	{	// case I2C_50KHz:
		// I2C PERIOD: 50KHz = 50000Hz = 1/50000 sec.
		i2c_set_standard_mode(I2C2);
		// I2C_CCR_DUTY_DIV2 or I2C_CCR_DUTY_16_DIV_9
		i2c_set_dutycycle(I2C2, I2C_CCR_DUTY_DIV2);
		// CCR = PERIOD / (2 * T(PCLK1))
		// CCR = (1/50000) / (2/36000000) = 360
		//   (341 works but not 342 or higher)
		i2c_set_ccr(I2C2, 341);
		// TRISE = ( (1000/1000000000) / (1/36000000) ) + 1 = 37
		i2c_set_trise(I2C2, 37);

	}
	// clear status registers
	I2C_SR1(I2C2);
	I2C_SR2(I2C2);
	// enable I2C
	i2c_peripheral_enable(I2C2);
}

/***
 * Code copied from:
 *    git@github.com:yohanes-erwin/stm32f103-keil.git
 *    STM32F10x_StdPeriph_Lib_V3.5.0
 **/

#ifndef __I2C_TOOLS_H_
#define __I2C_TOOLS_H_

#include <libopencm3/stm32/i2c.h>

#define PCA8575_LED_ADDRESS (0x20)

/* I2C FLAG mask */
#define FLAG_Mask ((uint32_t)0x00FFFFFF)

/** 
  * @brief  flags used with I2C_GetFlagStatus()
  */

/* @brief  SR2 register flags   */
#define I2C_FLAG_DUALF                  ((uint32_t)(I2C_SR2_DUALF << 16))			// 0x00800000
#define I2C_FLAG_SMBHOST                ((uint32_t)(I2C_SR2_SMBHOST << 16))			// 0x00400000
#define I2C_FLAG_SMBDEFAULT             ((uint32_t)(I2C_SR2_SMBDEFAULT << 16))		// 0x00200000
#define I2C_FLAG_GENCALL                ((uint32_t)(I2C_SR2_GENCALL << 16))			// 0x00100000
#define I2C_FLAG_TRA                    ((uint32_t)(I2C_SR2_TRA << 16))				// 0x00040000
#define I2C_FLAG_BUSY                   ((uint32_t)(I2C_SR2_BUSY << 16))			// 0x00020000
#define I2C_FLAG_MSL                    ((uint32_t)(I2C_SR2_MSL << 16))				// 0x00010000
/* @brief  SR1 register flags   */
#define I2C_FLAG_SMBALERT               ((uint32_t)(I2C_SR1_SMBALERT | (1 << 28)))	// 0x10008000
#define I2C_FLAG_TIMEOUT                ((uint32_t)(I2C_SR1_TIMEOUT | (1 << 28)))	// 0x10004000
#define I2C_FLAG_PECERR                 ((uint32_t)(I2C_SR1_PECERR | (1 << 28)))	// 0x10001000
#define I2C_FLAG_OVR                    ((uint32_t)(I2C_SR1_OVR | (1 << 28)))		// 0x10000800
#define I2C_FLAG_AF                     ((uint32_t)(I2C_SR1_AF | (1 << 28)))		// 0x10000400
#define I2C_FLAG_ARLO                   ((uint32_t)(I2C_SR1_ARLO | (1 << 28)))		// 0x10000200
#define I2C_FLAG_BERR                   ((uint32_t)(I2C_SR1_BERR | (1 << 28)))		// 0x10000100
#define I2C_FLAG_TXE                    ((uint32_t)(I2C_SR1_TxE | (1 << 28)))		// 0x10000080
#define I2C_FLAG_RXNE                   ((uint32_t)(I2C_SR1_RxNE | (1 << 28)))		// 0x10000040
#define I2C_FLAG_STOPF                  ((uint32_t)(I2C_SR1_STOPF | (1 << 28)))		// 0x10000010
#define I2C_FLAG_ADD10                  ((uint32_t)(I2C_SR1_ADD10 | (1 << 28)))		// 0x10000008
#define I2C_FLAG_BTF                    ((uint32_t)(I2C_SR1_BTF | (1 << 28)))		// 0x10000004
#define I2C_FLAG_ADDR                   ((uint32_t)(I2C_SR1_ADDR | (1 << 28)))		// 0x10000002
#define I2C_FLAG_SB                     ((uint32_t)(I2C_SR1_SB | (1 << 28)))		// 0x10000001

/** 
  * @brief  Communication start
  * 
  * After sending the START condition (I2C_GenerateSTART() function) the master 
  * has to wait for this event. It means that the Start condition has been correctly 
  * released on the I2C bus (the bus is free, no other devices is communicating).
  * 
  */
/* --EV5 */
#define  I2C_EVENT_MASTER_MODE_SELECT (((I2C_SR2_BUSY | I2C_SR2_MSL) << 16) | (I2C_SR1_SB))     /* BUSY, MSL and SB flag */
/** 
  * @brief  Address Acknowledge
  * 
  * After checking on EV5 (start condition correctly released on the bus), the 
  * master sends the address of the slave(s) with which it will communicate 
  * (I2C_Send7bitAddress() function, it also determines the direction of the communication: 
  * Master transmitter or Receiver). Then the master has to wait that a slave acknowledges 
  * his address. If an acknowledge is sent on the bus, one of the following events will 
  * be set:
  * 
  *  1) In case of Master Receiver (7-bit addressing): the I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED 
  *     event is set.
  *  
  *  2) In case of Master Transmitter (7-bit addressing): the I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED 
  *     is set
  *  
  *  3) In case of 10-Bit addressing mode, the master (just after generating the START 
  *  and checking on EV5) has to send the header of 10-bit addressing mode (I2C_SendData() 
  *  function). Then master should wait on EV9. It means that the 10-bit addressing 
  *  header has been correctly sent on the bus. Then master should send the second part of 
  *  the 10-bit address (LSB) using the function I2C_Send7bitAddress(). Then master 
  *  should wait for event EV6. 
  *     
  */
/* --EV6 */
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED (((I2C_SR2_BUSY | I2C_SR2_MSL | I2C_SR2_TRA) << 16) | (I2C_SR1_TxE | I2C_SR1_ADDR))  /* BUSY, MSL, ADDR, TXE and TRA flags */
#define  I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED (((I2C_SR2_BUSY | I2C_SR2_MSL) << 16) | (I2C_SR1_ADDR))  /* BUSY, MSL and ADDR flags */

/** 
  * @brief Communication events
  * 
  * If a communication is established (START condition generated and slave address 
  * acknowledged) then the master has to check on one of the following events for 
  * communication procedures:
  *  
  * 1) Master Receiver mode: The master has to wait on the event EV7 then to read 
  *    the data received from the slave (I2C_ReceiveData() function).
  * 
  * 2) Master Transmitter mode: The master has to send data (I2C_SendData() 
  *    function) then to wait on event EV8 or EV8_2.
  *    These two events are similar: 
  *     - EV8 means that the data has been written in the data register and is 
  *       being shifted out.
  *     - EV8_2 means that the data has been physically shifted out and output 
  *       on the bus.
  *     In most cases, using EV8 is sufficient for the application.
  *     Using EV8_2 leads to a slower communication but ensure more reliable test.
  *     EV8_2 is also more suitable than EV8 for testing on the last data transmission 
  *     (before Stop condition generation).
  *     
  *  @note In case the  user software does not guarantee that this event EV7 is 
  *  managed before the current byte end of transfer, then user may check on EV7 
  *  and BTF flag at the same time (ie. (I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF)).
  *  In this case the communication may be slower.
  * 
  */
/* Master RECEIVER mode -----------------------------*/ 
/* --EV7 */
#define  I2C_EVENT_MASTER_BYTE_RECEIVED (((I2C_SR2_BUSY | I2C_SR2_MSL) << 16) | (I2C_SR1_RxNE))  /* BUSY, MSL and RXNE flags */
/* Master TRANSMITTER mode --------------------------*/
/* --EV8 */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING (((I2C_SR2_TRA | I2C_SR2_BUSY | I2C_SR2_MSL) << 16) | (I2C_SR1_TxE)) /* TRA, BUSY, MSL, TXE flags */
/* --EV8_2 */
#define  I2C_EVENT_MASTER_BYTE_TRANSMITTED (((I2C_SR2_TRA | I2C_SR2_BUSY | I2C_SR2_MSL) << 16) | (I2C_SR1_TxE | I2C_SR1_BTF)) /* TRA, BUSY, MSL, TXE and BTF flags */


typedef enum {RESET = 0, SET = !RESET} FlagStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionState;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

void stop_systick(void);
void init_systick(void);
void i2c_timeout(void);
void i2c_write(uint32_t I2Cx, uint8_t address, uint8_t *data, uint8_t num_char);
void init_i2c_ports(void);

#endif /* __I2C_TOOLS_H_ */

/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Jan 21, 2025
 *      Author: sfkfa
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * I2C SCL speed
 */
#define I2C_SPEED_SM     100000   //Standard mode 100 KHz
#define I2C_SPEED_FM     400000   //Fast mode 400 KHz

/*
 * I2C FM Duty Cycle
 */
#define I2C_ACK_ENABLE     1
#define I2C_ACK_DISABLE    0

/*
 * I2C ACK Control
 */
#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

/*
 * I2C related status definition flags
 */
#define I2C_TXE_FLAG           (1 << I2C_SR1_TxE)
#define I2C_RXNE_FLAG          (1 << I2C_SR1_RxNE)
#define I2C_SB_FLAG            (1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG     	   (1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG           (1 << I2C_SR1_BTF)
#define I2C_STOPF_FLAG         (1 << I2C_SR1_STOPF)
#define I2C_BERR_FLAG          (1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG          (1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG            (1 << I2C_SR1_AF)
#define I2C_OVR_FLAG           (1 << I2C_SR1_OVR)
#define I2C_TIMEOUT_FLAG       (1 << I2C_SR1_TIMEOUT)


/*
 * Configuration structure for I2Cx peripheral
 */

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint32_t I2C_DeviceAddress;
	uint32_t I2C_ACKControl;
	uint32_t I2C_FMDutyCycle;
}I2C_Config_t;


/*
 * Handle structure for I2Cx peripheral
 */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;



/*******************************************************************************
 *                    APIs supported by this driver
 *        For more information about the APIs check the function definitions
 *******************************************************************************/

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Data send & receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t * pTxBuff,uint32_t len,uint8_t slaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t * pRxBuff,uint32_t len,uint8_t slaveAddr);


// APIs Works in interrupt mode

/*
 * Other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);
/*
 * IRQ Configuration and ISR handlingb
 */

/*
 * Application callBack
 */


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */

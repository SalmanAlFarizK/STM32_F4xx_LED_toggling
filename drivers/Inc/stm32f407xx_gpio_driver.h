/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Sep 30, 2024
 *      Author: Salman
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinCOnfig_t;

/*
 * This is a handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t * pGPIOx; /*This holds the base address of the GPIO port to which the pin belongs*/
	GPIO_PinCOnfig_t GPIO_PinConfig; /*This holds the GPIO pin configuration settings*/
}GPIO_Handle_t;


/*
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN       0
#define GPIO_MODE_OUT      1
#define GPIO_MODE_ALTFN    2
#define GPIO_MODE_ANALOG   3
#define GPIO_MODE_IT_FT    4       //GPIO i/p mode with falling edge
#define GPIO_MODE_IT_RT    5       //GPIO i/p mode with rising edge
#define GPIO_MODE_IT_RFT   6       ////GPIO i/p mode with rising edge falling edge trigger


/*
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP       0 //push pull
#define GPIO_OP_TYPE_OD       1 //open drain

/*
 * GPIO pin possible output speed
 */
#define GPIO_SPEED_LOW                0
#define GPIO_SPEED_MEDIUM             1
#define GPIO_SPEED_FAST               2
#define GPIO_SPEED_HIGH               3

/*
 * GPIO pin pull up AND pull down configuration macros
 */

#define GPIO_NO_PUPD         0
#define GPIO_PIN_PU          1
#define GPIO_PIN_PD          2

/*
 * GPIO Pin Numbers
 */

#define GPIO_PIN_NO_0         0
#define GPIO_PIN_NO_1         1
#define GPIO_PIN_NO_2         2
#define GPIO_PIN_NO_3         3
#define GPIO_PIN_NO_4         4
#define GPIO_PIN_NO_5         5
#define GPIO_PIN_NO_6         6
#define GPIO_PIN_NO_7         7
#define GPIO_PIN_NO_8         8
#define GPIO_PIN_NO_9         9
#define GPIO_PIN_NO_10        10
#define GPIO_PIN_NO_11        11
#define GPIO_PIN_NO_12        12
#define GPIO_PIN_NO_13        13
#define GPIO_PIN_NO_14        14
#define GPIO_PIN_NO_15        15

/*******************************************************************************
 *                    APIs supported by this driver
 *        For more information about the APIs check the function definitions
 *******************************************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

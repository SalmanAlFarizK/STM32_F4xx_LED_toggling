/*
 * stm32f407xx.h
 *
 *  Created on: Sep 25, 2024
 *      Author: sfkfa
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#define __vo volatile

/*
 * Base addresses of flash memeory and SRAM memories
 */

#define FLASH_BASEADDR         0x08000000u
#define SRAM1_BASEADDR         0x20000000u
#define SRAM2_BASEADDR         0x2001C000u
#define ROM_BASEADDR           0x1FFF0000u
#define SRAM                   SRAM1_BASEADDR

/*
 * AHBx & APBx Peripheral base addresses
 */
#define PERIPH_BASE            0x40000000u
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE        0x40010000u
#define AHB1PERIPH_BASE        0x40020000u
#define AHB2PERIPH_BASE        0x50000000u

/*
 * Base addresses of peripherals which are hanging on AHB1 Bus
 */

#define GPIOA_BASEADDR         (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR         (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR         (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR         (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR         (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR         (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR         (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR         (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR         (AHB1PERIPH_BASE + 0x2000)
#define RCC_BASEADDR           (AHB1PERIPH_BASE + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 Bus
 */

#define I2C1_BASEADDR          (APB1PERIPH_BASE + 0X5400)
#define I2C2_BASEADDR          (APB1PERIPH_BASE + 0X5800)
#define I2C3_BASEADDR          (APB1PERIPH_BASE + 0X5C00)
#define SPI2_BASEADDR          (APB1PERIPH_BASE + 0X3800)
#define SPI3_BASEADDR          (APB1PERIPH_BASE + 0X3C00)
#define USART2_BASEADDR        (APB1PERIPH_BASE + 0X4400)
#define USART3_BASEADDR        (APB1PERIPH_BASE + 0X4800)
#define UART4_BASEADDR         (APB1PERIPH_BASE + 0X4C00)
#define UART5_BASEADDR         (APB1PERIPH_BASE + 0X5000)


/*
 * Base addresses of peripherals which are hanging on APB2 Bus
 */
#define SPI1_BASEADDR          (APB2PERIPH_BASE + 0X3000)
#define USART1_BASEADDR        (APB2PERIPH_BASE + 0X1000)
#define USART6_BASEADDR        (APB2PERIPH_BASE + 0X1400)
#define EXTI_BASEADDR          (APB2PERIPH_BASE + 0X3C00)
#define SYSCFG_BASEADDR        (APB2PERIPH_BASE + 0X3800)


/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRRL;
	__vo uint32_t BSRRH;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/*
 * Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA             ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB             ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC             ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD             ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE             ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF             ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG             ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH             ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI             ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC               ((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()   ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()   ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()   ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()   ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()   ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()   ( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()   ( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()   ( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()   ( RCC->AHB1ENR |= (1 << 8) )

/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()   ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()   ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()   ( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()   ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()   ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()   ( RCC->APB1ENR |= (1 << 15) )

/*
 * Clock enable macros for USART peripherals
 */
#define USART1_PCLK_EN()   ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()   ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()   ( RCC->APB1ENR |= (1 << 18) )
#define USART6_PCLK_EN()   ( RCC->APB2ENR |= (1 << 5) )


/*
 * Clock enable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()   ( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock disable macros for GPIOx peripheral
 */
#define GPIOA_PCLK_DI()   ( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()   ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()   ( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()   ( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()   ( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()   ( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()   ( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()   ( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()   ( RCC->AHB1ENR &= ~(1 << 8) )


/*
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 23) )


/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 15) )

/*
 * Clock diable macros for USARTx peripherals
 */
#define USART1_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 18) )
#define USART6_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 5) )

/*
 * Clock diable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 14) )

/*
 * Macros to reset GPIOx peripherals
 */
/*Note : Below is the syntax C programming technique for defineing macro functions with multiple lines*/
#define GPIOA_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 0)); (RCC->APB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 1)); (RCC->APB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 2)); (RCC->APB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 3)); (RCC->APB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 4)); (RCC->APB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 5)); (RCC->APB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 6)); (RCC->APB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 7)); (RCC->APB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()   do{(RCC->APB1RSTR |= (1 << 8)); (RCC->APB1RSTR &= ~(1 << 8)); } while(0)

//Some generic macros
#define ENABLE         1
#define DISABLE        0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET RESET




#endif /* INC_STM32F407XX_H_ */

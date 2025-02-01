/*
 * stm32f407xx.h
 *
 *  Created on: Sep 25, 2024
 *      Author: sfkfa
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include<string.h>
#define __vo volatile
#define __weak __attribute__((weak))

/********************************** Start processor specific details**********************/
/*
 * ARM Cortex Mx Processor NVIC ISERx (Interrupt set enable register) Addresses
 */
#define NVIC_ISER0               ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1               ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2               ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3               ((__vo uint32_t*)0xE000E10C)


/*
 * ARM Cortex Mx Processor NVIC ICERx (Interrupt Clear enable register) Addresses
 */
#define NVIC_ICER0               ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1               ((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2               ((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3               ((__vo uint32_t*)0XE000E18C)



/*
 * ARM Cortex Mx Processor Priority register address calculation
 */

#define NVIC_PR_BASE_ADDR        ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED    4


/*
 * IRQ (Interrupt Request) Number of STM32F407xx MCU
 */

#define IRQ_NO_EXTI0                6
#define IRQ_NO_EXTI1                7
#define IRQ_NO_EXTI2                8
#define IRQ_NO_EXTI3                9
#define IRQ_NO_EXTI4                10
#define IRQ_NO_EXTI9_5              23
#define IRQ_NO_EXTI15_10            40
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51
#define IRQ_NO_SPI4					84
#define IRQ_NO_SPI5					85
#define IRQ_NO_SPI6					86
/*
 * Macros for all the possible priorities levels
 */

#define  NVIC_IRQ_PRI0            0
#define  NVIC_IRQ_PRI15            15

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



/********************************************************************
 * Bit position definitions of SPI peripheral
 ********************************************************************/

/*
 * Bit position definition for SPI_CR1
 */
#define SPI_CR1_CPHA      0
#define SPI_CR1_CPOL      1
#define SPI_CR1_MSTR      2
#define SPI_CR1_BR        3
#define SPI_CR1_SPE       6
#define SPI_CR1_LSBFIRST  7
#define SPI_CR1_SSI       8
#define SPI_CR1_SSM       9
#define SPI_CR1_RXONLY    10
#define SPI_CR1_DFF       11
#define SPI_CR1_CRCNEXT   12
#define SPI_CR1_CRCEN     13
#define SPI_CR1_BIDIOE    14
#define SPI_CR1_BIDIMODE  15

/*
 * Bit position definition for SPI_CR2
 */
#define SPI_CR2_RXDMAEN      0
#define SPI_CR2_TXDMAEN      1
#define SPI_CR2_SSOE      	 2
#define SPI_CR2_FRF          4
#define SPI_CR2_ERRIE        5
#define SPI_CR2_RXNEIE  	 6
#define SPI_CR2_TXEIE  		 7

/*
 * Bit position definition for SPI_SR
 */
#define SPI_SR_RXNE      0
#define SPI_SR_TXE       1
#define SPI_SR_CHSIDE    2
#define SPI_SR_UDR       3
#define SPI_SR_CRCERR    4
#define SPI_SR_MODF 	 5
#define SPI_SR_OVR  	 6
#define SPI_SR_BSY  	 7
#define SPI_SR_FRE  	 8



/********************************************************************
 * Bit position definitions of I2C peripheral
 ********************************************************************/

/*
 * Bit position definition for I2C_CR1
 */
#define I2C_CR1_PE           0
#define I2C_CR1_SMBUS        1
#define I2C_CR1_SMBTYPE      3
#define I2C_CR1_ENARP        4
#define I2C_CR1_ENPEC        5
#define I2C_CR1_ENGC         6
#define I2C_CR1_NOSTRETCH    7
#define I2C_CR1_START        8
#define I2C_CR1_STOP         9
#define I2C_CR1_ACK          10
#define I2C_CR1_POS          11
#define I2C_CR1_PEC          12
#define I2C_CR1_ALERT        13
#define I2C_CR1_SWRST        15


/*
 * Bit position definition for I2C_SR1
 */
#define I2C_SR1_SB           0
#define I2C_SR1_ADDR         1
#define I2C_SR1_BTF          2
#define I2C_SR1_ADD10        3
#define I2C_SR1_STOPF        4
#define I2C_SR1_RxNE         6
#define I2C_SR1_TxE          7
#define I2C_SR1_BERR         8
#define I2C_SR1_ARLO         9
#define I2C_SR1_AF           10
#define I2C_SR1_OVR          11
#define I2C_SR1_PECERR       12
#define I2C_SR1_TIMEOUT      14
#define I2C_SR1_SMBALERT     15


/*
 * Bit position definition for I2C_SR2
 */
#define I2C_SR2_MSL           0
#define I2C_SR2_BUSY          1
#define I2C_SR2_TRA           2
#define I2C_SR2_GENCALL       4
#define I2C_SR2_SMBDEFAULT    5
#define I2C_SR2_SMBHOST       6
#define I2C_SR2_DUALF         7




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



/*
 * Peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
}SPI_RegDef_t;



/*
 * Peripheral register definition structure for I2C
 */

typedef struct
{
	__vo uint32_t I2C_CR1;
	__vo uint32_t I2C_CR2;
	__vo uint32_t I2C_OAR1;
	__vo uint32_t I2C_OAR2;
	__vo uint32_t I2C_DR;
	__vo uint32_t I2C_SR1;
	__vo uint32_t I2C_SR2;
	__vo uint32_t I2C_CCR;
	__vo uint32_t I2C_TRISE;
	__vo uint32_t I2C_FLTR;
}I2C_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
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
 * Peripheral register definition structure for EXTI
 */
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMCPR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;


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
#define EXTI              ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG            ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1              ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2              ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3              ((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1              ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2              ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3              ((I2C_RegDef_t*)I2C3_BASEADDR)

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

/*
 * Return Port Code for given GPIO base address
 */

#define GPIO_BASEADDR_TO_CODE(x)     ( (x == GPIOA)?0:\
		                               (x == GPIOB)?1:\
				                       (x == GPIOC)?2:\
				                       (x == GPIOD)?3:\
						               (x == GPIOE)?4:\
								       (x == GPIOF)?5:0)


//Some generic macros
#define ENABLE         1
#define DISABLE        0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET     RESET
#define FLAG_SET     SET





#endif /* INC_STM32F407XX_H_ */

/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Nov 19, 2024
 *      Author: salman
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * SPI Device Mode
 */
#define SPI_DEVICE_MODE_SLAVE         0
#define SPI_DEVICE_MODE_MASTER        1

/*
 * SPI Bus Config
 */
#define SPI_BUS_CONFIG_FD                      1  /*Full Duplex*/
#define SPI_BUS_CONFIG_HD                      2  /*Half Duplex*/
#define SPI_BUS_CONFIG_SIMPLEX_TX_ONLY         3  /*Simplex Tx Duplex*/
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY         4  /*Simplex Rx Duplex*/

/*
 * SPI Sclk Speed
 */
#define SPI_SCLK_SPEED_DIV2                      0
#define SPI_SCLK_SPEED_DIV4                      1
#define SPI_SCLK_SPEED_DIV8                      2
#define SPI_SCLK_SPEED_DIV16                     3
#define SPI_SCLK_SPEED_DIV32                     4
#define SPI_SCLK_SPEED_DIV64                     5
#define SPI_SCLK_SPEED_DIV128                    6
#define SPI_SCLK_SPEED_DIV256                    7

/*
 * SPI DFF
 */
#define SPI_DFF_8BITS                      0
#define SPI_DFF_16BITS                     1

/*
 * SPI CPOL
 */
#define SPI_CPOL_HIGH                      1
#define SPI_CPOL_LOW                       0

/*
 * SPI CPHA
 */
#define SPI_CPHA_HIGH                      1
#define SPI_CPHA_LOW                       0

/*
 * SPI SSM
 */
#define SPI_SSM_EN                      1
#define SPI_SSM_DI                      0


/*
 * Spi Possible states
 */
#define SPI_READY         0
#define SPI_BSY_IN_RX     1
#define SPI_BSY_IN_TX     2


/*
 * Spi Possible application events
 */
#define SPI_EVENT_TX_CMPLT     1
#define SPI_EVENT_RX_CMPLT     2
#define SPI_EVENT_OVR_ERR      3
#define SPI_EVENT_CRC_ERR      4

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; /*This holds the base address of SPIx Peripheral*/
	SPI_Config_t SPIConfig;
	uint8_t * pTxBuff;  /* !< To store app. Tx buffer address> */
	uint8_t * pRxBuff;  /* !< To store app. Rx buffer address> */
	uint32_t txLen;		/* !< To store Tx Length > */
	uint32_t rxLen;		/* !< To store Rx Length > */
	uint8_t txState;	/* !< To store Tx State > */
	uint8_t rxState;	/* !< To store Rx State > */
}SPI_Handle_t;

/*******************************************************************************
 *                    APIs supported by this driver
 *        For more information about the APIs check the function definitions
 *******************************************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Data send & receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuff,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuff,uint32_t Len);

// APIs Works in interrupt mode
uint8_t SPI_SendDataIT(SPI_Handle_t *pHandle,uint8_t *pTxBuff,uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pHandle,uint8_t *pRxBuff,uint32_t Len);

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi);
/*
 * IRQ Configuration and ISR handlingb
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_CleaOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pHandle);

/*
 * Application callBack
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */

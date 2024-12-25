/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 19, 2024
 *      Author: salman
 */
#include "stm32f407xx_spi_driver.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
        	SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
        	SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
        	SPI3_PCLK_EN();
        }
    }
    else if (EnorDi == DISABLE)
    {
        if (pSPIx == SPI1)
        {
        	SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
        	SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
        	SPI3_PCLK_DI();
        }
    }
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//First Configure SPI_CR_1 register
	uint32_t tempRegister = 0;

	//Peripheral clock initailization
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Configure the Device mode
	tempRegister |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.Configure Bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode shuld be cleared
		tempRegister &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode shuld be set
		tempRegister |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		//BIDI mode shuld be clear
		tempRegister &= ~(1 << SPI_CR1_BIDIMODE);
		//Rx Only bit must be set
		tempRegister |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the SPI serial clock speed (baud rate)
	tempRegister |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempRegister |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempRegister |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempRegister |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7.Configure the SSM
	tempRegister |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->SPI_CR1 = tempRegister;

}


void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuff,uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(!(pSPIx->SPI_SR & (1 << 1)) );

		//2. Check the DFF bit in CR1
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1.Load the data into data register (DR)
			pSPIx->SPI_DR = *((uint16_t*)pTxBuff);
			Len--;
			Len--;
			(uint16_t*)pTxBuff++;
		}
		else
		{
			//8 bit DFF
			pSPIx->SPI_DR = *(pTxBuff);
			Len--;
			pTxBuff++;
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

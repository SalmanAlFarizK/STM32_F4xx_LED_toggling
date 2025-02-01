/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 19, 2024
 *      Author: salman
 */
#include "stm32f407xx_spi_driver.h"

/*
 * Private function declarations
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuff,uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXNE is set
		while(!(pSPIx->SPI_SR & (1 << 0)) );

		//2. Check the DFF bit in CR1
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			*((uint16_t *)pRxBuff) = pSPIx->SPI_DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuff++;
		}
		else
		{
			//8 bit DFF
			*(pRxBuff) = pSPIx->SPI_DR;
			Len--;
			pRxBuff++;
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

/*
 * IRQ Configuration and ISR handlingb
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Pogram ISER0 register (Interrupt set enable 0 register // refer
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber <96)
		{
			/*Program ISER 2 register*/
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			// Pogram ICER0 register (Interrupt clear enable 0 register // refer
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber <96)
		{
			/*Program ICER 2 register*/
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//First lets find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shiftAmount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shiftAmount);
}

/*
 * API to send data through SPI with Interrrupt Mode
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pHandle,uint8_t *pTxBuff,uint32_t Len)
{
	uint8_t state = pHandle->txState;

	if(state != SPI_BSY_IN_TX)
	{
		//1. Save the Tx buffer & Len information in some global variable
		pHandle->pTxBuff = pTxBuff;
		pHandle->txLen = Len;

		//2.Mark the SPI transmission as busy so that no other code can take
		// over same SPI peripheral until transmission is over
		pHandle->txState = SPI_BSY_IN_TX;

		//3.Enable the TXEIE control bit to get interrupt whenever the TXE flag is set in SR
	    pHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);
	}

	//4.Data transmission will be implemented by the ISR code (will implement later)

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pHandle,uint8_t *pRxBuff,uint32_t Len)
{
	uint8_t state = pHandle->rxState;

	if(state != SPI_BSY_IN_RX)
	{
		//1. Save the Rx buffer & Len information in some global variable
		pHandle->pRxBuff = pRxBuff;
		pHandle->rxLen = Len;

		//2.Mark the SPI reception as busy so that no other code can take
		// over same SPI peripheral until reception is over
		pHandle->rxState = SPI_BSY_IN_RX;

		//3.Enable the RXNEIE control bit to get interrupt whenever the RXNE flag is set in SR
		pHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 , temp2;

	//first check for TXE
	temp1 = pHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pHandle);
	}

	//check for OVR flag
	temp1 = pHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//2. Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1.Load the data into data register (DR)
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuff);
		pSPIHandle->txLen--;
		pSPIHandle->txLen--;
		(uint16_t*)pSPIHandle->pTxBuff++;
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuff++);
		pSPIHandle->txLen--;
		pSPIHandle->pTxBuff++;
	}

	if(! pSPIHandle->txLen)
	{
		//tx len is zero, close the spi communication and inform the application that
		//transmission is over
		SPI_CloseTransmission(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//2. Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		*((uint16_t *)pSPIHandle->pRxBuff) = (uint16_t)pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->rxLen--;
		pSPIHandle->rxLen--;
		(uint16_t*)pSPIHandle->pRxBuff++;
	}
	else
	{
		//8 bit DFF
		*(pSPIHandle->pRxBuff) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->rxLen--;
		pSPIHandle->pRxBuff++;
	}

	if(! pSPIHandle->rxLen)
	{
		SPI_CloseReception(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}

void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the OVR flag
	if(pSPIHandle->txLen != SPI_BSY_IN_TX)
	{
		/*
		 * Clearing the OVR bit is done by a read operation on the SPI_DR register followed by a read
		 * access to the SPI_SR register. (Reference manual)
		 */
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;

	}
	(void)temp;
	//2. Inform the application

	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_CleaOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	/*
	 * Clearing the OVR bit is done by a read operation on the SPI_DR register followed by a read
	 * access to the SPI_SR register. (Reference manual)
	 */
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pHandle)
{
	/* This prevents interrupts from setting up of TXE flag */
	pHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);

	/*Reset the buffer*/
	pHandle->pTxBuff = NULL;
	pHandle->txLen = 0;
	pHandle->txState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pHandle)
{
	//Reception is complete
	//lets turn off rxneie interrupt
	pHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);

	/*Reset the buffer*/
	pHandle->pRxBuff = NULL;
	pHandle->rxLen = 0;
	pHandle->rxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent)
{
	//This is a weak implementation. the application may override this function
}

/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jan 21, 2025
 *      Author: sfkfa
 */

#include "stm32f407xx_i2c_driver.h"

#define WRITE 0
#define READ  1

uint16_t AHB_PreScalar[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScalar[8] = {2,4,8,16};


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);
static void I2C_ExecteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr,uint8_t writeOrRead);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);

/*
 * Not implemented: this function is to get the clock value of PLL clock
 */
uint32_t Rcc_GetPllOutputClk()
{
	return  0;
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
        	I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
        	I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
        	I2C3_PCLK_EN();
        }
    }
    else if (EnorDi == DISABLE)
    {
        if (pI2Cx == I2C1)
        {
        	I2C1_PCLK_DI();
        }
        else if (pI2Cx == I2C2)
        {
        	I2C2_PCLK_DI();
        }
        else if (pI2Cx == I2C3)
        {
        	I2C3_PCLK_DI();
        }
    }
}

uint32_t Rcc_GetPCLK1Val(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clkSrc,temp,ahbP,apb1P;

	clkSrc = ((RCC->CFGR >> 2) & 0x3);

	if(clkSrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(clkSrc == 1)
	{
		SystemClk = 8000000;
	}else if(clkSrc == 2)
	{
		SystemClk = Rcc_GetPllOutputClk();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbP = 1;
	}else{
		ahbP = AHB_PreScalar[temp - 8];
	}

	//for apb1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1P = 1;
	}else{
		apb1P = APB1_PreScalar[temp - 4];
	}

	pclk1 = (SystemClk / ahbP) / apb1P;

	return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	//enable the clock for i2c peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t tempReg = 0;

	//ACK Control bit
	tempReg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->I2C_CR1 = tempReg;

	//Configure the FREQ field of CR2
	tempReg = Rcc_GetPCLK1Val()/1000000U;
	pI2CHandle->pI2Cx->I2C_CR2 = (tempReg & 0x3F);

	tempReg = pI2CHandle->I2C_Config.I2C_DeviceAddress;
	pI2CHandle->pI2Cx->I2C_OAR1 = tempReg;

	//CCR calculations
	uint16_t ccr_val = 0;
	tempReg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SPEED_SM)
	{
		// mode is standard mode
		ccr_val = Rcc_GetPCLK1Val()/(2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempReg |= (ccr_val & 0xFFF);
	}else{
		//mode is fast mode
		tempReg |= (1 << 15);
		tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_val = Rcc_GetPCLK1Val()/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else
		{
			ccr_val = Rcc_GetPCLK1Val()/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempReg |= (ccr_val & 0xFFF);
	}
	pI2CHandle->pI2Cx->I2C_CCR = tempReg;

	//TRISE Configuration
	tempReg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SPEED_SM)
	{
		//Mode is Standard
		tempReg = (Rcc_GetPCLK1Val()/1000000U) + 1;
	}else{
		//mode is fast mode
		tempReg = ((Rcc_GetPCLK1Val() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (tempReg & 0x3F);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t * pTxBuff,uint32_t len,uint8_t slaveAddr,uint8_t Sr)
{
	/*
	 * Step 1 : Generate the START condition
	 */
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/*
	 * Step 2: Confirm the start generation is completed by checking the SB flag in SR1
	 * NOTE : Until SB is cleared SCL will be stretched (pulled to LOW)
	 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));


	/*
	 * Step 3: Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	 */
	I2C_ExecteAddressPhase(pI2CHandle->pI2Cx,slaveAddr,WRITE);


	/*
	 * Step 4: Confirm the address phase is completed by checking the ADDR flag in the SR1
	 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));


	/*
	 * Step 5: Clear the ADDR flag according to its software sequence
	 * NOTE : Until ADDR is cleared SCL will be stretched (pulled to low)
	 */
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	/*
	 * Step 6: Send the data until the len become zero
	 */
	while(len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG)); // Wait until TXE is SET

		pI2CHandle->pI2Cx->I2C_DR = *pTxBuff;
		pTxBuff++;
		len--;
	}

	/*
	 * Step 7: When len becomes zero wait for TXE = 1 and BTF = 1 before generating the STOP condition
	 * NOTE: TXE = 1 BTF = 1 means that both SR and DR are empty and  next transmission should begin
	 * When BTF = 1 SCl will be stretched (pulled to low)
	 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));


	/*
	 * Step 8: Generate STOP condition and master need not to wait for the completion of STOP condition.
	 * Generating STOP automatically clears the BTF
	 */
	if(I2C_DISABLE_SR == Sr)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}





void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t * pRxBuff,uint32_t len,uint8_t slaveAddr,uint8_t Sr)
{
	/*
	 * Step 1 : Generate the START condition
	 */
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/*
	 * Step 2: Confirm the start generation is completed by checking the SB flag in SR1
	 * NOTE : Until SB is cleared SCL will be stretched (pulled to LOW)
	 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));


	/*
	 * Step 3: Send the address of the slave with r/w bit set to R(1) (total 8 bits)
	 */
	I2C_ExecteAddressPhase(pI2CHandle->pI2Cx,slaveAddr,READ);


	/*
	 * Step 4: Confirm the address phase is completed by checking the ADDR flag in the SR1
	 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	if(len == 1)
	{
		//Disable ACKing
		I2C_ManageACKing(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG)); // Wait until RXNE is SET

		if(I2C_DISABLE_SR == Sr)
		{
			// generate stop condition
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data into buffer
		*(pRxBuff) = pI2CHandle->pI2Cx->I2C_DR;
	}

	if(len > 1)
	{
		//clear the address flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until the length becomes zero
		for(uint32_t i = len ; i > 0;i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG)); // Wait until RXNE is SET

			if(i == 2)
			{
				//clear the ACK bit
				I2C_ManageACKing(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				if(I2C_DISABLE_SR == Sr)
				{
					// generate stop condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}
			//read the data from data register into the buffer
			*(pRxBuff) = pI2CHandle->pI2Cx->I2C_DR;
			//increment the buffer address
			pRxBuff++;

		}
	}
	// Re enable ACK ing
	I2C_ManageACKing(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName)
{
	if(pI2Cx->I2C_SR1 & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_ExecteAddressPhase(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr,uint8_t writeOrRead)
{
	SlaveAddr = SlaveAddr << 1; // Address is 7 bits long
	if(WRITE == writeOrRead)
	{
		SlaveAddr &= ~(1); // The lsb is R/nW bit which must be zero for WRITE // Slave address is Slave address + r/nw bit = 0
	}
	else if(READ == writeOrRead)
	{
		SlaveAddr |= 1; // The lsb is R/nW bit which must be one for READ // Slave address is Slave address + r/nw bit = 0
	}

	pI2Cx->I2C_DR = SlaveAddr;
}

void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t temp;
	temp = pI2Cx->I2C_SR1;
	temp = pI2Cx->I2C_SR2;
	(void)temp;
}
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_ManageACKing(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
	if(I2C_ACK_DISABLE == EnOrDi)
	{
		//Disable the ACk
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	}else if(I2C_ACK_ENABLE == EnOrDi)
	{
		//Enable the ACK
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	}
}

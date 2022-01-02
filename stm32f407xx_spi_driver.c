/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 15 Dec 2021
 *      Author: mucah
 */
#include "stm32f407xx_spi_driver.h"
#include<stdint.h>

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
		}
		else
		{
			//TODO
		}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 * @return            -
 *
 * @Note              -

 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//configuring SPI_CR1 Register
	uint32_t tempreg = 0;

	SPI_PeriClockControl(pSPIHandle-> pSPIx, ENABLE);

	//configure the DEVICE MODE

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//BUS CONFIGURATION

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
			//Bidi mode should be cleared
			tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
			//Bidi mode should be set
			tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
			//Bidi mode should be cleared
			tempreg &= ~(1 << SPI_CR1_BIDIMODE);
			// RXONLY bit must be set
			tempreg |= (1 << SPI_CR1_RXONLY);

	}

		//CLOCK SPEED
		tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR; // DEFINE MACRO IN LATER

		// DATA FRAME FORMAT
		tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

		//CONFIGURE CPOL (CLOCK POLARITY)
		tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

		// CONFIGURE CPHA
		tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPHA;

		//SOFTWARE SLAVE MANAGEMENT SSM
		tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

		pSPIHandle->pSPIx->CR1 = tempreg;

}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	pSPIx->CR1 = 0;
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName) // SR: Status Register
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{

		// wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET	);

		//Check DFF bit and CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1st. load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++; // next data item
		}else
		{
			//8 bit DFF
			pSPIx -> DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{

		// wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET	);

		//Check DFF bit and CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1st load the from DR to RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++; // next data item
		}else
		{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->DR ;
			Len--;
			pRxBuffer++;
		}

	}
}


/*********************************************************************
 * @fn      		  - SPI_PeripheralClock
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}


}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}

}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
		{
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}else
		{
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save TxBuffer address and length (Len) information in global variables
		pSPIHandle -> pTxBuffer = pTxBuffer;
		pSPIHandle -> TxLen = Len;
		// 2. Mark SPI state as busy in transmission so that no other code can take over same SPI peripheral
		//until the transmission is over
		pSPIHandle -> TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle -> pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code (will implement later)
	}

	return state;

}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
		if(state != SPI_BUSY_IN_RX)
		{
			// 1. Save TxBuffer address and length (Len) information in global variables
			pSPIHandle -> pRxBuffer = pRxBuffer;
			pSPIHandle -> RxLen = Len;
			// 2. Mark SPI state as busy in transmission so that no other code can take over same SPI peripheral
			//until the transmission is over
			pSPIHandle -> RxState = SPI_BUSY_IN_RX;

			//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle -> pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

			//4. Data transmission will be handled by the ISR code (will implement later)
		}

		return state;
}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
		{
			//handle TXE
			spi_rxne_interrupt_handle(pHandle);
		}

		//check for OVR flag
		temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
		temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		// handle OVR Error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

//Assistance functions (implementation)
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check DFF bit and CR1
			if(	(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
			{
				//16 bit DFF
				//1st. load the data into the DR
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++; // next data item
			}else
			{
				//8 bit DFF
				pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++; // next data item
			}

			if(! pSPIHandle ->TxLen)
			{
				//TxLen is zero, so close the SPI comm. and inform the application that the
				//Tx is over
				// this prevent interrupts from setting up the TXE flag
				SPI_CloseTransmission(pSPIHandle);
				SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
			}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << 11))
	{
		// 16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle ->pSPIx->DR;
		pSPIHandle->RxLen -=2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	}else
	{
		// 8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle ->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->pRxBuffer--;
	}
	if(! pSPIHandle->RxLen)
	{
		//TxLen is zero, so close the SPI comm. and inform the application that the
		//Tx is over
		// this prevent interrupts from setting up the TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{

	//A weak implementation that the application may override this function

}





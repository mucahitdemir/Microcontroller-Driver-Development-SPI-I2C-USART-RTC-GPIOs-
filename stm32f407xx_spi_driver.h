/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 15 Dec 2021
 *      Author: mucah
 */
#include "stm32f407xx.h"
#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_


/*
 *  Configuration structure for SPIx peripheral
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
 *Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;   /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
	SPI_Config_t 	SPIConfig;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx length > */
	uint32_t 		RxLen;		/* !< To store Tx length > */
	uint8_t 		TxState;	/* !< To store Tx state > */
	uint8_t 		RxState;	/* !< To store Rx state > */
}SPI_Handle_t;

/*
 * SPI Application states
 */
#define	SPI_READY				0
#define	SPI_BUSY_IN_RX			1
#define	SPI_BUSY_IN_TX			2

/*
 * Possible SPI Application events
 */

#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1 //WORK MOSTLY IN FULL-DUPLEX
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2 			0
#define SPI_SCLK_SPEED_DIV4 			1
#define SPI_SCLK_SPEED_DIV8 			2
#define SPI_SCLK_SPEED_DIV16 			3
#define SPI_SCLK_SPEED_DIV32 			4
#define SPI_SCLK_SPEED_DIV64 			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256 			7

/*
 * @SPI_DFF
 * (DATA FRAME FORMAT)
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @CPOL
 * (CLOCK Polarity)
 */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/*
 * @CPHA
 * CLOCK PHASE
 * 0: first clock transition is the first data capture	edge
 * 1: second clock transition is the first data capture edge
 */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

/*
 * @SSM
 * Slave Software Management
 * Control Register-1, when SSM is set, the NSS pin input is replaced with the value from SSI bit
 */
#define SPI_SSM_EN		1  //HARDWARE
#define SPI_SSM_DI		0  //SOFTWARE


/*
 * SPI related status flag definitions
 */

#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)



/******************************************************************************************
 *								APIs is supported by this driver
 *								Serial Peripheral Clock (SPI)
 ******************************************************************************************/

/*
 * Peripheral Clock Setup, Enable or Disable the base address
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Init & DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send & Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// Interrupt based send & receive data

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */

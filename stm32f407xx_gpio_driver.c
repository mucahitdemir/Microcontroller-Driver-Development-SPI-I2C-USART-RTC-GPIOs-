/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 13 Dec 2021
 *      Author: mucah
 */
#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{

	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 *
 */

// SHOW CANBERK THAT |= OPERATION IS CRITICAL ******

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; // temporary register

	// enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) //mode configuration of GPIO
	{
			//non-interrupt mode, each pin takes 2 bit fields that why multiplied by 2.
		temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle -> pGPIOx -> MODER &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); // clearing 2 bits
		pGPIOHandle -> pGPIOx -> MODER |= temp; // setting
		temp = 0;


	}else
	{
		//this part will be coded later (interrupt mode)
		if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure FTSR
			EXTI -> FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI -> FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure RTSR
			EXTI -> RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI -> RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure both FTSR & RTSR
			EXTI -> RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI -> FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		//2. configure GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle -> pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG -> EXTICR[temp1] = portcode << (temp2 * 4);


		//3 Enable the EXTI interrupt delivery using IMR

		EXTI->IMR |= 1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;

	}
	temp = 0;
	// speed configuration
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> OSPEEDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); // clearing 2 bits
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;

	temp = 0;

	// pull up pull down settings
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx -> PUPDR &= ~(0x3 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); // clearing 2 bits
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;

	temp = 0;

	//output type configuration
	temp = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OTYPER &= ~(0x1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber); // clearing 2 bits
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;

	temp = 0;

	// alternate functionality

	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure alternate function registers since AF[2] either 0 or 1 modes
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle -> pGPIOx -> AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle -> pGPIOx -> AFR[temp1] |= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}




}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

			if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN();
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}else if (pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
		}



/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - reading from input pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  0 or 1
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx -> IDR >> PinNumber) & 0x00000001);

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - reading from input port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx -> IDR;

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes to output pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin
		pGPIOx -> ODR |= (1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx -> ODR &= ~(1 << PinNumber);
	}



}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes to output port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx -> ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_TogglePin
 *
 * @brief             - This function toggles the pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx -> ODR ^= (1 << PinNumber);
}


/*********************************************************************
 * @fn      		  - IRQ Configuration
 *
 * @brief             - This function toggles the pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
				// program ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64) // 32 to 63
		{
				//ISER1 Register programming
			*NVIC_ISER1 |= (1 << (IRQNumber % 32) );

		}else if(IRQNumber >= 64 && (IRQNumber < 96))
		{
				//ISER2 Register programming
			*NVIC_ISER2 |= (1 << (IRQNumber % 32) );

		}
	}else
	{
		if(IRQNumber <= 31)
				{
						// program ICER0 Register
						*NVIC_ICER0 |= (1 << IRQNumber);

				}else if(IRQNumber > 31 && IRQNumber < 64) // 32 to 63
				{
						//ICER1 Register programming
					*NVIC_ICER2 |= (1 << (IRQNumber % 32) );

				}else if(IRQNumber >= 64 && IRQNumber < 96)
				{
						//ICER2 Register programming
					*NVIC_ICER2 |= (1 << (IRQNumber % 32) );

				}
	}
}



/*********************************************************************
 * @fn      		  - IRQ Handling
 *
 * @brief             - This function
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

	// clear the EXTI PR(pending) register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}

}


/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. IPR Register finding
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_selection = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_selection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx * 4) |=  (IRQPriority << shift_amount );

}







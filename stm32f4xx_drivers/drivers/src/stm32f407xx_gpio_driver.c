/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Jun 23, 2025
 *      Author: SUBHANKAR DAS
 */

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */



void GPIO_Peripheralclkcontrol(GPIO_regdef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
		GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB)
		{
		GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC)
				{
				GPIOC_PCLK_EN();
				}
		else if(pGPIOx==GPIOD)
				{
				GPIOD_PCLK_EN();
				}
		else if(pGPIOx==GPIOE)
				{
				GPIOE_PCLK_EN();
				}
		else if(pGPIOx==GPIOF)
				{
				GPIOF_PCLK_EN();
				}
		else if(pGPIOx==GPIOG)
				{
				GPIOG_PCLK_EN();
				}
		else if(pGPIOx==GPIOH)
				{
				GPIOH_PCLK_EN();
				}
		else if(pGPIOx==GPIOI)
				{
				GPIOI_PCLK_EN();
				}
		}
	else
	{
		if(pGPIOx==GPIOA)
				{
				GPIOA_PCLK_DI();
				}
				else if(pGPIOx==GPIOB)
				{
				GPIOB_PCLK_DI();
				}
				else if(pGPIOx==GPIOC)
						{
						GPIOC_PCLK_DI();
						}
				else if(pGPIOx==GPIOD)
						{
						GPIOD_PCLK_DI();
						}
				else if(pGPIOx==GPIOE)
						{
						GPIOE_PCLK_DI();
						}
				else if(pGPIOx==GPIOF)
						{
						GPIOF_PCLK_DI();
						}
				else if(pGPIOx==GPIOG)
						{
						GPIOG_PCLK_DI();
						}
				else if(pGPIOx==GPIOH)
						{
						GPIOH_PCLK_DI();
						}
				else if(pGPIOx==GPIOI)
						{
						GPIOI_PCLK_DI();
						}

	}

}

void GPIO_init(gpio_handle_t *pGPIOHandle){
	uint32_t temp=0;
	if(pGPIOHandle->gpio_pinconfig.GPIO_PinMode<=GPIO_MODE_ANALOG)           /*less than 3 then non interrupt mode,more than 3 is interrupt mode */
	{
		temp = (pGPIOHandle->gpio_pinconfig.GPIO_PinMode << (2 * pGPIOHandle->gpio_pinconfig.GPIO_PinNumber ) ); /*each pin has two bit positions*/
		pGPIOHandle->pGPIOx->MODER=temp;
		temp=0;
	}
	else
	{

	}
	//configuring the speed
	temp=(pGPIOHandle->gpio_pinconfig.GPIO_PinSpeed << (2 * pGPIOHandle->gpio_pinconfig.GPIO_PinNumber ) );
	pGPIOHandle->pGPIOx->OSPEEDR=temp;
	temp=0;

	//configure the pupd settings
	temp=(pGPIOHandle->gpio_pinconfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->gpio_pinconfig.GPIO_PinNumber ) );
	pGPIOHandle->pGPIOx->PUPDR=temp;
	temp=0;

	//configure the optype
	temp=(pGPIOHandle->gpio_pinconfig.GPIO_PinOPType << (2 * pGPIOHandle->gpio_pinconfig.GPIO_PinNumber ) );
	pGPIOHandle->pGPIOx->OTYPER=temp;
	temp=0;

	//configure alternate functionality
	temp=(pGPIOHandle->gpio_pinconfig.GPIO_PinAltFunMode << (2 * pGPIOHandle->gpio_pinconfig.GPIO_PinNumber ) );
	pGPIOHandle->pGPIOx->AFR[2]=temp;
	temp=0;


}

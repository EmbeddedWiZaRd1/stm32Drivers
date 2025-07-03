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
/*********************************************************************
 * @fn      		  - GPIO_Init
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
/* x<<y means x=value shifted to the left by y=position*/

void GPIO_init(gpio_handle_t *pGPIOHandle){
	uint32_t temp=0;
	if(pGPIOHandle->gpio_pinconfig.GPIO_PinMode==GPIO_MODE_ANALOG)           /*less than 3 then non interrupt mode,more than 3 is interrupt mode */
	{
		temp = (pGPIOHandle->gpio_pinconfig.GPIO_PinMode << (2 * pGPIOHandle->gpio_pinconfig.GPIO_PinNumber ) ); /*each pin has two bit positions*/
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<(2* pGPIOHandle->gpio_pinconfig.GPIO_PinNumber)); //clearing the bit fields before setting |11 so 0x3 in hex
		pGPIOHandle->pGPIOx->MODER |=temp;           //temp holds the masked value then placing that value in the actual mode_r register
		temp=0;
	}
	else
	{
		if(pGPIOHandle->gpio_pinconfig.GPIO_PinMode==GPIO_MODE_IT_FT)
		{
           EXTI->EXTI_FTSR |=(1<<pGPIOHandle->gpio_pinconfig.GPIO_PinNumber);
           EXTI->EXTI_RTSR &=~(1<<pGPIOHandle->gpio_pinconfig.GPIO_PinNumber); //RTSR PIN IS CLEARED
		}
		else if(pGPIOHandle->gpio_pinconfig.GPIO_PinMode==GPIO_MODE_IT_FT)
				{
			 EXTI->EXTI_RTSR |=(1<<pGPIOHandle->gpio_pinconfig.GPIO_PinNumber);
			 EXTI->EXTI_FTSR &=~(1<<pGPIOHandle->gpio_pinconfig.GPIO_PinNumber); //FTSR PIN IS CLEARED
				}
		else if(pGPIOHandle->gpio_pinconfig.GPIO_PinMode==GPIO_MODE_IT_FT)
				{
			 EXTI->EXTI_RTSR |=(1<<pGPIOHandle->gpio_pinconfig.GPIO_PinNumber);
			 EXTI->EXTI_FTSR |=(1<<pGPIOHandle->gpio_pinconfig.GPIO_PinNumber);
				}

		//configure gpio port selection in syscfg register
		uint8_t temp1=(1<<pGPIOHandle->gpio_pinconfig.GPIO_PinNumber)/4;
		uint8_t temp2=(1<<pGPIOHandle->gpio_pinconfig.GPIO_PinNumber)%4;
		uint8_t portcode=GPIO_BASE_ADD_TO_CODE(pGPIOHandle->pGPIOx);  //macro will return the port code like 1 for GPIOA,2 for GPIOB etc
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]=portcode<<(temp2*4);



		//CONFIGURE THE EXTI INTERUPT DELIVERY USING IMR
		 EXTI->EXTI_IMR |=(1<<pGPIOHandle->gpio_pinconfig.GPIO_PinNumber);

	}

	temp=0;
	//configuring the speed
	temp=(pGPIOHandle->gpio_pinconfig.GPIO_PinSpeed << (2 * pGPIOHandle->gpio_pinconfig.GPIO_PinNumber ) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<(2* pGPIOHandle->gpio_pinconfig.GPIO_PinNumber)); //clearing the bit positions before setting
	pGPIOHandle->pGPIOx->OSPEEDR |=temp; //when setting the value of physical register use bitwise or instead of assignment operator or else other bit positions will get disturbed
	temp=0;

	//configure the pupd settings
	temp=(pGPIOHandle->gpio_pinconfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->gpio_pinconfig.GPIO_PinNumber ) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 <<(2* pGPIOHandle->gpio_pinconfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |=temp;
	temp=0;

	//configure the optype
	temp=(pGPIOHandle->gpio_pinconfig.GPIO_PinOPType <<  pGPIOHandle->gpio_pinconfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->gpio_pinconfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |=temp;
	temp=0;

	//configure alternate functionality
	if(pGPIOHandle->gpio_pinconfig.GPIO_PinAltFunMode==GPIO_MODE_ALTFN)
	{
		/*alternate functionality touched only if alt functionality mode is enabled*/
	uint32_t temp1,temp2;
	temp1=((pGPIOHandle->gpio_pinconfig.GPIO_PinNumber)/8);
    temp2=((pGPIOHandle->gpio_pinconfig.GPIO_PinNumber)%8);
    pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
    pGPIOHandle->pGPIOx->AFR[temp1] |=( pGPIOHandle->gpio_pinconfig.GPIO_PinAltFunMode <<( 4*temp2));
	}


}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
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
void GPIO_deinit(gpio_handle_t *pGPIOx)
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
			GPIOF_REG_RESET();
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
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -

 */

uint8_t GPIO_ReadfromInputpin(GPIO_regdef_t *pGPIOx,uint8_t Pinumber){
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR >>Pinumber)& 0x00000001);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
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
uint16_t GPIO_ReadFromInputPort(GPIO_regdef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
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
void GPIO_WriteToOutputPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
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
void GPIO_WriteToOutputPort(GPIO_regdef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR  = Value;
}
/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
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
void GPIO_ToggleOutputPin(GPIO_regdef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber); /*Bitwise xor operation to toggle the required pin number*/
}

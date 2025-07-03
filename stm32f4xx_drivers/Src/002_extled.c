/*
 * 002_extled.c
 *
 *  Created on: Jul 3, 2025
 *      Author: SUBHANKAR DAS
 */


#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i=0;i<500000/2;i++)
		;
}

int main(void)
{
	gpio_handle_t GPIObutton;
	gpio_handle_t Gpioled;
	Gpioled.pGPIOx=GPIOA;
	Gpioled.gpio_pinconfig.GPIO_PinNumber=GPIO_PIN_NO_8;
	Gpioled.gpio_pinconfig.GPIO_PinMode=GPIO_MODE_OUT;
	Gpioled.gpio_pinconfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	Gpioled.gpio_pinconfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	Gpioled.gpio_pinconfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIObutton.pGPIOx=GPIOB;
	GPIObutton.gpio_pinconfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIObutton.gpio_pinconfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIObutton.gpio_pinconfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIObutton.gpio_pinconfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_Peripheralclkcontrol(GPIOB,ENABLE);
	GPIO_Peripheralclkcontrol(GPIOA,ENABLE);

	GPIO_init(&GPIObutton);
	GPIO_init(&Gpioled);

	while(1)
	{
		if(GPIO_ReadfromInputpin(GPIOB,GPIO_PIN_NO_12)==0)
		{
	    delay();
	    GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);
		}
	}
	return 0;
	}

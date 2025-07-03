/*
 * 001ledtoggle.c
 *
 *  Created on: Jul 2, 2025
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
	Gpioled.pGPIOx=GPIOD;
	Gpioled.gpio_pinconfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	Gpioled.gpio_pinconfig.GPIO_PinMode=GPIO_MODE_OUT;
	Gpioled.gpio_pinconfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	Gpioled.gpio_pinconfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	Gpioled.gpio_pinconfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIObutton.pGPIOx=GPIOA;
	GPIObutton.gpio_pinconfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GPIObutton.gpio_pinconfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIObutton.gpio_pinconfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIObutton.gpio_pinconfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_Peripheralclkcontrol(GPIOD,ENABLE);
	GPIO_Peripheralclkcontrol(GPIOA,ENABLE);

	GPIO_init(&GPIObutton);
	GPIO_init(&Gpioled);

	while(1)
	{
		if(GPIO_ReadfromInputpin(GPIOA,GPIO_PIN_NO_0)==1)
		{
	    delay();
	    GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		}
	}
	return 0;
	}

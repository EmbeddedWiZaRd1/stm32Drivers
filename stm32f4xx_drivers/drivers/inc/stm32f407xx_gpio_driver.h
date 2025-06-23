#ifndef INC_STM32F407XX_GPIO_DRV_H
#define INC_STM32F407XX_GPIO_DRV_H

#include "stm32f407xx.h"   /*MCU SPECIFIC HEADER FILE*/


//This header file will contain GPIOX Driver specific data
typedef struct{
	uint8_t GPIO_PinNumber;
		uint8_t GPIO_PinMode;
		uint8_t GPIO_PinSpeed;
		uint8_t GPIO_PinPuPdControl;
		uint8_t GPIO_PinOPType;
		uint8_t GPIO_PinAltFunMode;
}GPIO_pinconfig_t;
typedef struct
{
	GPIO_regdef_t *pGPIOx;  /*this holds the base adress of gpio port to which the pin belongs*/
	GPIO_pinconfig_t gpio_pinconfig;
}gpio_handle_t;
//peripheral clock setup
void GPIO_Peripheralclkcontrol(void);

//init and deint gpio
void GPIO_init(void);
void GPIO_deinit(void);

//Data read write
void GPIO_ReadfromInputpin(void);
void GPIO_ReadfromInputport(void);
void GPIO_WritetoOutputpin(void);
void GPIO_WritetoOutputport(void);
void GPIO_ToggleOutputpin(void);

//Isr handling
void IRQ_Config(void);
void IRD_Handling(void);

#endif

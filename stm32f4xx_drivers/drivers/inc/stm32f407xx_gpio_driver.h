#ifndef INC_STM32F407XX_GPIO_DRV_H
#define INC_STM32F407XX_GPIO_DRV_H

#include "stm32f407xx.h"   /*MCU SPECIFIC HEADER FILE*/


//This header file will contain GPIOX Driver specific data
typedef struct{
	uint8_t GPIO_PinNumber;            /*!< possible values of @GPIO_PIN_NUM >!*/
		uint8_t GPIO_PinMode;          /*!< possible values of @GPIO_PIN_MODES >!*/
		uint8_t GPIO_PinSpeed; 		  /*!< possible values of @GPIO_OUT_TYP >!*/
		uint8_t GPIO_PinPuPdControl;
		uint8_t GPIO_PinOPType;
		uint8_t GPIO_PinAltFunMode;
}GPIO_pinconfig_t;
typedef struct
{
	GPIO_regdef_t *pGPIOx;  /*this holds the base address of gpio port to which the pin belongs*/
	GPIO_pinconfig_t gpio_pinconfig;
}gpio_handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0					/*  0,1,2,3 ARE non-INTERRUPT modes*/
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4                    /* interrupt falling edge trigger 4,5,6 ARE INTERRUPT modes*/
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6



/*
 * @@GPIO_OUT_TYP
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3

/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2



//peripheral clock setup
void GPIO_Peripheralclkcontrol(GPIO_regdef_t *pGPIOx,uint8_t EnorDi);

//init and deint gpio
void GPIO_init(gpio_handle_t *pGPIOHandle);
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

/*
 * stm32f407xx.h
 *
 *  Created on: Jun 19, 2025
 *      Author: SUBHANKAR DAS
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo               volatile

#define ENABLE  1
#define DISABLE 0
#define SET 	ENABLE
#define RESET   DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
/*
 we will update the base adress of flash sram */

#define FLASH_BASEADDR          0x08000000U                  //COMPILER INFORMED ABOUT UNSIGNED VALUE
#define SRAM1_BASEADDR			0x20000000U
#define SRAM            		SRAM1_BASEADDR
#define SRAM2_BASEADDR			0x2001C000U
#define SRAM3_BASEADDR			0x20020000U
#define ROM 					0x1FFF0000U


//dummy change in main
/* PERIPHERAL BUS ADRESS OF DIFFERENT BUS DOMAIN-APB1,APB2,AHB1,AHB2*/

#define PERIPHERAL_BASEADDR 				0X40000000U
#define ABP1PERIPHERAL_BASEADDR				PERIPHERAL_BASEADDR
#define ABP2PERIPHERAL_BASEADDR				0X40010000U
#define AHB1PERIPHERAL_BASEADDR				0X40020000U
#define AHB2PERIPHERAL_BASEADDR				0X50000000U

/*DEFINING THE BASE ADRESS OF ALL THE PERIPHERALS HANGING ON THE AHB1BUS(GPIO'S ARE HANGNING ON THE AHB1 BUS)*/
#define GPIOA_BASEADDR      			    (AHB1PERIPHERAL_BASEADDR+0X0000)
#define GPIOB_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X0400)
#define GPIOC_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X0800)
#define GPIOD_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X0C00)
#define GPIOE_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X1000)
#define GPIOF_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X1400)
#define GPIOG_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X1800)
#define GPIOH_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X1C00)
#define GPIOI_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X2000)
#define GPIOJ_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X2400)
#define GPIOK_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X2800)
#define RCC_BASEADDR						(AHB1PERIPHERAL_BASEADDR+0X3800)

/*DEFINING THE BASE ADRESS OF ALL THE PERIPHERALS HANGING ON THE APB1BUS*/
#define I2C1_BASEADDR						(ABP1PERIPHERAL_BASEADDR+0X5400)
#define I2C2_BASEADDR						(ABP1PERIPHERAL_BASEADDR+0X5800)
#define I2C3_BASEADDR						(ABP1PERIPHERAL_BASEADDR+0X5C00)
#define SPI2_BASEADDR						(ABP1PERIPHERAL_BASEADDR+0X3800)
#define SPI3_BASEADDR						(ABP1PERIPHERAL_BASEADDR+0X3C00)
#define USART2_BASEADDR						(ABP1PERIPHERAL_BASEADDR+0X4400)
#define USART3_BASEADDR						(ABP1PERIPHERAL_BASEADDR+0X4800)
#define UART4_BASEADDR						(ABP1PERIPHERAL_BASEADDR+0X4C00)
#define UART5_BASEADDR						(ABP1PERIPHERAL_BASEADDR+0X5000)

/*DEFINING THE BASE ADRESS OF ALL THE PERIPHERALS HANGING ON THE APB2BUS*/
#define SPI1_BASEADDR						(ABP2PERIPHERAL_BASEADDR+0X3000)
#define USART1_BASEADDR						(ABP2PERIPHERAL_BASEADDR+0X1000)
#define USART6_BASEADDR						(ABP2PERIPHERAL_BASEADDR+0X1400)
#define EXTI_BASEADDR						(ABP2PERIPHERAL_BASEADDR+0X3C00)
#define SYSCFG_BASEADDR						(ABP2PERIPHERAL_BASEADDR+0X5000)


/*structure definition for GPIO*/

typedef struct{
	__vo uint32_t MODER;              /*GPIO port mode register                                                   Address offset-0x00*/
	__vo uint32_t OTYPER;			/*GPIO port output type register											Address offset-0x04*/
	__vo uint32_t OSPEEDR;           /*GPIO port output speed register											Address offset-0x08*/
	__vo uint32_t PUPDR;             /*GPIO port pull-up/pull-down register										Address offset-0x0C*/
	__vo uint32_t IDR;				/*GPIO port input data register												Address offset-0x10*/
	__vo uint32_t ODR;				/*GPIO port output data register*/
	__vo uint32_t BSRR;				/*GPIO port bit set/reset register*/
	__vo uint32_t LCKR;				/*GPIO port configuration lock register*/
	__vo uint32_t AFR[2];			/*GPIO alternate function low register AFR[0]-GPIO alternate function low register AFR[1]-GPIO alternate function high register*/
}GPIO_regdef_t;

typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;

/*PERIPHERAL base address type casted to xxx_regdef_t type*/

#define GPIOA                      ((GPIO_regdef_t*)GPIOA_BASEADDR)
#define GPIOB                      ((GPIO_regdef_t*)GPIOB_BASEADDR)
#define GPIOC                      ((GPIO_regdef_t*)GPIOC_BASEADDR)
#define GPIOD                      ((GPIO_regdef_t*)GPIOD_BASEADDR)
#define GPIOE                      ((GPIO_regdef_t*)GPIOE_BASEADDR)
#define GPIOF                      ((GPIO_regdef_t*)GPIOF_BASEADDR)
#define GPIOG                      ((GPIO_regdef_t*)GPIOG_BASEADDR)
#define GPIOH                      ((GPIO_regdef_t*)GPIOH_BASEADDR)
#define GPIOI                      ((GPIO_regdef_t*)GPIOI_BASEADDR)
#define GPIOJ                      ((GPIO_regdef_t*)GPIOJ_BASEADDR)
#define GPIOK                      ((GPIO_regdef_t*)GPIOK_BASEADDR)
#define RCC 					    ((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripheralsbu
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()           RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_PCLK_DI()           RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_PCLK_DI()          RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_PCLK_DI()          RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_PCLK_DI()          RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_PCLK_DI()          RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_PCLK_DI()          RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_PCLK_DI()          RCC->AHB1ENR &= ~(1 << 7)
#define GPIOI_PCLK_DI()          RCC->AHB1ENR &= ~(1 << 8)

#endif /* INC_STM32F407XX_H_ */

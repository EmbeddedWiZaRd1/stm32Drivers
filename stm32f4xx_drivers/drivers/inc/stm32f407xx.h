/*
 * stm32f407xx.h
 *
 *  Created on: Jun 19, 2025
 *      Author: SUBHANKAR DAS
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 we will update the base adress of flash sram */

#define FLASH_BASEADDR          0x08000000U                  //COMPILER INFORMED ABOUT UNSIGNED VALUE
#define SRAM1_BASEADDR			0x20000000U
#define SRAM            		SRAM1_BASEADDR
#define SRAM2_BASEADDR			0x2001C000U
#define SRAM3_BASEADDR			0x20020000U
#define ROM 					0x1FFF0000U


/* PERIPHERAL BUS ADRESS OF DIFFERENT BUS DOMAIN-APB1,APB2,AHB1,AHB2*/

#endif /* INC_STM32F407XX_H_ */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union _std_union{
	uint16_t 	istd;
	uint8_t 	cstd[2];
}std_union;

typedef union _l_std_union{
	uint32_t	listd;
	uint16_t 	istd[2];
	uint8_t 	cstd[4];
}l_std_union;

typedef union _l_l_std_union{
	uint64_t	llistd;
	uint32_t	listd[2];
	uint16_t 	istd[4];
	uint8_t 	cstd[8];
}l_l_std_union;

extern uint8_t 						USB_TX_MASS[1000];
extern uint8_t 						USB_RX_MASS[1000];

extern uint8_t						usb_parcel_counter;
extern uint8_t						usb_parcel_mode;

extern std_union					len;
extern std_union					temp;
extern l_std_union				maps_data;
extern l_std_union				pe_data;
extern uint8_t						pe_opt;
extern uint8_t						maps_mode;
extern uint8_t						pe_mode;
extern uint8_t						pe_opt;

extern float							maps_phase;
extern float							pe_phase;

extern double 						maps_steps[6];
extern double 						pe_steps[8];

extern int 								mode;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void usb_irq_parser(void);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
extern uint8_t xor_handler(uint8_t *mass);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC13_LED_Pin GPIO_PIN_13
#define PC13_LED_GPIO_Port GPIOC
#define CTRL_Trigger_Pin GPIO_PIN_0
#define CTRL_Trigger_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/*************************** USB PROTOCOL **********************************/

//SERVICE BYTES
#define SYNCHRO									0x02
#define	DEV_ADDRESS							0x0A

//FUNCTIONS
#define ECHO										0x00
#define BO_CTRL									0x01
#define	MAPS_DATA								0x02
#define PE_P_S									0x03
#define	PE_DATA									0x04
#define PE_OPT									0x05
#define I2C_REQUEST							0x06

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

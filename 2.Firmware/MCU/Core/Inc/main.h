/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FANTIMERARR (6800)
#define RLY_A_Pin LL_GPIO_PIN_13
#define RLY_A_GPIO_Port GPIOC
#define RLY_B_Pin LL_GPIO_PIN_14
#define RLY_B_GPIO_Port GPIOC
#define RLY_C_Pin LL_GPIO_PIN_15
#define RLY_C_GPIO_Port GPIOC
#define INRUSH_RELAY_Pin LL_GPIO_PIN_1
#define INRUSH_RELAY_GPIO_Port GPIOF
#define VgridA_Pin LL_GPIO_PIN_0
#define VgridA_GPIO_Port GPIOC
#define VgridB_Pin LL_GPIO_PIN_1
#define VgridB_GPIO_Port GPIOC
#define VgridC_Pin LL_GPIO_PIN_2
#define VgridC_GPIO_Port GPIOC
#define LED_statue_Pin LL_GPIO_PIN_3
#define LED_statue_GPIO_Port GPIOC
#define IA__Pin LL_GPIO_PIN_0
#define IA__GPIO_Port GPIOA
#define IB__Pin LL_GPIO_PIN_1
#define IB__GPIO_Port GPIOA
#define IC__Pin LL_GPIO_PIN_2
#define IC__GPIO_Port GPIOA
#define Idcbus_Pin LL_GPIO_PIN_3
#define Idcbus_GPIO_Port GPIOA
#define TRACE_Pin LL_GPIO_PIN_6
#define TRACE_GPIO_Port GPIOA
#define Vbus_up_Pin LL_GPIO_PIN_7
#define Vbus_up_GPIO_Port GPIOA
#define Vbus_down_Pin LL_GPIO_PIN_4
#define Vbus_down_GPIO_Port GPIOC
#define EN_GATE_Pin LL_GPIO_PIN_5
#define EN_GATE_GPIO_Port GPIOC
#define ILA_Pin LL_GPIO_PIN_0
#define ILA_GPIO_Port GPIOB
#define ILB_Pin LL_GPIO_PIN_1
#define ILB_GPIO_Port GPIOB
#define LED_B_Pin LL_GPIO_PIN_2
#define LED_B_GPIO_Port GPIOB
#define ILC_Pin LL_GPIO_PIN_13
#define ILC_GPIO_Port GPIOB
#define FAN_PWM_Pin LL_GPIO_PIN_12
#define FAN_PWM_GPIO_Port GPIOC
#define LED_G_Pin LL_GPIO_PIN_2
#define LED_G_GPIO_Port GPIOD
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

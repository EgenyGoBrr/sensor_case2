/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SHT31_DEFAULT_ADDR          (0x44 << 1)
#define SHT31_MEAS_HIGHREP_STRETCH  0x2C06
#define SHT31_MEAS_MEDREP_STRETCH   0x2C0D
#define SHT31_MEAS_LOWREP_STRETCH   0x2C10
#define SHT31_MEAS_HIGHREP          0x2400
#define SHT31_MEAS_MEDREP           0x240B
#define SHT31_MEAS_LOWREP           0x2416
#define SHT31_READSTATUS            0xF32D
#define SHT31_CLEARSTATUS           0x3041
#define SHT31_SOFTRESET             0x30A2
#define SHT31_HEATEREN              0x306D
#define SHT31_HEATERDIS             0x3066
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

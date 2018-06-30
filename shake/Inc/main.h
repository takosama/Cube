/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define swich2_Pin GPIO_PIN_0
#define swich2_GPIO_Port GPIOC
#define swich4_Pin GPIO_PIN_1
#define swich4_GPIO_Port GPIOC
#define swich3_Pin GPIO_PIN_2
#define swich3_GPIO_Port GPIOC
#define swich1_Pin GPIO_PIN_3
#define swich1_GPIO_Port GPIOC
#define swich8_Pin GPIO_PIN_0
#define swich8_GPIO_Port GPIOA
#define swich7_Pin GPIO_PIN_1
#define swich7_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define swich6_Pin GPIO_PIN_4
#define swich6_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define motor4_pwm_Pin GPIO_PIN_6
#define motor4_pwm_GPIO_Port GPIOA
#define motor5_pwm_Pin GPIO_PIN_7
#define motor5_pwm_GPIO_Port GPIOA
#define Reset_Pin GPIO_PIN_4
#define Reset_GPIO_Port GPIOC
#define motor3_DIR_Pin GPIO_PIN_5
#define motor3_DIR_GPIO_Port GPIOC
#define swich5_Pin GPIO_PIN_0
#define swich5_GPIO_Port GPIOB
#define R3_B_Pin GPIO_PIN_1
#define R3_B_GPIO_Port GPIOB
#define R2_B_Pin GPIO_PIN_2
#define R2_B_GPIO_Port GPIOB
#define R3_A_Pin GPIO_PIN_10
#define R3_A_GPIO_Port GPIOB
#define R1_B_Pin GPIO_PIN_12
#define R1_B_GPIO_Port GPIOB
#define limit_S3_Pin GPIO_PIN_13
#define limit_S3_GPIO_Port GPIOB
#define limit_S1_Pin GPIO_PIN_14
#define limit_S1_GPIO_Port GPIOB
#define R4_B_Pin GPIO_PIN_15
#define R4_B_GPIO_Port GPIOB
#define motor2_pwm_Pin GPIO_PIN_6
#define motor2_pwm_GPIO_Port GPIOC
#define LED_PC7_Pin GPIO_PIN_7
#define LED_PC7_GPIO_Port GPIOC
#define motor1_pwm_Pin GPIO_PIN_8
#define motor1_pwm_GPIO_Port GPIOC
#define motor1_DIR_Pin GPIO_PIN_9
#define motor1_DIR_GPIO_Port GPIOC
#define R2_A_Pin GPIO_PIN_8
#define R2_A_GPIO_Port GPIOA
#define limit_S4_Pin GPIO_PIN_10
#define limit_S4_GPIO_Port GPIOA
#define motor5_DIR_Pin GPIO_PIN_11
#define motor5_DIR_GPIO_Port GPIOA
#define motor4_DIR_Pin GPIO_PIN_12
#define motor4_DIR_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define swich10_Pin GPIO_PIN_15
#define swich10_GPIO_Port GPIOA
#define LED_PC10_Pin GPIO_PIN_10
#define LED_PC10_GPIO_Port GPIOC
#define LED_PC11_Pin GPIO_PIN_11
#define LED_PC11_GPIO_Port GPIOC
#define swich11_Pin GPIO_PIN_12
#define swich11_GPIO_Port GPIOC
#define LED_PD2_Pin GPIO_PIN_2
#define LED_PD2_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define R4_A_Pin GPIO_PIN_4
#define R4_A_GPIO_Port GPIOB
#define limit_S2_Pin GPIO_PIN_5
#define limit_S2_GPIO_Port GPIOB
#define Servo1_Pin GPIO_PIN_6
#define Servo1_GPIO_Port GPIOB
#define swich9_Pin GPIO_PIN_7
#define swich9_GPIO_Port GPIOB
#define motor2_DIR_Pin GPIO_PIN_8
#define motor2_DIR_GPIO_Port GPIOB
#define motor3_pwm_Pin GPIO_PIN_9
#define motor3_pwm_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

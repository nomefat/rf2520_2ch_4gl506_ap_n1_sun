/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SPI3_rf_power_onoff_Pin GPIO_PIN_13
#define SPI3_rf_power_onoff_GPIO_Port GPIOC
#define SPI5_rf_io_reset_Pin GPIO_PIN_0
#define SPI5_rf_io_reset_GPIO_Port GPIOF
#define SPI5_rf_io_vreg_Pin GPIO_PIN_1
#define SPI5_rf_io_vreg_GPIO_Port GPIOF
#define SPI5_rf_tx_Pin GPIO_PIN_2
#define SPI5_rf_tx_GPIO_Port GPIOF
#define SPI5_rf_rfoff_Pin GPIO_PIN_3
#define SPI5_rf_rfoff_GPIO_Port GPIOF
#define SPI5_rf_io_fifop_Pin GPIO_PIN_4
#define SPI5_rf_io_fifop_GPIO_Port GPIOF
#define SPI5_rf_io_fifop_EXTI_IRQn EXTI4_IRQn
#define SPI5_rf_io_fifo_Pin GPIO_PIN_5
#define SPI5_rf_io_fifo_GPIO_Port GPIOF
#define SPI5_rf_cs_Pin GPIO_PIN_6
#define SPI5_rf_cs_GPIO_Port GPIOF
#define SPI5_rf_power_onoff_Pin GPIO_PIN_10
#define SPI5_rf_power_onoff_GPIO_Port GPIOF
#define SPI1_rf_cs_Pin GPIO_PIN_4
#define SPI1_rf_cs_GPIO_Port GPIOA
#define SPI1_rf_io_vreg_Pin GPIO_PIN_4
#define SPI1_rf_io_vreg_GPIO_Port GPIOC
#define SPI1_rf_io_reset_Pin GPIO_PIN_5
#define SPI1_rf_io_reset_GPIO_Port GPIOC
#define SPI1_rf_tx_Pin GPIO_PIN_0
#define SPI1_rf_tx_GPIO_Port GPIOB
#define SPI1_rf_rfoff_Pin GPIO_PIN_1
#define SPI1_rf_rfoff_GPIO_Port GPIOB
#define SPI1_rf_io_fifo_Pin GPIO_PIN_11
#define SPI1_rf_io_fifo_GPIO_Port GPIOF
#define SPI1_rf_io_fifop_Pin GPIO_PIN_12
#define SPI1_rf_io_fifop_GPIO_Port GPIOF
#define SPI1_rf_io_fifop_EXTI_IRQn EXTI15_10_IRQn
#define SPI1_rf_power_onoff_Pin GPIO_PIN_13
#define SPI1_rf_power_onoff_GPIO_Port GPIOF
#define Watch_dog_io_Pin GPIO_PIN_9
#define Watch_dog_io_GPIO_Port GPIOE
#define SPI4_rf_cs_Pin GPIO_PIN_11
#define SPI4_rf_cs_GPIO_Port GPIOE
#define SPI4_rf_io_vreg_Pin GPIO_PIN_15
#define SPI4_rf_io_vreg_GPIO_Port GPIOE
#define SPI4_rf_io_fifop_Pin GPIO_PIN_10
#define SPI4_rf_io_fifop_GPIO_Port GPIOB
#define SPI4_rf_io_fifop_EXTI_IRQn EXTI15_10_IRQn
#define SPI4_rf_io_fifo_Pin GPIO_PIN_11
#define SPI4_rf_io_fifo_GPIO_Port GPIOB
#define SPI4_rf_io_reset_Pin GPIO_PIN_6
#define SPI4_rf_io_reset_GPIO_Port GPIOH
#define SPI4_rf_tx_Pin GPIO_PIN_7
#define SPI4_rf_tx_GPIO_Port GPIOH
#define SPI4_rf_rfoff_Pin GPIO_PIN_8
#define SPI4_rf_rfoff_GPIO_Port GPIOH
#define SPI4_rf_power_onoff_Pin GPIO_PIN_9
#define SPI4_rf_power_onoff_GPIO_Port GPIOH
#define G4_33TO18_ENABLE_Pin GPIO_PIN_15
#define G4_33TO18_ENABLE_GPIO_Port GPIOB
#define GPRS_pwoer_onoff_Pin GPIO_PIN_10
#define GPRS_pwoer_onoff_GPIO_Port GPIOD
#define general_led_1_Pin GPIO_PIN_2
#define general_led_1_GPIO_Port GPIOG
#define general_led_2_Pin GPIO_PIN_3
#define general_led_2_GPIO_Port GPIOG
#define general_led_3_Pin GPIO_PIN_4
#define general_led_3_GPIO_Port GPIOG
#define general_led_4_Pin GPIO_PIN_5
#define general_led_4_GPIO_Port GPIOG
#define general_led_5_Pin GPIO_PIN_6
#define general_led_5_GPIO_Port GPIOG
#define general_led_6_Pin GPIO_PIN_7
#define general_led_6_GPIO_Port GPIOG
#define SPI3_rf_io_vreg_Pin GPIO_PIN_13
#define SPI3_rf_io_vreg_GPIO_Port GPIOH
#define SPI3_rf_io_reset_Pin GPIO_PIN_14
#define SPI3_rf_io_reset_GPIO_Port GPIOH
#define SPI3_rf_tx_Pin GPIO_PIN_0
#define SPI3_rf_tx_GPIO_Port GPIOI
#define SPI3_rf_rfoff_Pin GPIO_PIN_1
#define SPI3_rf_rfoff_GPIO_Port GPIOI
#define SPI3_rf_io_fifop_Pin GPIO_PIN_2
#define SPI3_rf_io_fifop_GPIO_Port GPIOI
#define SPI3_rf_io_fifop_EXTI_IRQn EXTI2_IRQn
#define SPI3_rf_io_fifo_Pin GPIO_PIN_3
#define SPI3_rf_io_fifo_GPIO_Port GPIOI
#define SPI3_rf_cs_Pin GPIO_PIN_15
#define SPI3_rf_cs_GPIO_Port GPIOA
#define GPRS_powerkey_Pin GPIO_PIN_1
#define GPRS_powerkey_GPIO_Port GPIOD
#define SPI4_led_crc_error_Pin GPIO_PIN_7
#define SPI4_led_crc_error_GPIO_Port GPIOD
#define SPI1_led_crc_error_Pin GPIO_PIN_10
#define SPI1_led_crc_error_GPIO_Port GPIOG
#define SPI1_led_rf_send_Pin GPIO_PIN_11
#define SPI1_led_rf_send_GPIO_Port GPIOG
#define SPI5_led_crc_error_Pin GPIO_PIN_12
#define SPI5_led_crc_error_GPIO_Port GPIOG
#define SPI5_led_rf_send_Pin GPIO_PIN_13
#define SPI5_led_rf_send_GPIO_Port GPIOG
#define G4_on_off_Pin GPIO_PIN_3
#define G4_on_off_GPIO_Port GPIOB
#define G4_reset_Pin GPIO_PIN_4
#define G4_reset_GPIO_Port GPIOB
#define G4_power_onoff_Pin GPIO_PIN_8
#define G4_power_onoff_GPIO_Port GPIOB
#define SPI3_led_crc_error_Pin GPIO_PIN_6
#define SPI3_led_crc_error_GPIO_Port GPIOI
#define SPI3_led_rf_send_Pin GPIO_PIN_7
#define SPI3_led_rf_send_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

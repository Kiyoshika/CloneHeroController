/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t HID_buffer[8] = { 0 };

uint8_t get_key_map(char key)
{
	// only map D, F, J, K, L keys (for clone hero)
	switch (key)
	{
		case 'd':
			return 0x07;
		case 'f':
			return 0x09;
		case 'j':
			return 0x0D;
		case 'k':
			return 0x0E;
		case 'l':
			return 0x0F;
	}

	return 0x00;
}
void actuate_key(USBD_HandleTypeDef* usb_handle, uint8_t* HID_buffer, size_t buffer_index, char key)
{
	size_t calculated_idx = 2 + buffer_index;
	HID_buffer[calculated_idx] = get_key_map(key);
	USBD_HID_SendReport(usb_handle, HID_buffer, 8);
	HAL_Delay(50);
}

void release_key(USBD_HandleTypeDef* usb_handle, uint8_t* HID_buffer, size_t buffer_index)
{
	size_t calculated_idx = 2 + buffer_index;
	HID_buffer[calculated_idx] = 0x00;
	USBD_HID_SendReport(usb_handle, HID_buffer, 8);
	HAL_Delay(50);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  bool pressed_keys[5] = {false, false, false, false, false};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // writing logic in such a way that if one button is release but user is still holding down another,
	  // that other button will still be actuated. This is mainly for handling long notes/hammer ons

	  // actuate keys
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 1)
	  {
		  pressed_keys[0] = true;
		  actuate_key(&hUsbDeviceFS, HID_buffer, 0, 'd');
	  }

	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1)
	  {
		  pressed_keys[1] = true;
		  actuate_key(&hUsbDeviceFS, HID_buffer, 1, 'l');
	  }

	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1)
	  {
		  pressed_keys[2] = true;
		  actuate_key(&hUsbDeviceFS, HID_buffer, 2, 'f');
	  }

	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 1)
	  {
		  pressed_keys[3] = true;
		  actuate_key(&hUsbDeviceFS, HID_buffer, 3, 'j');
	  }

	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 1)
	  {
		  pressed_keys[4] = true;
		  actuate_key(&hUsbDeviceFS, HID_buffer, 4, 'k');
	  }

	  // release keys
	  if (pressed_keys[0] && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)
	  {
		  pressed_keys[0] = false;
		  release_key(&hUsbDeviceFS, HID_buffer, 0);
	  }

	  if (pressed_keys[1] && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0)
	  {
		  pressed_keys[1] = false;
		  release_key(&hUsbDeviceFS, HID_buffer, 1);
	  }

	  if (pressed_keys[2] && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 0)
	  {
		  pressed_keys[2] = false;
		  release_key(&hUsbDeviceFS, HID_buffer, 2);
	  }

	  if (pressed_keys[3] && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)
	  {
		  pressed_keys[3] = false;
		  release_key(&hUsbDeviceFS, HID_buffer, 3);
	  }

	  if (pressed_keys[4] && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0)
	  {
		  pressed_keys[4] = false;
		  release_key(&hUsbDeviceFS, HID_buffer, 4);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


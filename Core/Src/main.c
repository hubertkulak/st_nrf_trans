/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "nRF24_Defs.h"
#include "nRF24.h"
#include "../BME280/bme280.h"
#include "i2c_driver.h"
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
volatile uint8_t nrf24_rx_flag, nrf24_tx_flag, nrf24_mr_flag;
uint8_t Nrf24_Message[NRF24_PAYLOAD_SIZE];
uint8_t Message[32];
uint8_t MessageLength;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
void bme280_reading();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float temperature =0;
float humidity =0;
float pressure =0;



struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

char line1[16];
char line2[16];
char line3[16];

  uint8_t i;
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
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev);

     dev.settings.osr_h = BME280_OVERSAMPLING_1X;
     dev.settings.osr_p = BME280_OVERSAMPLING_16X;
     dev.settings.osr_t = BME280_OVERSAMPLING_2X;
     dev.settings.filter = BME280_FILTER_COEFF_16;
     rslt = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);

     rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
     dev.delay_ms(40);

     rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);







     /****** Suspend the Ticks before entering the STOP mode or else this can wake the device up **********/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	  SystemClock_Config();

	  	 HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0xEA60, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	     char *str = "GOING IN STOP MODE!!!\n\n";
	     HAL_UART_Transmit(&huart2, (uint8_t *) str, strlen (str), HAL_MAX_DELAY);
	     bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev);
	     /* Enter Stop Mode */
	     HAL_SuspendTick();
	     HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);


	     HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	     bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
	     nRF24_Init(&hspi2);
	     nRF24_SetRXAddress(0, "Nad");
	     nRF24_SetTXAddress("Odb");
	     nRF24_TX_Mode();

	     for(i=0; i<5; i++)
	     	 	  {
	     	 		  MessageLength = sprintf(Message, "%03.1f%03.1f%03.1f", temperature, humidity,pressure );
	     	 		  nRF24_WriteTXPayload(Message);
	     	 		  HAL_Delay(1);
	     	 		  nRF24_WaitTX();
	     	 		  bme280_reading();
	     	 		  HAL_Delay(1000);
	     	 	  }

	     bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	SystemClock_Config ();
	HAL_ResumeTick();
	char *str = "WAKEUP FROM RTC\n NOW GOING IN STOP MODE AGAIN\n\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) str, strlen (str), HAL_MAX_DELAY);

}

void bme280_reading()
{
	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		 	     if(rslt == BME280_OK)
		 	     {
		 	       temperature = comp_data.temperature / 100.0;      /* °C  */
		 	       humidity = comp_data.humidity / 1024.0;           /* %   */
		 	       pressure = comp_data.pressure / 10000.0;          /* hPa */

		 	       sprintf(line1, "HUMID: %03.1f \n", humidity);
		 	       sprintf(line2, "TEMP: %03.1f \n", temperature);
		 	       sprintf(line3, "PRESS: %03.1f \n", pressure);

		 	       HAL_UART_Transmit(&huart2,(uint8_t *) line1, strlen(line1), HAL_MAX_DELAY);
		 	       HAL_UART_Transmit(&huart2,(uint8_t *) line2, strlen(line2), HAL_MAX_DELAY);
		 	       HAL_UART_Transmit(&huart2,(uint8_t *) line3, strlen(line3), HAL_MAX_DELAY);
		 	       HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);

		 	     	 }

}

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Serial lib
#include "EasyProfile/EasyProfile.h"
#include "EasyProfile/EasyObjectDictionary.h"

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
//Serial init
EasyObjectDictionary eOD;
EasyProfile eP(&eOD);

//DMA Buffer
uint8_t RxBuffer[256] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//this function provide by ST
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart1){
		char* rxData = (char*)RxBuffer;
		int   rxSize = Size;
		Ep_Header header;
		while(EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header)){
	        rxData = 0;
	    	rxSize = 0;
	    	uint32 fromId = header.fromId;
	    	(void)fromId;
	        switch(header.cmd){
	            case EP_CMD_STATUS_:{
	                Ep_Status ep_Status;
	                if(EP_SUCC_ == eOD.Read_Ep_Status(&ep_Status)){
	                	//get EP_CMD_STATUS_ pkg here
	                	//LED test
	                	HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port,LED_YELLOW_Pin);
	                	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
	                }
	            }break;
	            case EP_CMD_Raw_GYRO_ACC_MAG_:{
	                Ep_Raw_GyroAccMag ep_Raw_GyroAccMag;
	                if(EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&ep_Raw_GyroAccMag)){
	                	//get EP_CMD_Raw_GYRO_ACC_MAG_ pkg here
	                	float gyro_x = ep_Raw_GyroAccMag.gyro[0];
	                	float gyro_y = ep_Raw_GyroAccMag.gyro[1];
	                	float gyro_z = ep_Raw_GyroAccMag.gyro[2];

	                	float acc_x = ep_Raw_GyroAccMag.acc[0];
	                	float acc_y = ep_Raw_GyroAccMag.acc[1];
	                	float acc_z = ep_Raw_GyroAccMag.acc[2];

	                	float mag_x = ep_Raw_GyroAccMag.mag[0];
	                	float mag_y = ep_Raw_GyroAccMag.mag[1];
	                	float mag_z = ep_Raw_GyroAccMag.mag[2];
	                	//unused value
	                	//if you need use those value please delete those code
	                	(void)gyro_x;(void)gyro_y;(void)gyro_z;

	                	(void)acc_x;(void)acc_y;(void)acc_z;

	                	(void)mag_x;(void)mag_y;(void)mag_z;
	                	//LED test
	                	HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port,LED_YELLOW_Pin);
	                	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
	                }
	            }break;
	            case EP_CMD_Q_S1_E_:{
	                Ep_Q_s1_e ep_Q_s1_e;
	                if(EP_SUCC_ == eOD.Read_Ep_Q_s1_e(&ep_Q_s1_e)){
	                	//get EP_CMD_Q_S1_E_ pkg here
	                    float q1 = ep_Q_s1_e.q[0];
	                    float q2 = ep_Q_s1_e.q[1];
	                    float q3 = ep_Q_s1_e.q[2];
	                    float q4 = ep_Q_s1_e.q[3];
	                	//unused value
	                	//if you need use those value please delete those code
	                	(void)q1;(void)q2;(void)q3;(void)q4;
	                	//LED test
	                	HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port,LED_YELLOW_Pin);
	                	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
	                }
	            }break;
	            case EP_CMD_RPY_:{
	                Ep_RPY ep_RPY;
	                if(EP_SUCC_ == eOD.Read_Ep_RPY(&ep_RPY)){
	                	//get EP_CMD_RPY_ pkg here
	                    float roll = ep_RPY.roll;
	                    float pitch = ep_RPY.pitch;
	                    float yaw = ep_RPY.yaw;
	                	//unused value
	                	//if you need use those value please delete those code
	                	(void)roll;(void)pitch;(void)yaw;
	                	//LED test
	                	HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port,LED_YELLOW_Pin);
	                	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
	                }
	            }break;
	            case EP_CMD_COMBO_: {
	                Ep_Combo ep_Combo;
	                if(EP_SUCC_ == eOD.Read_Ep_Combo(&ep_Combo)){
	                	//get EP_CMD_COMBO_ pkg here
	                    // Accelerometer:
	                    float ax = (ep_Combo.ax)*(1e-5f);                     // Unit: 1g, 1g = 9.794m/(s^2)
	                    float ay = (ep_Combo.ay)*(1e-5f);
	                    float az = (ep_Combo.az)*(1e-5f);
	                    // Gyroscope:
	                    float wx = (ep_Combo.wx)*(1e-5f);                     // Unit: rad/s
	                    float wy = (ep_Combo.wy)*(1e-5f);
	                    float wz = (ep_Combo.wz)*(1e-5f);
	                    // Magnetometer:
	                    float mx = (ep_Combo.mx)*(1e-3f);                     // Unit: one earth magnetic field
	                    float my = (ep_Combo.my)*(1e-3f);                     // vector (mx, my, mz) is used as direction reference of the local magnetic field.
	                    float mz = (ep_Combo.mz)*(1e-3f);
	                    // Quaternion in (q1,q2,q3,q4)=(w,x,y,z) format
	                    float q1 = (ep_Combo.q1)*(1e-7f);
	                    float q2 = (ep_Combo.q2)*(1e-7f);
	                    float q3 = (ep_Combo.q3)*(1e-7f);
	                    float q4 = (ep_Combo.q4)*(1e-7f);
	                    // RPY:
	                    float roll  = (ep_Combo.roll)*(1e-2f);                // Unit: degree
	                    float pitch = (ep_Combo.pitch)*(1e-2f);               // Unit: degree
	                    float yaw   = (ep_Combo.yaw)*(1e-2f);                 // Unit: degree
	                	//unused value
	                	//if you need use those value please delete those code
	                	(void)ax;(void)ay;(void)az;

	                	(void)wx;(void)wy;(void)wz;

	                	(void)mx;(void)my;(void)mz;

	                	(void)q1;(void)q2;(void)q3;(void)q4;

	                	(void)roll;(void)pitch;(void)yaw;
	                	//LED test
	                	HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port,LED_YELLOW_Pin);
	                	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
	                }
	                } break;
	            }
		}
		//reopen RX
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, sizeof(RxBuffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //open RX
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, sizeof(RxBuffer));
   __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

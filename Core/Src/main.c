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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Do    383
#define Re    340
#define Mi    304
#define Fa    287
#define Sol   255
#define Ra    227
#define Si    203
#define DDo   191
#define qNote 600
#define wNote 1800


typedef enum
{
  IDLE,
  PLAYING
}PLAYBACKSTATE;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rxData[1];

static PLAYBACKSTATE playBackState = IDLE;
static uint32_t lastTick = 0;
static uint32_t delayStart = 0;
static uint16_t i = 0;
static uint8_t rxCommand = 0;

uint16_t song[] =
    {
        Do, Mi, Sol, Do, Mi, Sol, Ra, Ra, Ra, Sol,
        Fa, Fa, Fa, Mi, Mi, Mi, Re, Re, Re, Do
    };

//uint16_t time[] =
//    {
//        qNote, qNote, qNote, qNote, qNote, qNote, qNote, qNote, qNote, wNote,
//        qNote, qNote, qNote, qNote, qNote, qNote, qNote, qNote, qNote, wNote
//    };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart2, rxData, sizeof(rxData));

  rxCommand = rxData[0];
  playBackState = (rxCommand == 'a' || rxCommand == 'b') ? PLAYING : IDLE;
  i = 0;
  lastTick = HAL_GetTick();
  delayStart = HAL_GetTick();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void updatePlayBack()
{
  uint32_t currTick = HAL_GetTick();

  if(playBackState == PLAYING)
  {
    if(rxCommand == 'a')
    {
      //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
      // 20ms 마다 TIM4->PSC , TIM4->CCR1 업데이트
      if((currTick - lastTick) >= 20 && i < (sizeof(song)/sizeof(song[0])))
      {
        TIM4->PSC = song[i];
        TIM4->CCR1 = 300;
        lastTick = currTick;
      }

      // 600ms 가 지난후에 다음 음으로 이동
      if((currTick - delayStart) >= 600 && i < (sizeof(song)/sizeof(song[0])))
      {
        i++;
        delayStart = currTick;
      }

      // 모든 음을 재생한 후에 상태를 IDLE로 바꿈
      if(i >= (sizeof(song)/sizeof(song[0])))
      {
        playBackState = IDLE;
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
      }
    }
    else if(rxCommand == 'b')
    {
      //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
      if((currTick - lastTick) >= 20 && i < (sizeof(song)/sizeof(song[0])))
      {
        TIM4->PSC = song[i];
        TIM4->CCR1 = 300;
        lastTick = currTick;
      }

      // 600ms 가 지난후에 다음 음으로 이동
      if((currTick - delayStart) >= 600 && i < (sizeof(song)/sizeof(song[0])))
      {
        i++;
        delayStart = currTick;
      }

      // 모든 음을 재생한 후에 상태를 IDLE로 바꿈
      if(i >= (sizeof(song)/sizeof(song[0])))
      {
        playBackState = IDLE;
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
      }
    }
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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart2, rxData, sizeof(rxData));



//  for(int i = 0; i <= sizeof(song); i++)
//  {
//    TIM4->PSC = song[i];
//    HAL_Delay(time[i]);
//    TIM4->CCR1 = 0;
//    HAL_Delay(5);
//    TIM4->CCR1 = 500;
//  }
//  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    updatePlayBack();
    HAL_Delay(1);


//    TIM4->PSC = 383;    // 도
//    HAL_Delay(500);
//    TIM4->CCR1 = 0;     // 출력 안함
//    HAL_Delay(10);      // 음간 간격을 살짝 끊는다
//    TIM4->CCR1 = 500;   // 출력
//
//    TIM4->PSC = 304;    // 미
//    HAL_Delay(500);
//    TIM4->CCR1 = 0;
//    HAL_Delay(10);
//    TIM4->CCR1 = 500;
//
//    TIM4->PSC = 255;    // 솔
//    HAL_Delay(500);
//    TIM4->CCR1 = 0;
//    HAL_Delay(10);
//    TIM4->CCR1 = 500;



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
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

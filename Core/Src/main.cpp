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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.hpp"
#include "gm6020.hpp"
#include "HW_fdcan.hpp"
#include <cmath>
#include "stdbool.h"
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
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile float actual_angle_rad = 0.0f;
volatile float error_rad = 0.0f;
volatile float target_velocity = 0.0f;
volatile float actual_velocity = 0.0f;
volatile float output_current = 0.0f;
volatile float target_angle_rad = 0.0f;
volatile float monitor_actual_angle_rad = 0.0f;
volatile int i = 1;
volatile float pid_actual_angle_rad = 0.0f;
volatile bool position_control_first_run = true;
volatile float last_actual_angle_rad = 0.0f;

GM6020 motor1(1);

// 速度环
PidParams speed_pid_params_1 = {
    .kp = 15.0f, .ki = 0.1f, .kd = 0.025f,
    .output_max = 16384.0f, .integral_max = 4000.0f
};
Pid speed_pid_1(speed_pid_params_1);




// 位置环
PidParams pos_pid_params_1 = {
    .kp = 110.0f, .ki = 0.0f, .kd = 5.0f,
    .output_max = 400.0f, .integral_max = 100.0f
};
Pid position_pid_1(pos_pid_params_1);


// 4. 任务控制状态变量
enum ControlMode {
    SPEED_CONTROL,
    POSITION_CONTROL
};

// 手动切换
volatile ControlMode current_mode = POSITION_CONTROL; 

const float TARGET_A = 1.0f * PI / 4.0f; 
const float TARGET_B = -1.0f * PI / 1.0f;

static float current_target_angle = TARGET_A; 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_SPI2_Init();
  MX_SPI6_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM12_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */

  HAL_FDCAN_Start(&hfdcan1);
  FdcanFilterInit(&hfdcan1, FDCAN_FILTER_TO_RXFIFO0); // 配置CAN过滤器
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能接收中断

  HAL_TIM_Base_Start_IT(&htim1); 
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_FDCAN|RCC_PERIPHCLK_UART7;
  PeriphClkInitStruct.PLL2.PLL2M = 16;
  PeriphClkInitStruct.PLL2.PLL2N = 128;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) 
    {
        if (current_mode == SPEED_CONTROL)
        {
            // --- 任务一：速度闭环控制 ---

            float t = HAL_GetTick() / 1000.0f; 
            static volatile float target_speed = 0.0f, actual_speed = 0.0f, output_current = 0.0f;
            target_speed = 300.0f * sin(2.0f * PI * 0.5f * t); //定义正弦曲线
            actual_speed = motor1.getVelocityRPM();
            if(t>0.0f && t <2.0f){
                output_current = speed_pid_1.pidCalc(target_speed, actual_speed); //速度--电流
            }

            motor1.encode((int16_t)output_current, 0, 0, 0);
        }
        else if (current_mode == POSITION_CONTROL)
        {
            // --- 任务二：位置闭环控制 ---
            
            target_angle_rad = current_target_angle;
            actual_angle_rad = motor1.getAngleRad();
            
            if(actual_angle_rad > PI){
              actual_angle_rad-=2*PI;
            }

            // 解算连续角度
            if (position_control_first_run)
            {
                pid_actual_angle_rad = actual_angle_rad;
                position_control_first_run = false;
            }
            else
            {
                // 计算与上一次的“最短”差异
                float diff = actual_angle_rad - last_actual_angle_rad;

                // 检查是否发生了 >PI 或 <-PI 的跳跃
                if (diff > PI)  diff -= 2.0f * PI; // 从 -PI 附近跳到 +PI 附近
                if (diff < -PI) diff += 2.0f * PI; // 从 +PI 附近跳到 -PI 附近

                pid_actual_angle_rad += diff;
            }

            last_actual_angle_rad = actual_angle_rad;

            float error_for_check = current_target_angle - actual_angle_rad;
            if (error_for_check > PI) error_for_check -= 2.0f * PI;
            else if (error_for_check < -PI) error_for_check += 2.0f * PI;           
            float tolerance_rad = 0.05f;
            if (std::fabs(error_for_check) < tolerance_rad)
            {
                if (current_target_angle == TARGET_A){
                     current_target_angle = TARGET_B;
                  }
                else if(i<2){
                    //current_target_angle = TARGET_A;
                }
            }

            error_rad = current_target_angle - pid_actual_angle_rad;
            //如果取消注释，就是最短路径，注释掉，更符合作业的例图
            //if (error_rad > PI) error_rad -= 2.0f * PI;
            //else if (error_rad < -PI) error_rad += 2.0f * PI;

            target_velocity = position_pid_1.pidCalc(0, -error_rad); //角度--速度

          
            actual_velocity = motor1.getVelocityRPM();

            output_current = speed_pid_1.pidCalc(target_velocity, actual_velocity);//速度--电流

            motor1.encode((int16_t)output_current, 0, 0, 0);
        }
    }
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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

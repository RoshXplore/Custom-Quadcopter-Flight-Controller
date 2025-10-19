/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "PID control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Calibrate 1


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;

    float pitch_reference;
	float pitch_p;
	float pitch_error;
	float pitch_i;
	float pitch_error_sum = 0;
	float pitch_derivative;
	float pitch_d;
	float pitch_pid;

	float pitch_rate_reference;
	float pitch_rate_p;
	float pitch_rate_error;
	float pitch_rate_i;
	float pitch_rate_error_sum = 0;
	float pitch_rate_derivative;
	float pitch_rate_d;
	float pitch_rate_pid;
	float icm20602_gyro_x_prev;

	uint16_t ccr1, ccr2, ccr3, ccr4;

	typedef struct {

	    uint16_t LV;   // Pin for UP direction
	    uint16_t LH; // Pin for DOWN direction
	    uint16_t RV; // Pin for LEFT direction
	    uint16_t RH;// Pin for RIGHT direction

	} Joystick_TypeDef;

	Joystick_TypeDef Joystick;


	float pitch_in_kp =8;
	float pitch_in_ki=2;
	float pitch_in_kd=5;

	float pitch_out_kp=10;
	float pitch_out_ki=3;
	float pitch_out_kd=5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t ICValue1=0;
uint32_t ICValue2=0;
uint32_t ICValue3=0;
uint32_t ICValue4=0;

uint16_t channel1_raw=0;
uint16_t channel2_raw=0;
uint16_t channel3_raw=0;
uint16_t channel4_raw=0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) // Receiver Channel 1
    {
    	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // If the interrupt is triggered by channel 1
    		{
    			// Read the IC value
    			ICValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    			if (ICValue1 != 0)
    			{
    				// calculate the Duty Cycle
    				channel1_raw = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) *10000)/ICValue1;
    				Joystick.LH = map(channel1_raw, 590,1030,0,1000);


//                	Frequency = 90000000/ICValue;
    			}
    		}    }
    else if (htim->Instance == TIM3)// Receiver Channel 2
    {
    	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // If the interrupt is triggered by channel 1
    		{
    			// Read the IC value
    			ICValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    			if (ICValue2 != 0)
    			{
    				// calculate the Duty Cycle
    				channel2_raw = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) *100)/ICValue2;
    				Joystick.RV = map( channel2_raw, 75,120,0,1000);
    				//Frequency = 90000000/ICValue;
    			}
    		}
    }
    else if (htim->Instance == TIM4)// Receiver Channel 3
    {
    	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // If the interrupt is triggered by channel 1
    		{
    			// Read the IC value
    			ICValue3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    			if (ICValue3 != 0)
    			{
    				// calculate the Duty Cycle
    				channel3_raw = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) *100)/ICValue3;
    				Joystick.LV = map( channel3_raw, 7,127,0,400);
    				//Frequency = 90000000/ICValue;
    			}
    		}
    }
    // Add more conditions for other timers as needed
    else if (htim->Instance == TIM12)// Receiver Channel 4
    {
    	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // If the interrupt is triggered by channel 1
    		{
    			// Read the IC value
    			ICValue4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    			if (ICValue4 != 0)
    			{
    				// calculate the Duty Cycle
    				channel4_raw = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) *1000)/ICValue4;
    				Joystick.RH = map(channel4_raw, 60,105,0,1000);
    			//	Frequency = 90000000/ICValue;
    			}
    		}
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7 && Calibrate==0)
    {


    	// Double PID

    	Double_Roll_Pitch_PID_Calculation(&pitch, (Joystick.RV ) * 0.125f, MPU6050.KalmanAngleX,MPU6050.Gx);
        Double_Roll_Pitch_PID_Calculation(&roll, (Joystick.RH ) * 0.125f, MPU6050.KalmanAngleY, MPU6050.Gy);



//  	  pitch_reference = (Joystick.RV - 500) * 0.1f;
//  	 		  pitch_error = pitch_reference - MPU6050.KalmanAngleX;
//  	 		  pitch_p = pitch_error * pitch_out_kp;
//
//  	 		  pitch_error_sum = pitch_error_sum + pitch_error * 0.0025;
//  	 		  //if(motor_arming_flag == 0 || iBus.LV < 1030) pitch_error_sum = 0;
//  	 		  pitch_i = pitch_error_sum * pitch_out_ki;
//
//  	 //	  pitch_derivative = (BNO080_Pitch - BNO080_Pitch_Prev) / 0.001;
//  	 //		  BNO080_Pitch_Prev = BNO080_Pitch;
//
//  	 		  pitch_derivative = MPU6050.Gx;
//  	 		  pitch_d = -pitch_derivative * pitch_out_kd;
//
//  	 		  pitch_pid = pitch_p + pitch_i + pitch_d;
//
//  	 		  pitch_rate_reference = pitch_pid;
//  	 		  pitch_rate_error = pitch_rate_reference - MPU6050.Gx;
//  	 		  pitch_rate_p = pitch_rate_error * pitch_in_kp;
//
//  	 		  pitch_rate_error_sum = pitch_rate_error_sum + pitch_rate_error * 0.0025;
//  	 		  //if(motor_arming_flag == 0 || iBus.LV < 1030) pitch_rate_error_sum = 0;
//  	 		  pitch_rate_i = pitch_rate_error_sum * pitch_in_ki;
//
//  	 		  pitch_rate_derivative = (MPU6050.Gx - icm20602_gyro_x_prev) / 0.0025;
//  	 		  icm20602_gyro_x_prev = MPU6050.Gx;
//  	 		  pitch_rate_d = -pitch_rate_derivative * pitch_in_kd;
//
//  	 		  pitch_rate_pid = pitch_rate_p + pitch_rate_i + pitch_rate_d;

        if(Joystick.LV < 10)
        		  {
        			  Reset_All_PID_Integrator();
        		  }


  			  ccr1 = 400 + (Joystick.LV ) ;//- pitch.in.pid_result + roll.in.pid_result; ;//- pitch_rate_pid;// + (iBus.RH - 1500) * 5 - (iBus.LH - 1500) * 5;
  			  ccr2 = 400 + (Joystick.LV ) ;//+ pitch.in.pid_result + roll.in.pid_result; ;//+ pitch_rate_pid;// + (iBus.RH - 1500) * 5 + (iBus.LH - 1500) * 5;
  			  ccr3 = 400 + (Joystick.LV ) ;//+ pitch.in.pid_result + roll.in.pid_result;;//+ pitch_rate_pid;// - (iBus.RH - 1500) * 5 - (iBus.LH - 1500) * 5;
  			  ccr4 = 400 + (Joystick.LV ) ;//- pitch.in.pid_result + roll.in.pid_result; ; //- pitch_rate_pid;// - (iBus.RH - 1500) * 5 + (iBus.LH - 1500) * 5;

  			 if(Joystick.LV > 30 )
  						  {
  							  TIM1->CCR3 = ccr1 > 800 ? 800 : ccr1 <450 ? 450 : ccr1;
  							  TIM1->CCR4 = ccr2 > 800 ? 800 : ccr2 < 450 ? 450 : ccr2;
  							  TIM9->CCR1 = ccr3 > 800 ? 800 : ccr3 < 450 ? 450 : ccr3;
  							  TIM9->CCR2 = ccr4 > 800 ? 800 : ccr4 < 450 ? 450 : ccr4;
  						  }

  			 else		  {
  							  TIM1->CCR3 = 400;
  							  TIM1->CCR4= 400;
  							  TIM9->CCR1 = 400;
  							  TIM9->CCR2 = 400;
  						  }


    }
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_I2C2_Init();
  MX_TIM12_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
// PWM INPUT FROM RECEIVER INIT
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);

  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&htim12, TIM_CHANNEL_2);

  // MOTOR PWM INIT
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim7);
  // Calibration

//#if Calibrate
//
//  TIM1->CCR4 = 800;
//  TIM9->CCR1 = 800;
//  TIM9->CCR2 = 800;// Set the maximum pulse (2ms)
//  HAL_Delay (2000);  // wait for 1 beep
//
//  TIM1->CCR4 = 400;
//  TIM9->CCR1 = 400;
//  TIM9->CCR2 = 400;// Set the minimum Pulse (1ms)
//  HAL_Delay (2000);  // wait for 2 beeps
//
//  TIM1->CCR4 = 0;
//  TIM9->CCR1 = 0;
//  TIM9->CCR2 = 0;//

//  TIM1->CCR3 = 800;
//  HAL_Delay (2000);
//  TIM1->CCR3 = 400;
//  HAL_Delay (1000);
//  TIM1->CCR3 = 0; //reset to 0, so it can be controlled via ADC
//#endif

// Defining PID for pitch

  //Inner loop
  pitch.in.kp=6;
  pitch.in.kd=5;
  pitch.in.kp=1.5;

  //Outer loop
  pitch.out.kp=45;
  pitch.out.kd=3;
  pitch.out.kp=4;

// Defining PID for roll

  //Inner loop
  roll.in.kp=6;
  roll.in.kd=5;
  roll.in.kp=4.2;

  //Outer loop
  roll.out.kp=45;
  roll.out.kd=3;
  roll.out.kp=4;

  // 500Hz Timer Start

  HAL_TIM_Base_Start_IT(&htim7);

  while (MPU6050_Init(&hi2c2) == 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  MPU6050_Read_All(&hi2c2, &MPU6050);
	   //HAL_Delay (100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

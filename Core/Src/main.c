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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "stepperControl.h"
#include "vl53l0x_api.h"
#include "hx711.h"
#include "serialFromPC.h"
#include "zeroing.h"
#include "pid.h"

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
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

//Stepper Control

stepper thetaMotor;
stepper yMotor;
stepper rMotor;
char doOnceFlag=0;

enum scanStates {posReceive,goToPos,distGet};
enum scanStates scanState;

enum tattooStates {posReceiveTat,goToPosTat,travellingTat};
enum tattooStates tattooState;

// Serial print

uint8_t Message[64];
uint8_t MessageLen;

VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t  vl53l0x_c; // center module
VL53L0X_DEV    Dev = &vl53l0x_c;

uint32_t timer;
uint32_t pidTimer;

//Load Cell Values
hx711_t loadCell;
uint32_t pressureZero;
uint32_t filteredPressure;
double pressureVal;
int64_t pressureValInt;
double setPoint;
//PID Values
PID_TypeDef pressPID;
double deltaR = 0;

typedef struct{
	uint64_t outputSum;
	uint8_t index;
	u_int32_t values[5];

} movingAverageFilter;

//Data Transfer
uint8_t rxBuffer[16];
uint8_t uartRecievedFlag = 0;
Instruction nextInstr;
uint16_t lastRInstruct = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t movingAverage(movingAverageFilter *filter, int newVal){
	//Function Variables
	uint8_t lastValIndex;
	uint8_t filterSize;
	uint32_t lastVal;
	uint32_t average;
	filterSize = sizeof(filter->values)/sizeof(filter->values[0]);
	//Find last value in circular buffer
	lastValIndex = (filter->index+1)%filterSize;
	lastVal = filter->values[lastValIndex];
	//calculate new sum and output average
	filter->outputSum = filter->outputSum-lastVal+newVal;
	average = filter->outputSum/filterSize;
	//Update index and replace
	filter->values[lastValIndex] = newVal;
	filter->index = lastValIndex;

	return average;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	//uint32_t timer;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	InitSerialFromPC(&hlpuart1,rxBuffer);
	movingAverageFilter pressureMAF = {0};
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_LPUART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

	//Enable Timer Interrupts
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
	//Initialize stepper structures
	initStepper(&thetaMotor, &htim3, TIM_CHANNEL_1, thetaDir_GPIO_Port, thetaDir_Pin, 400);
	initStepper(&yMotor,&htim2,TIM_CHANNEL_1,yDir_GPIO_Port,yDir_Pin, 400);
	initStepper(&rMotor, &htim4, TIM_CHANNEL_3, rDir_GPIO_Port, rDir_Pin, 400);

	yMotor.PPS_ZeroDefault = 200;
	thetaMotor.PPS_ZeroDefault = 200;
	rMotor.PPS_ZeroDefault = 200;

	Dev->I2cHandle = &hi2c2;
	Dev->I2cDevAddr = 0x52;

	// VL53L0X init for Single Measurement
	//

	VL53L0X_WaitDeviceBooted( Dev );
	VL53L0X_DataInit( Dev );
	VL53L0X_StaticInit( Dev );
	VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

	// Enable/Disable Sigma and Signal check
	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	//Enable UART Enable IT
	HAL_UART_Receive_IT(&hlpuart1, rxBuffer, 6); //receive 6 bytes

	//Start timer for uSDelay for HX711
	HAL_TIM_Base_Start(&htim5);
	//Init load cell
	hx711_init(&loadCell, loadCLK_GPIO_Port, loadCLK_Pin, loadDATA_GPIO_Port, loadDATA_Pin, &htim5);
	HAL_Delay(1000);
	pressureZero = hx711_value_ave(&loadCell, 10);
//	setTarget(&thetaMotor, 1000, 1);
//	setTarget(&yMotor, 800, 1);
//	setTarget(&rMotor,500,1);

	//PID Setup
	setPoint = 17000;
	PID(&pressPID, &pressureVal, &deltaR, &setPoint, 0.01, 0, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&pressPID, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&pressPID, 100);
	PID_SetOutputLimits(&pressPID, -200, 200);

	//
	//	MessageLen = sprintf((char*)Message, "Here 1 \n\r");
	//	HAL_UART_Transmit(&hlpuart1, Message, MessageLen, 100);

	//	hx711_init(&loadCell, loadCLK_GPIO_Port, loadCLK_Pin, loadDATA_GPIO_Port, loadDATA_Pin, &htim5);
	//		MessageLen = sprintf((char*)Message, "Here 2 \n\r");
	//		HAL_UART_Transmit(&hlpuart1, Message, MessageLen, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//timer = HAL_GetTick();

	//testing stopping function
	//	GoHome(&thetaMotor);
	//	HAL_Delay(10000);
	//	GoHome(&yMotor);
	//	HAL_Delay(10000);
	//	GoHome(&rMotor);
	//	HAL_Delay(10000);

	while (1)
	{
		//		HAL_Delay(100);
		//		MessageLen = sprintf((char*)Message, "Current Position: %d Target: %d \n\r",(int)yMotor.CurrentPosition,(int)yMotor.TargetPosition);
		//		HAL_UART_Transmit(&hlpuart1, Message, MessageLen, 100);
		//
		//		if((HAL_GetTick()-timer>=5000) && (HAL_GetTick()-timer<=5180)){
		//			setTarget(&yMotor, 1000, 0);
		//		}
		//		timer = HAL_GetTick();
		//		VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
		//		if(RangingData.RangeStatus == 0)
		//		{
		//			MessageLen = sprintf((char*)Message, "Measured distance: %i", RangingData.RangeMilliMeter);
		//			HAL_UART_Transmit(&hlpuart1, Message, MessageLen, 100);
		//			MessageLen = sprintf((char*)Message, "calcTime: %i\n\r",(int)(HAL_GetTick()-timer));
		//			HAL_UART_Transmit(&hlpuart1, Message, MessageLen, 100);
		//		}

//		//Scanning Sequence
//		//check if data has been received
//		if(scanState == posReceive && yMotor.Status == Stopped && thetaMotor.Status == Stopped){
//			HAL_GPIO_WritePin(state1LED_GPIO_Port, state1LED_Pin, SET);
//			if(uartRecievedFlag){
//				//retrieve instructions
//				ParseInstructions(rxBuffer, &nextInstr);
//				//enable receive interrupt
//				uartRecievedFlag = 0;
//				HAL_UART_Receive_IT(&hlpuart1, rxBuffer, 6);
//				HAL_GPIO_WritePin(state1LED_GPIO_Port, state1LED_Pin, RESET);
//				scanState = goToPos;
//			}
//		}
//		else if(scanState == goToPos && yMotor.Status == Stopped && thetaMotor.Status == Stopped){
//			//HAL_GPIO_WritePin(state2LED_GPIO_Port, state2LED_Pin, SET);
//			//Increment theta based on direction
//			if(nextInstr.th>=thetaMotor.TargetPosition){
//				setTarget(&thetaMotor, (uint64_t)abs(nextInstr.th - thetaMotor.TargetPosition), 1);
//			}
//			else{
//				setTarget(&thetaMotor, (uint64_t)abs(nextInstr.th - thetaMotor.TargetPosition), 0);
//			}
//			//Increment Y based on direction
//			if(nextInstr.y >= yMotor.TargetPosition){
//				setTarget(&yMotor, (uint64_t)abs(nextInstr.y - yMotor.TargetPosition), 1);
//			}
//			else{
//				setTarget(&yMotor, (uint64_t)abs(nextInstr.y - yMotor.TargetPosition), 0);
//			}
//			//HAL_GPIO_WritePin(state2LED_GPIO_Port, state2LED_Pin, RESET);
//			scanState = distGet;
//		}
//		else if(scanState == distGet && (yMotor.Status != Stopped || thetaMotor.Status != Stopped)){
//			HAL_GPIO_WritePin(state2LED_GPIO_Port, state2LED_Pin, SET);
//			timer = HAL_GetTick();
//		}
//		else if(scanState == distGet && yMotor.Status == Stopped && thetaMotor.Status == Stopped){
//			HAL_GPIO_WritePin(state2LED_GPIO_Port, state2LED_Pin, RESET);
//			HAL_GPIO_WritePin(state3LED_GPIO_Port, state3LED_Pin, SET);
//			VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
//			if(RangingData.RangeStatus == 0){
//				SendSerialInt(RangingData.RangeMilliMeter);
//				timer = HAL_GetTick()-timer;
//				HAL_GPIO_WritePin(state3LED_GPIO_Port, state3LED_Pin, RESET);
//				scanState = posReceive;
//			}
//		}

		//Tattooing sequence
		if(tattooState == posReceiveTat && yMotor.Status == Stopped && thetaMotor.Status == Stopped){
			HAL_GPIO_WritePin(state2LED_GPIO_Port, state2LED_Pin, SET);
			if(uartRecievedFlag){
				//retrieve instructions
				ParseInstructions(rxBuffer, &nextInstr);
				//enable receive interrupt
				uartRecievedFlag = 0;
				HAL_UART_Receive_IT(&hlpuart1, rxBuffer, 6);
				HAL_GPIO_WritePin(state2LED_GPIO_Port, state2LED_Pin, RESET);
				tattooState = goToPosTat ;
			}
		}
		if(tattooState == goToPosTat && yMotor.Status == Stopped && thetaMotor.Status == Stopped){
			HAL_GPIO_WritePin(state3LED_GPIO_Port, state3LED_Pin, SET);
			//Increment theta based on direction
			if(nextInstr.th>=thetaMotor.TargetPosition){
				setTarget(&thetaMotor, (uint64_t)abs(nextInstr.th - thetaMotor.TargetPosition), 1);
			}
			else{
				setTarget(&thetaMotor, (uint64_t)abs(nextInstr.th - thetaMotor.TargetPosition), 0);
			}
			//Increment Y based on direction
			if(nextInstr.y >= yMotor.TargetPosition){
				setTarget(&yMotor, (uint64_t)abs(nextInstr.y - yMotor.TargetPosition), 1);
			}
			else{
				setTarget(&yMotor, (uint64_t)abs(nextInstr.y - yMotor.TargetPosition), 0);
			}
			tattooState = travellingTat;
		}
		if(tattooState == travellingTat && yMotor.Status == Stopped && thetaMotor.Status == Stopped){
			HAL_GPIO_WritePin(state3LED_GPIO_Port, state3LED_Pin, RESET);
			SendSerialChar('a');
			tattooState = posReceiveTat;
		}




		//		if(uartRecievedFlag  && yMotor.Status == Stopped && thetaMotor.Status == Stopped){
		//			//retrieve instructions
		//			ParseInstructions(rxBuffer, &nextInstr);
		//			//enable receive interrupt
		//			uartRecievedFlag = 0;
		//			HAL_UART_Receive_IT(&huart1, rxBuffer, 6);
		//			VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
		//			if(RangingData.RangeStatus == 0){
		//			//send TOF Val
		//			SendSerialInt();
		//			}
		//		}

//		if((HAL_GetTick()-timer)>15000 && !doOnceFlag){
////			setTarget(&thetaMotor, 1000, 0);
////		setTarget(&yMotor, 800, 0);
////			setTarget(&rMotor,500,0);
//			doOnceFlag = 1;
//		}

//			MessageLen = sprintf((char*)Message, " Current: %d ",(int)(thetaMotor.CurrentPosition));
//			HAL_UART_Transmit(&hlpuart1, Message, MessageLen, 100);
//
//			MessageLen = sprintf((char*)Message, "Target: %d\n\r ",(int)(thetaMotor.TargetPosition));
//			HAL_UART_Transmit(&hlpuart1, Message, MessageLen, 100);


		//HAL_Delay(100);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10C0ECFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1250-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1250-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1250-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, thetaDir_Pin|yDir_Pin|rDir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, state3LED_Pin|state2LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(loadCLK_GPIO_Port, loadCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(state1LED_GPIO_Port, state1LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : thetaDir_Pin yDir_Pin rDir_Pin */
  GPIO_InitStruct.Pin = thetaDir_Pin|yDir_Pin|rDir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : thLim_Pin yLim_Pin rLim_Pin */
  GPIO_InitStruct.Pin = thLim_Pin|yLim_Pin|rLim_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : state3LED_Pin state2LED_Pin */
  GPIO_InitStruct.Pin = state3LED_Pin|state2LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : loadCLK_Pin */
  GPIO_InitStruct.Pin = loadCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(loadCLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : loadDATA_Pin */
  GPIO_InitStruct.Pin = loadDATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(loadDATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : state1LED_Pin */
  GPIO_InitStruct.Pin = state1LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(state1LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void  HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim){
	//Theta Motor interrupts
	if(htim == thetaMotor.Timer){
		if(thetaMotor.Status == RunningForward){
			thetaMotor.CurrentPosition++;
		}
		else if (thetaMotor.Status == RunningBackward){
			thetaMotor.CurrentPosition--;
		}
		if(thetaMotor.CurrentPosition == thetaMotor.TargetPosition){
			HAL_TIM_PWM_Stop(thetaMotor.Timer, thetaMotor.Channel);
			HAL_TIM_Base_Stop(thetaMotor.Timer);
			thetaMotor.Status = Stopped;
		}
	}
	//yMotor Interrupts
	if(htim == yMotor.Timer){
		if(yMotor.Status == RunningForward){
			yMotor.CurrentPosition++;
		}
		else if (yMotor.Status == RunningBackward){
			yMotor.CurrentPosition--;
		}
		if(yMotor.CurrentPosition == yMotor.TargetPosition){
			HAL_TIM_PWM_Stop(yMotor.Timer, yMotor.Channel);
			HAL_TIM_Base_Stop(yMotor.Timer);
			yMotor.Status = Stopped;
		}
	}
	//rMotor Interrupts
	if(htim == rMotor.Timer){
		if(rMotor.Status == RunningForward){
			rMotor.CurrentPosition++;
		}
		else if (rMotor.Status == RunningBackward){
			rMotor.CurrentPosition--;
		}
		if(rMotor.CurrentPosition == rMotor.TargetPosition){
			HAL_TIM_PWM_Stop(rMotor.Timer, rMotor.Channel);
			HAL_TIM_Base_Stop(rMotor.Timer);
			rMotor.Status = Stopped;
		}

	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//set flag
	uartRecievedFlag = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	//check which limit switch was hit
	if(GPIO_Pin == rLim_Pin){
		zeroStepper(&rMotor);
	}
	else if(GPIO_Pin == thLim_Pin){
		zeroStepper(&thetaMotor);
	}
	else if(GPIO_Pin == yLim_Pin){
		zeroStepper(&yMotor);
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

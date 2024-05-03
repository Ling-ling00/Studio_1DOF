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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "arm_math.h"
#include "ModBusRTU.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
int debug =0;

//modbus
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];

int count=0;
int vaccum;
int gripper;

int home =0;
int point_mode = 0;
int setshelves_mode = 0;
int jog_mode = 0;
int place_order;
int pick_order;

//Joy
int x_floor[5] = {-1, -1, -1, -1, -1};
int z_floor[5] = {-1, -1, -1, -1, -1};
uint8_t RxBuffer[10];

//sensor
uint8_t sensor_up = 0;
uint8_t sensor_down = 0;

//Joy control
uint16_t z_target_position = 0;
uint16_t x_target_position = 0;

//for PID joy
arm_pid_instance_f32 PID = {0};
float setposition = 0; //target of PID
float Vfeedback = 0;
float Error;

//base system
int gotofloor;

//QEI
typedef struct
{
	// for record New / Old value to calculate dx / dt
	uint32_t Position[2];
	uint64_t TimeStamp[2];
	float Velocity[2];
	float QEIPostion_1turn;
	float QEIAngularVelocity;
}
QEI_StructureTypeDef;
QEI_StructureTypeDef QEIdata = {0};
uint64_t _micros = 0;
enum
{
	NEW,OLD
};
float x_position = 0;
uint16_t z_status = 0;
float acceleration = 0;
float velocity = 0;
float position = 0;
float angular_velocity;
float angular_position;
int position_round = 0;
int counter = 0;


//mode
int mode = 0;

//trajectory
float trajec_position;
float trajec_velocity;
float trajec_acceleration;
float trajec_target;
uint8_t trajec_state;
float p0;

//position PID
float position_Kp = 0;
float position_Ki = 0;
float position_Kd = 0;
float position_Ts = 0.005;
float position_PID_output = 0;

//velocity PID
float velocity_Kp = 0.008;
float velocity_Ki = 0.005;
float velocity_Kd = 0.00001;
float velocity_Ts = 0.001;
float velocity_PID_output = 0;

//gripper
int gripper_status = 0;

int vacuum_state = 0;
int gripper_state = 0;
int gripper_ready = 0;


int Reed_out = 0;
int Reed_in = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
//modbus
void set_shelves();
void run_pointmode();
void set_home();
void run_jogmode();

void vacuum_onoff();
void gripper_pushpull();

//Joy
void UARTDMAConfig();
void update_position();

//Timer
uint64_t micros();

//QEI
void QEIEncoderPosVel_Update();

//Motor
void setMotor();

//trajectory
void trajectory();

//PID
void velocity_PID();
void position_PID();

//gripper
void gripper_pick();
void gripper_place();

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //modbus
  hmodbus.huart = &huart2;
  hmodbus.htim = &htim16;
  hmodbus.slaveAddress = 0x15;
  hmodbus.RegisterSize =200;
  Modbus_init(&hmodbus, registerFrame);
  HAL_TIM_Base_Start_IT(&htim6);

  //Joy
  UARTDMAConfig();

  //Motor
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);

  //encoder
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim5);

  //PID
//    PID.Kp = 0.34;
//    PID.Ki = 0.0000006;
//    PID.Kd = 0.0000001;
    PID.Kp = 0.478;
    PID.Ki = 0.0000006;
    PID.Kd = 0.0000000151;
    arm_pid_init_f32(&PID, 0);

  //Home
  while(1)
  {
	  sensor_up = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	  sensor_down = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	  registerFrame[0x00].U16=22881;
	  if(sensor_down == 0){
		  registerFrame[0x00].U16=22881;
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999*1.5/10.0);
	  }
	  else if(sensor_down == 1){
		  registerFrame[0x00].U16=22881;
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		  __HAL_TIM_SET_COUNTER(&htim4, 0);
		  QEIdata.Position[NEW] = 0;
		  QEIEncoderPosVel_Update();
		  angular_position = 0.0;
		  position = 0.0;
		  position_round = 0;
		  z_target_position = 0;
		  x_target_position = 0;
		  break;
	  }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sensor_up = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	  sensor_down = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);

	  static uint64_t timestamp_QEI = 0;
	  uint64_t currentTime = micros();
	  if(currentTime > timestamp_QEI)
	  {
		  timestamp_QEI = currentTime + 1000;//us
		  QEIEncoderPosVel_Update();
	  }

	  Modbus_Protocal_Worker();
//	  registerFrame[0x00].U16=22881;

	  set_shelves();
	  run_pointmode();
	  set_home();
	  run_jogmode();
	  gripper_pushpull();
	  vacuum_onoff();
//
//
//	  /*-------------------------go to mode-------------------------------*/
//
//	  if(mode == 3){
//		  if (gotofloor == 1){
//			  setposition = z_floor[0];
//		  }
//		  else if (gotofloor == 2){
//			  setposition = z_floor[1];
//		  }
//		  else if (gotofloor == 3){
//			  setposition = z_floor[2];
//		  }
//		  else if (gotofloor == 4){
//			  setposition = z_floor[3];
//		  }
//		  else if (gotofloor == 5){
//			  setposition = z_floor[4];
//		  }
//
//		  trajectory();
//
//
//		  static uint64_t timestamp_velocity_PID = 0;
//		  uint64_t currentTime = micros();
//		  if(currentTime > timestamp_velocity_PID)
//		  {
//			  timestamp_velocity_PID = currentTime + 1000;//us
//			  QEIEncoderPosVel_Update();
//			  velocity = angular_velocity*14/2.0/M_PI;
//			  if(fabsf(setposition-position) < 0.15){
//				  Vfeedback = 0;
//			  }
//			  else{
//				  velocity_PID();
//				  Vfeedback = velocity_PID_output;
//			  }
//			  setMotor();
//		  }
//
//		  static uint64_t timestamp_position_PID = 0;
//		  currentTime = micros();
//		  if(currentTime > timestamp_position_PID)
//		  {
//			  timestamp_position_PID = currentTime + 5000;//us
//			  position_PID();
//		  }
//	  }
//
//	  /*-------------------------jog mode-------------------------------*/
//
//	  else if(mode == 2){
//		  //Call every 0.1 s
//		  static uint64_t timestamp =0;
//		  int64_t currentTime = micros();
//		  if(currentTime > timestamp)
//		  {
//			  timestamp =currentTime + 100000;//us
//			  QEIEncoderPosVel_Update();
//		  }
//		  velocity = angular_velocity*14/2.0/M_PI;
//		  update_position();
//		  setposition = z_target_position;
//		  Error = setposition - position;
//		  Vfeedback = arm_pid_f32(&PID, Error);
//		  if (fabsf(Error) <= 0.1){
//			  Vfeedback = 0;
//		  }
//		  if(Vfeedback <= 4.3 && Vfeedback > 0){
//			  Vfeedback = 4.3;
//		  }
//		  if(Vfeedback >= -1.75 && Vfeedback < 0){
//			  Vfeedback = -1.78;
//		  }
//		  if(Vfeedback >= 24){
//			  Vfeedback = 24;
//		  }
//		  setMotor();
//	  }
//	  else if(mode == 1){
//		  //Call every 0.1 s
//		  static uint64_t timestamp =0;
//		  int64_t currentTime = micros();
//		  if(currentTime > timestamp)
//		  {
//			  timestamp =currentTime + 100000;//us
//			  QEIEncoderPosVel_Update();
//		  }
//		  velocity = angular_velocity*14/2.0/M_PI;
//
//		  update_position();
//		  if (gotofloor == 1){
//			setposition = z_floor[0];
//		  }
//		  else if (gotofloor == 2){
//			setposition = z_floor[1];
//		  }
//		  else if (gotofloor == 3){
//		  	setposition = z_floor[2];
//		  }
//		  else if (gotofloor == 4){
//		  	setposition = z_floor[3];
//		  }
//		  else if (gotofloor == 5){
//		  	setposition = z_floor[4];
//		  }
//
//
//		  Error = setposition - position;
//		  Vfeedback = arm_pid_f32(&PID, Error);
//		  if (fabsf(Error) <= 0.1){
//		  	Vfeedback = 0;
//		  }
//		  setMotor();
//	  }
//
//	  else if(mode == 0){
//		  //Call every 0.1 s
//		  		  static uint64_t timestamp =0;
//		  		  int64_t currentTime = micros();
//		  		  if(currentTime > timestamp)
//		  		  {
//		  			  timestamp =currentTime + 100000;//us
//		  			  QEIEncoderPosVel_Update();
//		  		  }
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//		  update_position();
//	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 63487;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim5.Init.Prescaler = 169;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 169;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1145;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 4800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB14 PB15 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void vacuum_onoff(){
 if(registerFrame[0x02].U16==0b0000){
  registerFrame[0x00].U16=22881;
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_RESET);
 }
 else if(registerFrame[0x02].U16==0b0001){
  registerFrame[0x00].U16=22881;
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_SET);
 }
}

void gripper_pushpull(){
// Reed_out = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
// Reed_in = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
 if(registerFrame[0x03].U16==0b0000){  //in
  registerFrame[0x00].U16=22881;
  if(Reed_in == 0){
	 registerFrame[0x00].U16=22881;
	 gripper_state = 0;
//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_RESET);
//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
  }
 }
 else if(registerFrame[0x03].U16==0b0001){
  registerFrame[0x00].U16=22881;
  if(Reed_out == 0){
	 registerFrame[0x00].U16=22881;
	 gripper_state = 1;
//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_SET);
//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
  }
 }
}

void set_shelves(){
	registerFrame[0x00].U16=22881;
	if(registerFrame[0x01].U16==0b0001){
		registerFrame[0x01].U16=0b0000;
		registerFrame[0x10].U16=0b0001; //z-axis Moving Status
		setshelves_mode = 1;
		x_floor[0] = -1;
		x_floor[1] = -1;
		x_floor[2] = -1;
		x_floor[3] = -1;
		x_floor[4] = -1;
		z_floor[0] = -1;
		z_floor[1] = -1;
		z_floor[2] = -1;
		z_floor[3] = -1;
		z_floor[4] = -1;
		z_status = 1;
	}
	if(setshelves_mode == 1){
		//Call every 0.1 s
		registerFrame[0x00].U16=22881;
		static uint64_t timestamp = 0;
		int64_t currentTime = micros();
		if(currentTime > timestamp)
		{
			timestamp =currentTime + 100000;//us
			QEIEncoderPosVel_Update();
		}
		update_position();
		setposition = z_target_position;
		Error = setposition - position;
		Vfeedback = arm_pid_f32(&PID, Error);
		if (fabsf(Error) <= 0.1){
			Vfeedback = 0;
		}
		if(Vfeedback <= 4.3 && Vfeedback > 0){
			Vfeedback = 4.3;
		}
		if(Vfeedback >= -1.75 && Vfeedback < 0){
			Vfeedback = -1.78;
		}
		if(Vfeedback >= 24){
			Vfeedback = 24;
		}
		setMotor();
		if(z_floor[0] != -1 && z_floor[1] != -1 && z_floor[2] != -1 && z_floor[3] != -1 && z_floor[4] != -1){
			registerFrame[0x23].U16 = z_floor[0]*10;
			registerFrame[0x24].U16 = z_floor[1]*10;
			registerFrame[0x25].U16 = z_floor[2]*10;
			registerFrame[0x26].U16 = z_floor[3]*10;
			registerFrame[0x27].U16 = z_floor[4]*10;
			setshelves_mode = 0;
			z_status = 0;
			registerFrame[0x10].U16=0b0000;
		}
	}
}

void run_pointmode(){
	if(registerFrame[0x01].U16==0b1000){
		registerFrame[0x01].U16=0b0000;
		registerFrame[0x10].U16=0b00010000; //Go point
		setposition = (registerFrame[0x30].U16)/10.0;
		z_status = 16;
		point_mode = 1;
	}
	if(point_mode == 1){
		if(fabsf(setposition-position) > 1 || trajec_state == 1 ){
			trajectory();
			static uint64_t timestamp_velocity_PID = 0;
			uint64_t currentTime = micros();
			if(currentTime > timestamp_velocity_PID)
			{
				timestamp_velocity_PID = currentTime + 1000;//us
				velocity_PID();
				Vfeedback = velocity_PID_output;
				setMotor();
			}

			static uint64_t timestamp_position_PID = 0;
			currentTime = micros();
			if(currentTime > timestamp_position_PID)
			{
				timestamp_position_PID = currentTime + 5000;//us
				position_PID();
			}
		}
		else if((setposition-position) >= 0.01){
			Vfeedback = 3;
			setMotor();
		}
		else if((setposition-position) <= -0.01){
			Vfeedback = -2.5;
			setMotor();
		}
		else if(fabsf(setposition-position) < 0.01){
			velocity_PID_output = 0;
			Vfeedback = 0;
			setMotor();
			registerFrame[0x10].U16=0b0000;
			z_status = 0;
			point_mode = 0;
		}
	}
}

void set_home(){
	static uint8_t step = 0;
	if(registerFrame[0x01].U16==0b0010){
		registerFrame[0x01].U16=0b0000;
		registerFrame[0x10].U16=0b0010;
		home = 1;
		z_status = 2;
		step = 0;
		if(sensor_down == 1){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999*1.5/10.0);
		}

	}
	if(home == 1){
		registerFrame[0x00].U16=22881;
		sensor_up = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
		sensor_down = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);

		if(step == 0){
			if(sensor_up == 0){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999*3/10.0);
			}
			else if(sensor_up == 1){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				step = 1;
			}
		}
		if(step == 1){
			if(sensor_down == 0){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 999*1.5/10.0);
			}
			else if(sensor_down == 1){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COUNTER(&htim4, 0);
				QEIdata.Position[NEW] = 0;
				QEIEncoderPosVel_Update();
				angular_position = 0.0;
				position = 0.0;
				position_round = 0;
				registerFrame[0x10].U16=0b0000;
				z_status = 0;
				home = 0;
			}
		}
	}
}

void run_jogmode(){
	static int floor_counter = 0;
	static uint8_t floor_end = 0;
	if(registerFrame[0x01].U16==0b0100){
		registerFrame[0x01].U16=0b0000;
		jog_mode = 1;
		pick_order = registerFrame[0x21].U16;
		place_order = registerFrame[0x22].U16;
	}
	if(jog_mode == 1){
		if(floor_counter%2 == 0){
			registerFrame[0x10].U16=0b0100;//Go Pick
			z_status = 4;
			gotofloor = (pick_order/(10000/(int)pow(10,(floor_counter/2))))%10;
		}
		else{
			registerFrame[0x10].U16=0b1000;//Go place
			z_status = 8;
			gotofloor = (place_order/(10000/(int)pow(10,(floor_counter/2))))%10;
		}
		if (gotofloor == 1){
			setposition = z_floor[0];
			x_position = x_floor[0];
		}
		else if (gotofloor == 2){
			setposition = z_floor[1];
			x_position = x_floor[1];
		}
		else if (gotofloor == 3){
			setposition = z_floor[2];
			x_position = x_floor[2];
		}
		else if (gotofloor == 4){
			setposition = z_floor[3];
			x_position = x_floor[3];
		}
		else if (gotofloor == 5){
			setposition = z_floor[4];
			x_position = x_floor[4];
		}

		if(fabsf(setposition-position) > 1 || trajec_state == 1 ){
			trajectory();
			static uint64_t timestamp_velocity_PID = 0;
			uint64_t currentTime = micros();
			if(currentTime > timestamp_velocity_PID)
			{
				timestamp_velocity_PID = currentTime + 1000;//us
				velocity_PID();
				Vfeedback = velocity_PID_output;
				setMotor();
			}

			static uint64_t timestamp_position_PID = 0;
			currentTime = micros();
			if(currentTime > timestamp_position_PID)
			{
				timestamp_position_PID = currentTime + 5000;//us
				position_PID();
			}
		}
		else if((setposition-position) >= 0.01 && floor_end == 0){
			Vfeedback = 3;
			setMotor();
		}
		else if((setposition-position) <= -0.01 && floor_end == 0){
			Vfeedback = -2.5;
			setMotor();
		}
		else if(fabsf(setposition-position) < 0.01){
			velocity_PID_output = 0;
			Vfeedback = 0;
			floor_end = 1;
			setMotor();
		}
		if(floor_end == 1){

			static uint64_t timestamp_gripper = 0;
			uint64_t currentTime = micros();
			static uint8_t time_counter = 0;
			if(currentTime > timestamp_gripper){
				timestamp_gripper = micros()+2000000;
				if(time_counter == 0){
					time_counter = 1;
				}
				else{
					floor_counter++;

					time_counter = 0;
					floor_end = 0;
				}
			}
		}
		if(floor_counter > 9){
				registerFrame[0x10].U16=0b0000;
				floor_counter = 0;
				z_status = 0;
				jog_mode = 0;
				debug++;

		}
	}

}

void setMotor()
{

	if(Vfeedback > 24){
		registerFrame[0x00].U16=22881;
		Vfeedback = 24;
	}
	else if(Vfeedback < -24){
		registerFrame[0x00].U16=22881;
		Vfeedback = -24;
	}
	if(Vfeedback > 0){
		registerFrame[0x00].U16=22881;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int)Vfeedback*999/24.0);
	}
	else{
		registerFrame[0x00].U16=22881;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int)Vfeedback*(-999)/24.0);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
		{
			//(for string only) Add string stop symbol \0 to end string
			RxBuffer[9] = '\0';
		}
}

void UARTDMAConfig()
{
	//start UART in DMA mode
	HAL_UART_Receive_DMA(&huart1, RxBuffer,9);
}

void update_position()
{
	static int floor = -1;
	floor = RxBuffer[6]-'0'-1;
	x_target_position = (RxBuffer[0]-'0')*100+(RxBuffer[1]-'0')*10+(RxBuffer[2]-'0');
	z_target_position = (RxBuffer[3]-'0')*100+(RxBuffer[4]-'0')*10+(RxBuffer[5]-'0');
	if(floor >= 0){
		x_floor[floor] = x_target_position;
		z_floor[floor] = z_target_position;
	}
}

//MicroSecondTimer Implement
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim5)
	{
		_micros += UINT32_MAX;
	}
	if(htim == &htim6)
	{
		debug++;
		registerFrame[0x00].U16 = 22881;
		registerFrame[0x10].U16 = z_status;
		registerFrame[0x11].U16 = (int)(position*10);
		registerFrame[0x12].U16 = abs((int)(velocity*10));
		registerFrame[0x13].U16 = abs((int)(acceleration*10));
		registerFrame[0x40].U16 = (int)(x_position*10);
	}
}

uint64_t micros()
{
	return __HAL_TIM_GET_COUNTER(&htim5)+_micros;
}

void QEIEncoderPosVel_Update()
{
	//collect data
	QEIdata.TimeStamp[NEW] = micros();
	QEIdata.Position[NEW] = __HAL_TIM_GET_COUNTER(&htim4);
	//Postion 1 turn calculation
	QEIdata.QEIPostion_1turn = QEIdata.Position[NEW] % 2048;
	//calculate dx
	int32_t diffPosition = QEIdata.Position[NEW] - QEIdata.Position[OLD];
	//Handle Warp around
	if(diffPosition > 31744){
		diffPosition -=63488;
		counter--;
	}
	if(diffPosition < -31744){
		diffPosition +=63488;
		counter++;
	}
	//calculate dt
	float diffTime = (QEIdata.TimeStamp[NEW]-QEIdata.TimeStamp[OLD]) * 0.000001;
	//calculate anglar velocity
	QEIdata.QEIAngularVelocity = diffPosition / diffTime;

	angular_velocity = QEIdata.QEIAngularVelocity/2048.0*2*M_PI;
	angular_position = (QEIdata.Position[NEW]%2048)/2048.0*2*M_PI;
	position_round = (counter*31)+(int)(QEIdata.Position[NEW]/2048.0);
	position = ((angular_position)/(2.0*M_PI)*14)+(14*position_round);
	velocity = angular_velocity*14/2.0/M_PI;
	QEIdata.Velocity[NEW] = velocity;
	float diffVelocity = QEIdata.Velocity[NEW] - QEIdata.Velocity[OLD];
	acceleration = diffVelocity / diffTime;

	//store value for next loop
	QEIdata.Position[OLD] = QEIdata.Position[NEW];
	QEIdata.Velocity[OLD] = QEIdata.Velocity[NEW];
	QEIdata.TimeStamp[OLD]=QEIdata.TimeStamp[NEW];

}

void trajectory(){
	static uint32_t Timestamp;
	if(trajec_target != 0 && trajec_state == 0){
		trajec_state = 1;
		Timestamp = HAL_GetTick();
	}
	else if(trajec_state == 1 && trajec_target >= 0){
		float t = ((HAL_GetTick() - Timestamp)*0.001)+0.001;
		float time = (-100 + sqrt(10000 + (2000*trajec_target)))/1000;
		if(HAL_GetTick() - Timestamp <= (time*1000)){
			trajec_acceleration = 500.0;
			trajec_velocity = 500*t;
			trajec_position = (250*t*t)+p0;
		}
		else if(HAL_GetTick() - Timestamp <= ((time+0.2)*1000)){
			trajec_acceleration = 0;
			trajec_velocity = 500*time;
			trajec_position = ((500*time*(t-time))+(250*time*time))+p0;
		}
		else if(HAL_GetTick() - Timestamp <= (((time*2)+0.2)*1000)){
			trajec_acceleration = -500.0;
			trajec_velocity = (-500*(t-time-0.2))+(500*time);
			trajec_position = ((-250*(t-time-0.2)*(t-time-0.2))+(500*time*(t-time-0.2))+(250*time*time)+(100*time))+p0;
		}
		else{
			trajec_acceleration = 0;
			trajec_velocity = 0;
//			trajec_position = 0;
			trajec_target = 0;
			trajec_state = 0;
		}
	}
	else if(trajec_state == 1 && trajec_target < 0){
		float t = ((HAL_GetTick() - Timestamp)*0.001+0.001);
		float time = (-100 + sqrt(10000 + (-2000*trajec_target)))/1000;
		if(HAL_GetTick() - Timestamp <= (time*1000)){
			trajec_acceleration = -500.0;
			trajec_velocity = (500*t)*-1;
			trajec_position = ((250*t*t)*-1)+p0;
		}
		else if(HAL_GetTick() - Timestamp <= ((time+0.2)*1000)){
			trajec_acceleration = 0;
			trajec_velocity = (500*time)*-1;
			trajec_position = (((500*time*(t-time))+(250*time*time))*-1)+p0;
		}
		else if(HAL_GetTick() - Timestamp <= (((time*2)+0.2)*1000)){
			trajec_acceleration = 500.0;
			trajec_velocity = ((-500*(t-time-0.2))+(500*time))*-1;
			trajec_position = (((-250*(t-time-0.2)*(t-time-0.2))+(500*time*(t-time-0.2))+(250*time*time)+(100*time))*-1)+p0;
		}
		else{
			trajec_acceleration = 0;
			trajec_velocity = 0;
//			trajec_position = 0;
			trajec_target = 0;
			trajec_state = 0;
		}
	}
	else if(trajec_state == 0 && trajec_target == 0){
		trajec_target = setposition-position;
		p0 = position;
	}
}

void velocity_PID(){
	static float u_n;
	static float u_n1 = 0;
	static float u_n2 = 0;
	static float y_n;
	static float y_n1 = 0;
	float one = (2*velocity_Ts*velocity_Kp)+(velocity_Ki*velocity_Ts*velocity_Ts)+(2*velocity_Kd);
	float two = (-2*velocity_Ts*velocity_Kp)+(velocity_Ki*velocity_Ts*velocity_Ts)-(4*velocity_Kd);
	float three = 2*velocity_Kd;
	float four = 2*velocity_Ts;
	u_n = trajec_velocity + position_PID_output - velocity;
	y_n = ((one*u_n)+(two*u_n1)+(three*u_n2)+(four*y_n1))/four;

	velocity_PID_output += y_n;
	u_n2 = u_n1;
	u_n1 = u_n;
	y_n1 = y_n;
}

void position_PID(){
	static float u_n;
	static float u_n1 = 0;
	static float u_n2 = 0;
	static float y_n;
	static float y_n1 = 0;
	float one = (2*position_Ts*position_Kp)+(position_Ki*position_Ts*position_Ts)+(2*position_Kd);
	float two = (-2*position_Ts*position_Kp)+(position_Ki*position_Ts*position_Ts)-(4*position_Kd);
	float three = 2*position_Kd;
	float four = 2*position_Ts;
	u_n = trajec_position - position;
	y_n = ((one*u_n)+(two*u_n1)+(three*u_n2)+(four*y_n1))/four;

	position_PID_output += y_n;
	u_n2 = u_n1;
	u_n1 = u_n;
	y_n1 = y_n;
}
void gripper_pick(){


}
void gripper_place(){

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

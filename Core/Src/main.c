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
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//adresa LCD monitora u port extenderu i duzina bafera za serijsku komunikaciju
#define SLAVE_ADDRESS_LCD 0x4E // change this according to setup
#define UART_RX_BUFFER_SIZE  40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint32_t TxMailbox;
uint8_t TxData[8];
uint8_t RxData[8];

uint8_t hello_msg[] = "Hello from PL2303\r\n";
uint8_t RxChar;
uint8_t UART1_RxBuffer[UART_RX_BUFFER_SIZE] = {0};
uint16_t RxDataLen = 0;
uint8_t hexValue[20];

int lcd_thermistor = 1;
int16_t light_goal = 100;
int16_t light_now = 100;
uint8_t numChar[40];
char msg[100];

//podesavanja prisluskivaca po poziciji bita
//1 - Pojednostavljen CAN okvir
//2 - Interpretiranje komandi sa terminala
//3 - Interpretiranje CAN poruka
//4 - Izvor kontolisanja LED dioda, potenciometar(1) ili termistori(0)
//5 - LCD prikaz on/off
//6 - TBA (to be added)
//7 - TBA
//8 - TBA
//
//komande za promenu podesavanja
//$0XX - sniffer_setting = 0xXX
//$10X - pojednostavljen CAN on/off (for X = 1/0)
//$20X - Interpretacija sa terminala on/off
//$30X - Interpretacija sa CAN-a on/off
//$40X - Izvor kontrolisanja LED dioda = X
//$50X - LCD prikaz on/off
uint8_t sniffer_setting = 0xFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void ADC_Read(int16_t* pValue1, int16_t* pValue2);
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_init (void);
void lcd_send_string (char *str);
void lcd_put_cur(int row, int col);
uint8_t ASCII_To_Hex(uint8_t ascii_char);
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
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //inicijaliyacija tajmera za PWM kontrolu dioda
  //pocetno sve diode ugasene
  TIM1->CCR1 = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  TIM2->CCR1 = 0;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  TIM3->CCR1 = 0;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  //inicijalizacija LCD ekrana sa pocetnom porukom
  lcd_init();
  lcd_put_cur(0, 0);
  lcd_send_string ("Hello");

  //podesavanje CAN filtra
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 12;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  //canfilterconfig.FilterIdHigh = 0x103<<5;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  //canfilterconfig.FilterMaskIdHigh = 0x1<<13;
  canfilterconfig.FilterMaskIdHigh = 0x0;
  canfilterconfig.FilterMaskIdLow = 0x0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assigned to CAN1
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(
          &hcan,
		  CAN_IT_TX_MAILBOX_EMPTY |
          CAN_IT_RX_FIFO0_MSG_PENDING |
          CAN_IT_RX_FIFO0_FULL |
          CAN_IT_RX_FIFO0_OVERRUN |
          CAN_IT_RX_FIFO1_MSG_PENDING |
          CAN_IT_RX_FIFO1_FULL |
          CAN_IT_RX_FIFO1_OVERRUN |
          CAN_IT_WAKEUP |
          CAN_IT_SLEEP_ACK |
          CAN_IT_ERROR_WARNING |
          CAN_IT_ERROR_PASSIVE |
          CAN_IT_BUSOFF |
          CAN_IT_LAST_ERROR_CODE |
          CAN_IT_ERROR
  );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //promenljive za kvazi-tajmere
  //sluze za izvrsavanje odredjenih delova koda svaki sekund bez prekidne rutine
  uint32_t now = 0, last_blink = 0, last_tx = 0;

  //promenljive za debouncing tastera
  uint8_t block = 0;
  uint8_t release = 0;

  //nakon inicijalizacija napravi pauzu 1s i pokreni serijsku komunikaciju
  HAL_Delay(1000);
  HAL_UART_Transmit(&huart1, hello_msg, sizeof(hello_msg), 1000);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuffer, UART_RX_BUFFER_SIZE);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  now = HAL_GetTick();

	  //pritisnuto dugme
	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
	  {
		  block++;
	  }
	  else //pusteno dugme
	  {
		  block = 0;
		  release++;
	  }

	  //ako je dugme dovoljno dugo pritisnuto i proslo dovoljno od proslog pritiska
	  if(block >= 10 && release >= 20)
	  {
		  //promeni koji termistor se priakzuje na LCDu
		  lcd_thermistor  = (lcd_thermistor % 2) + 1;
		  release = 0;

		  //posalji CAN poruku da se zatrazi vrednost drugog termistora
		  TxHeader.DLC = 0;
		  TxHeader.ExtId = 0;
		  TxHeader.IDE = CAN_ID_STD;
		  TxHeader.RTR = CAN_RTR_REMOTE;
		  if(lcd_thermistor == 1) TxHeader.StdId = 0x601;
		  else if(lcd_thermistor == 2) TxHeader.StdId = 0x602;
		  if((sniffer_setting & 0x02) == 0x02){
			  //ako je ukljucena interpretacija poruka sa terminala
			  //ispisi da je dugme pritisnuto i bitne podatke CAN poruke
			  sprintf(msg, "\nButton pressed\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			  sprintf(msg, "StdId: 0x%03X\r\n", TxHeader.StdId);
			  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			  sprintf(msg, "RTR: 0x%01X\r\n", TxHeader.RTR);
			  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			  sprintf(msg, "DLC: 0x%02X\r\n", TxHeader.DLC);
			  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		  }
	  	  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  	  {
	  		  Error_Handler();
	  	  }
	  }

	  if (now - last_tx >= 10) {// svakih 10ms
		  if((light_goal - light_now) > 1){
			  //ako je ciljano osvetljenje vece od trenutnog povecaj trenutno u koracima
			  light_now = light_now + 1;
		  }
		  else if((light_goal - light_now) < -1){
			  //ako je ciljano osvetljenje manje od trenutnog smanji trenutno u koracima
			  light_now = light_now - 1;
		  }
		  if (light_now > 300) light_now = 300; //maksimum osvetljenja dioda
		  else if (light_now < 0) light_now = 0; //minimum osvetljenja dioda

		  //osvetljenje prve diode
		  if(light_now <= 100) TIM1->CCR1 = pow(light_now, 0.8);
		  else if(light_now > 100) TIM1->CCR1 = 100;
		  else TIM1->CCR1 = 0;

		  //osvetljenje druge diode
		  if(light_now <= 200 && light_now > 100) TIM2->CCR1 = pow(light_now-100, 0.8);
		  else if(light_now > 200) TIM2->CCR1 = 100;
		  else TIM2->CCR1 = 0;

		  //osvetljenje trece diode
		  if(light_now <= 300 && light_now > 200) TIM3->CCR1 = pow(light_now-200, 0.8);
		  else if(light_now > 300) TIM3->CCR1 = 100;
		  else TIM3->CCR1 = 0;
	      last_tx = now;
	  }

	  if (now - last_blink >= 500) {
		  last_blink = now;
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
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
  sConfigOC.Pulse = 0;
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
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//prekidna rutina za pristigle CAN poruke
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint16_t adcValue;	//privremena promenljiva za smestanje ADC vrednosti
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //upali integrisanu diodu ako je stigla poruka

    	if((sniffer_setting & 0x01) == 0x01){
    		//ispisi pristiglu CAN poruku u pojednostavljenom zapisu
    		sprintf(msg, "\nReceived CAN message: %x%x", ((RxHeader.StdId << 1) | RxHeader.RTR), RxHeader.DLC);
    	}
    	else{
    		//ispisi pristiglu CAN poruku samo bez CRC i ACK polja
    		sprintf(msg, "\nReceived CAN message: %x",
    		(((((0x01 << 11) | RxHeader.StdId) << 6) | (RxHeader.IDE << 5) | (0x01 << 4) | RxHeader.DLC) << 1) | (RxData[0] >> 7));
    	}
    	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    	for(int brojac = 0; brojac < RxHeader.DLC; brojac++){
    		//ispisi svako polje podataka na osnovu duzine dobijene DLC poljem
    		if((sniffer_setting & 0x01) == 0x01){
    			//ispis za pojednostavljeni zapis
    			if(RxData[brojac] > 0x0F) sprintf(msg, "%x", RxData[brojac]);
    			else sprintf(msg, "0%x", RxData[brojac]);
    		}
    		else{
    			//ispis za produzeni zapis
    			if(((RxData[brojac] << 1) | RxData[brojac+1] >> 7) > 0x0F){
    				sprintf(msg, "%x", (RxData[brojac] << 1) | RxData[brojac+1] >> 7);
    			}
    			else{
    				sprintf(msg, "0%x", (RxData[brojac] << 1) | RxData[brojac+1] >> 7);
    			}
    		}
    		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    	}
    	sprintf(msg, "\n");
    	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    	if(RxHeader.StdId == 0x400)
    	{
    		//ako je stigla poruka sa ADC vrednostima i stanjem diode
    		//ispisi sve vrednosti
    		//adcValue = (RxData[0] << 8) | RxData[1];
    		if((sniffer_setting & 0x04) == 0x04){
    			sprintf(numChar, "Potentiometer: %d\n", (RxData[0] << 8) | RxData[1]);
        		HAL_UART_Transmit(&huart1, numChar, sizeof(numChar), 1000);
    		}
    		//adcValue = (RxData[2] << 8) | RxData[3];
    		if((sniffer_setting & 0x04) == 0x04){
    			sprintf(numChar, "Thermistor 1: %d\n", (RxData[2] << 8) | RxData[3]);
        		HAL_UART_Transmit(&huart1, numChar, sizeof(numChar), 1000);
    		}
    		//adcValue = (RxData[4] << 8) | RxData[5];
    		if((sniffer_setting & 0x04) == 0x04){
    			sprintf(numChar, "Thermistor 2: %d\n", (RxData[4] << 8) | RxData[5]);
        		HAL_UART_Transmit(&huart1, numChar, sizeof(numChar), 1000);
    		}
    		//adcValue = RxData[6];
    		if((sniffer_setting & 0x04) == 0x04){
    			sprintf(numChar, "LED state: %d\n", RxData[6]);
        		HAL_UART_Transmit(&huart1, numChar, sizeof(numChar), 1000);
    		}
    	}
    	else if(RxHeader.StdId == 0x401)
    	{
    		//poruka sa stanjem potenciometra
    		//adcValue = (RxData[0] << 8) | RxData[1];
    		if((sniffer_setting & 0x04) == 0x04){
    			sprintf(numChar, "Potentiometer: %d\n", (RxData[0] << 8) | RxData[1]);
        		HAL_UART_Transmit(&huart1, numChar, sizeof(numChar), 1000);
    		}
    	}
    	else if(RxHeader.StdId == 0x402)
    	{
    		//poruka sa stanjem prvog termistora
    		//adcValue = (RxData[0] << 8) | RxData[1];
    		if((sniffer_setting & 0x04) == 0x04){
    			sprintf(numChar, "Thermistor 1: %d\n", (RxData[0] << 8) | RxData[1]);
        		HAL_UART_Transmit(&huart1, numChar, sizeof(numChar), 1000);
    		}
    	}
    	else if(RxHeader.StdId == 0x403)
    	{
    		//poruka sa stanjem drugog termistora
    		//adcValue = (RxData[0] << 8) | RxData[1];
    		if((sniffer_setting & 0x04) == 0x04){
    			sprintf(numChar, "Thermistor 2: %d\n", (RxData[0] << 8) | RxData[1]);
        		HAL_UART_Transmit(&huart1, numChar, sizeof(numChar), 1000);
    		}
    	}
    	else if(RxHeader.StdId == 0x404)
    	{
    		//poruka sa stanjem diode
    		//adcValue = (RxData[0] << 8) | RxData[1];
    		if((sniffer_setting & 0x04) == 0x04){
    			sprintf(numChar, "LED state: %d\n", RxData[0]);
        		HAL_UART_Transmit(&huart1, numChar, sizeof(numChar), 1000);
    		}
    	}
    	else if(RxHeader.StdId == 0x605){
    		//prikaz na LCDu svakih 1s
    		if((sniffer_setting & 0x08) == 0x08){
    			//diode se kontrolisu potenciometrom
    			light_goal = 300*((RxData[0] << 8) | RxData[1])/4096;
    			adcValue = (RxData[2] << 8) | RxData[3];
    		}
    		else{
    			//diode se kontrolisu razlikom na termistorima
    			light_goal = 2*(((RxData[0] << 8) | RxData[1]) - ((RxData[2] << 8) | RxData[3]));
    			if(lcd_thermistor == 1) adcValue = (RxData[0] << 8) | RxData[1];
    			else adcValue = (RxData[2] << 8) | RxData[3];
    		}

    		//ispis stanja termistora na LCDu
    		if((sniffer_setting & 0x10) == 0x10){
    			sprintf(numChar, "Thermistor %d", lcd_thermistor);
        		lcd_put_cur(0, 0);
        		lcd_send_string (numChar);
        		sprintf(numChar, "%4d", adcValue);
        		lcd_put_cur(1, 0);
        		lcd_send_string (numChar);
    		}

    	}
    }
}

//prekidna rutina za poruke sa serijske komunikacije
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	sprintf(msg, "\nReceived Command: %s\n", UART1_RxBuffer);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	//ako se salje CAN paket
	if(UART1_RxBuffer[0] == '#'){
		//protumaci sve karaktere u poruci i prebaci ih u HEX vrednosti
		for(int brojac = 1; brojac<Size; brojac++){
			hexValue[brojac-1] = ASCII_To_Hex(UART1_RxBuffer[brojac]);
		}

		//sastavi CAN poruku
		//uint16_t arb_field = ((hexValue[0] << 8) | (hexValue[1] << 4) | hexValue[2]);
		TxHeader.StdId = ((hexValue[0] << 8) | (hexValue[1] << 4) | hexValue[2]) >> 1;
		TxHeader.RTR = ((hexValue[0] << 8) | (hexValue[1] << 4) | hexValue[2]) & 0x01;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.ExtId = 0;
		TxHeader.DLC = hexValue[3];
		if((sniffer_setting & 0x02) == 0x02){
			//ako je ukljucena interpretacija prikayi sastavljenu poruku
			sprintf(msg, "StdId: 0x%03X\r\n", TxHeader.StdId);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			sprintf(msg, "RTR: 0x%01X\r\n", TxHeader.RTR);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			sprintf(msg, "DLC: 0x%02X\r\n", TxHeader.DLC);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		if(TxHeader.RTR == 0){
			//ako je poruka sa podacima onda sastavi i taj deo poruke
			for(int brojac = 0; brojac < TxHeader.DLC; brojac++){
				TxData[brojac] = (hexValue[4+brojac*2] << 4) | (hexValue[5+brojac*2]);
				if((sniffer_setting & 0x02) == 0x02){
					//ako je ukljucena interpretacija poruka prikazi sastavljene podatke
					sprintf(msg, "TxData[%d]: 0x%02X\r\n", brojac, TxData[brojac]);
			    	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
		    }
		}
		TxHeader.TransmitGlobalTime = DISABLE;
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		{
		   	Error_Handler();
		}
	}

	//ako je poruka za promenu podesavanja Sniffera
	else if(UART1_RxBuffer[0] == '$'){
		//protumaci sve karaktere u poruci i prebaci ih u HEX vrednosti
		for(int brojac = 1; brojac<Size; brojac++){
			hexValue[brojac-1] = ASCII_To_Hex(UART1_RxBuffer[brojac]);
		}

		//ako je komanda za podesavanje svih vrednosti od jednom
		if(hexValue[0] == 0x00){
			sniffer_setting = (hexValue[1] << 4) | hexValue[2];
			sprintf(msg, "Settings: 0x%02X\r\n", sniffer_setting);
	    	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	    	if((sniffer_setting & 0x10) == 0x00){
	    		//ako treba da se iskljuci prikaz na monitoru ispisi prazna mesta
	    		sprintf(numChar, "                ");
        		lcd_put_cur(0, 0);
        		lcd_send_string (numChar);
        		lcd_put_cur(1, 0);
        		lcd_send_string (numChar);
	    	}
	    	//sastavi CAN poruku gde se salje vrednost 4. bita
	    	TxHeader.StdId = 0x301;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.ExtId = 0;
			TxHeader.DLC = 1;
			TxData[0] = (sniffer_setting >> 3) & 0x01;
			TxHeader.TransmitGlobalTime = DISABLE;
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
			   	Error_Handler();
			}
		}

		//ako je definisan koji bit se menja postavi ga na upisanu vrednost
		if(hexValue[0] == 0x01){
			if(hexValue[2] == 0x01) sniffer_setting |= 0x01;
			else if(hexValue[2] == 0x00) sniffer_setting &= 0xFE;
		}
		if(hexValue[0] == 0x02){
			if(hexValue[2] == 0x01) sniffer_setting |= 0x02;
			else if(hexValue[2] == 0x00) sniffer_setting &= 0xFD;
		}
		if(hexValue[0] == 0x03){
			if(hexValue[2] == 0x01) sniffer_setting |= 0x04;
			else if(hexValue[2] == 0x00) sniffer_setting &= 0xFB;
		}
		if(hexValue[0] == 0x04){
			if(hexValue[2] == 0x01) sniffer_setting |= 0x08;
			else if(hexValue[2] == 0x00) sniffer_setting &= 0xF7;
			//ako se menja 4. bit posalji njegovu vrednost u CAN poruci
			TxHeader.StdId = 0x301;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.ExtId = 0;
			TxHeader.DLC = 1;
			TxData[0] = hexValue[2];
			TxHeader.TransmitGlobalTime = DISABLE;
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
			   	Error_Handler();
			}
		}
		if(hexValue[0] == 0x05){
			if(hexValue[2] == 0x01) sniffer_setting |= 0x10;
			else if(hexValue[2] == 0x00){
				//ako treba da se iskljuci prikaz na monitoru ispisi prazna mesta
				sniffer_setting &= 0xEF;
				sprintf(numChar, "                ");
        		lcd_put_cur(0, 0);
        		lcd_send_string (numChar);
        		lcd_put_cur(1, 0);
        		lcd_send_string (numChar);
			}
		}
	}

	//ako prvi karakter nije ni # ni $ onda poruka nije dobrog formata
	else{
		sprintf(msg, "Invalid message type\nMessages begin with # for CAN frame or $ for Sniffer setup\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}

	fflush(UART1_RxBuffer);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuffer, UART_RX_BUFFER_SIZE);
}

//funckije za inicijalizaciju i kontrolisanje LCD ekrana
void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
  data_u = (cmd&0xf0);
  data_l = ((cmd<<4)&0xf0);
  uint8_t data_t[4];
  data_t[0] = data_u|0x0C;  //en=1, rs=0 -> bxxxx1100
  data_t[1] = data_u|0x08;  //en=0, rs=0 -> bxxxx1000
  data_t[2] = data_l|0x0C;  //en=1, rs=0 -> bxxxx1100
  data_t[3] = data_l|0x08;  //en=0, rs=0 -> bxxxx1000
  HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[1] = data_u|0x09;  //en=0, rs=1 -> bxxxx1001
	data_t[2] = data_l|0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[3] = data_l|0x09;  //en=0, rs=1 -> bxxxx1001
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void)
{
  // 4 bit initialisation
  HAL_Delay(50);  // wait for >40ms
  lcd_send_cmd (0x30);
  HAL_Delay(5);  // wait for >4.1ms
  lcd_send_cmd (0x30);
  HAL_Delay(1);  // wait for >100us
  lcd_send_cmd (0x30);
  HAL_Delay(10);
  lcd_send_cmd (0x20);  // 4bit mode
  HAL_Delay(10);

  // display initialisation
  lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
  HAL_Delay(1);
  lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
  HAL_Delay(1);
  lcd_send_cmd (0x01);  // clear display
  HAL_Delay(2);
  lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  HAL_Delay(1);
  lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
  while (*str) lcd_send_data (*str++);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd (col);
}

//funckija za konverziju karaktera u HEX vrednost koju on predstavlja
uint8_t ASCII_To_Hex(uint8_t ascii_char)
{
    if (ascii_char >= '0' && ascii_char <= '9')
    {
        return ascii_char - '0';  // '0' to '9' are mapped to 0x0 to 0x9
    }
    else if (ascii_char >= 'A' && ascii_char <= 'F')
    {
        return ascii_char - 'A' + 10;  // 'A' to 'F' are mapped to 0xA to 0xF
    }
    else if (ascii_char >= 'a' && ascii_char <= 'f')
    {
        return ascii_char - 'a' + 10;  // 'a' to 'f' are mapped to 0xA to 0xF
    }
    else
    {
        return 0xFF;  // Invalid hex character
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

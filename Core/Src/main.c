#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/*-----Variables for controller-----*/
double T = 0.01;

double set1 = 0;
double N1, preN1;
double y11, y12, y13, y14, ym11, ym12, ym13;
double ec11, ec12, ec13, em11, em12, em13;
double delkp11, delkp12, delkp13;
double delki11, delki12, delki13;
double Kp1, Ki1;
double gammap1 = 0.0095, gammai1 = 0.0095;
double alpha1, beta1;
double u11, u12;

double set2 = 0;
double N2, preN2;
double y21, y22, y23, y24, ym21, ym22, ym23;
double ec21, ec22, ec23, em21, em22, em23;
double delkp21, delkp22, delkp23;
double delki21, delki22, delki23;
double Kp2, Ki2;
double gammap2 = 0.0095, gammai2 = 0.0095;
double alpha2, beta2;
double u21, u22;

double pitch = 0, roll = 0, yaw = 0, yaw_rad = 0;

/*-----Variables for UART2-----*/
#define UART2_BUFF_SIZE 25
char rx2_data[1], rx2_buffer[UART2_BUFF_SIZE];
int rx2_indx;
int send_enable = 0;
char tx2_data[UART2_BUFF_SIZE] = "connected\n";

/*-----Variables for UART1-----*/
#define UART1_BUFF_SIZE 44
char rx1_buffer[2*UART1_BUFF_SIZE];
int startIndx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim6);
	
	HAL_UART_Transmit(&huart2, (uint8_t*)tx2_data, sizeof(tx2_data), 1000);
	tx2_data[3] = '.'; tx2_data[6] = '/'; tx2_data[10] = '.'; tx2_data[13] = '/'; tx2_data[18] = '.';
	tx2_data[21] = '\n';	
	HAL_UART_Receive_IT(&huart2,(uint8_t*)rx2_data,1);
	
	HAL_UART_Receive_DMA(&huart1,(uint8_t*)rx1_buffer,2*UART1_BUFF_SIZE);
	
  while (1)
  {
		
  }
}

/*-----Sampling 10ms with timer6-----*/
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);

	// Left wheel
	N1 = __HAL_TIM_GetCounter(&htim4);
	if(abs((int)(N1 - preN1)) < 40000) y13 = (N1 - preN1)*60.0*100/7392;
	y11 = 0.9753*y12 + 0.02469*y14;
	y12 = y11; y14 = y13;
	preN1 = N1;
	
	ec11 = set1 - y11;
	ym11 = 0.00122*set1 + 0.00078*set1 + 1.921*ym12 - 0.923*ym13;
	em11 = y11 - ym11;
	delkp11 = 1.921*delkp12 - 0.923*delkp13 + 0.07685*ec12 - 0.07685*ec13;
	Kp1 += -gammap1*em11*delkp11;
	delki11 = 1.921*delki12 - 0.923*delki13 + 0.00039*ec12 + 0.00025*ec13;
	Ki1 += -gammai1*em11*delki11;
	
	alpha1 = Kp1*(ec11 - ec12);
	beta1 = T/2*Ki1*(ec11 + ec12);
	u11 = u12 + alpha1 + beta1;
	
	if(u11 < 0)
	{
		if(u11 < -4199) u11 = -4199;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
	}
	else
	{
		if(u11 > 4199) u11 = 4199;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
	}
	
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, u11);
	
	u12 = u11;
	ec13 = ec12; ec12 = ec11; em13 = em12; em12 = em11;	ym13 = ym12; ym12 = ym11;
	delkp13 = delkp12; delkp12 = delkp11; delki13 = delki12; delki12 = delki11;
	
	// Right wheel
	N2 = __HAL_TIM_GetCounter(&htim3);
	if(abs((int)(N2 - preN2)) < 40000) y23 = (N2 - preN2)*60.0*100/7392;
	y21 = 0.9753*y22 + 0.02469*y24;
	y22 = y21; y24 = y23;
	preN2 = N2;
	
	ec21 = set2 - y21;
	ym21 = 0.00122*set2 + 0.00078*set2 + 1.921*ym22 - 0.923*ym23;
	em21 = y21 - ym21;
	delkp21 = 1.921*delkp22 - 0.923*delkp23 + 0.07685*ec22 - 0.07685*ec23;
	Kp2 += -gammap2*em21*delkp21;
	delki21 = 1.921*delki22 - 0.923*delki23 + 0.00039*ec22 + 0.00025*ec23;
	Ki2 += -gammai2*em21*delki21;
	
	alpha2 = Kp2*(ec21 - ec22);
	beta2 = T/2*Ki2*(ec21 + ec22);
	u21 = u22 + alpha2 + beta2;
	
	if(u21 < 0)
	{
		if(u21 < -4199) u21 = -4199;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	}
	else
	{
		if(u21 > 4199) u21 = 4199;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	}
	
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, u21);
	
	u22 = u21;
	ec23 = ec22; ec22 = ec21; em23 = em22; em22 = em21;	ym23 = ym22; ym22 = ym21;
	delkp23 = delkp22; delkp22 = delkp21; delki23 = delki22; delki22 = delki21;
	
	// Sending speed data through UART2
	int temp1 = y11*100;
	int temp2 = y21*100;
	
	tx2_data[0] = (temp1 >= 0)? '+' : '-';
	tx2_data[7] = (temp2 >= 0)? '+' : '-';
	
	temp1 = abs(temp1); temp2 = abs(temp2);
	
	tx2_data[1] = (uint8_t)(temp1/1000) + 48;
	tx2_data[2] = (uint8_t)((temp1 - 1000*(tx2_data[1] - 48))/100) + 48;
	tx2_data[4] = (uint8_t)((temp1 - 1000*(tx2_data[1] - 48) - 100*(tx2_data[2] - 48))/10) + 48;
	tx2_data[5] = (uint8_t)(temp1%10) + 48;
	
	tx2_data[8] = (uint8_t)(temp2/1000) + 48;
	tx2_data[9] = (uint8_t)((temp2 - 1000*(tx2_data[8] - 48))/100) + 48;
	tx2_data[11] = (uint8_t)((temp2 - 1000*(tx2_data[8] - 48) - 100*(tx2_data[9] - 48))/10) + 48;
	tx2_data[12] = (uint8_t)(temp2%10) + 48;
	
	HAL_UART_Transmit(&huart2, (uint8_t*)tx2_data, sizeof(tx2_data), 1000);		
}

/*-----Receive data through UART-----*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{
		if(rx2_indx == 0) for(int i=0;i<UART2_BUFF_SIZE;i++) rx2_buffer[i] = 0;
				
		if(rx2_data[0] != 'k') rx2_buffer[rx2_indx++] = rx2_data[0];
		else
		{
			if(((double)(rx2_buffer[1] - 48) + (double)(rx2_buffer[2] - 48) + (double)(rx2_buffer[4] - 48) + (double)(rx2_buffer[5] - 48)) == ((double)(rx2_buffer[14] - 48)*10 + (double)(rx2_buffer[15] - 48)) 
				|| ((double)(rx2_buffer[8] - 48) + (double)(rx2_buffer[9] - 48) + (double)(rx2_buffer[11] - 48) + (double)(rx2_buffer[12] - 48)) == ((double)(rx2_buffer[17] - 48)*10 + (double)(rx2_buffer[18] - 48))){			
			set1 = (double)(rx2_buffer[1] - 48)*10 + (double)(rx2_buffer[2] - 48) + (double)(rx2_buffer[4] - 48)/10 + (double)(rx2_buffer[5] - 48)/100;
			set2 = (double)(rx2_buffer[8] - 48)*10 + (double)(rx2_buffer[9] - 48) + (double)(rx2_buffer[11] - 48)/10 + (double)(rx2_buffer[12] - 48)/100;
			if(rx2_buffer[0] == '-') set1 = - set1;
			if(rx2_buffer[7] == '-')	set2 = - set2;
		}
			rx2_indx = 0;
		}

		HAL_UART_Receive_IT(&huart2,(uint8_t*)rx2_data,1);
	}
	
	else
	{
		for(int j=0;j<2*UART1_BUFF_SIZE;j++)
		{
			if(rx1_buffer[j] == '\n')
			{
				startIndx = j;
				break;
			}
		}
		tx2_dat		a[14] = rx1_buffer[startIndx + 15];
		tx2_data[15] = rx1_buffer[startIndx + 16];
		tx2_data[16] = rx1_buffer[startIndx + 17];
		tx2_data[17] = rx1_buffer[startIndx + 18];
		tx2_data[19] = rx1_buffer[startIndx + 19];
		tx2_data[20] = rx1_buffer[startIndx + 20];
		
		roll = (double)(rx1_buffer[2]-48)*100 + (double)(rx1_buffer[3]-48)*10 + (double)(rx1_buffer[4]-48) + (double)(rx1_buffer[5]-48)/10 + (double)(rx1_buffer[6]-48)/100;
		if(rx1_buffer[1] == '-') roll = - roll;
		
		pitch = (double)(rx1_buffer[9]-48)*100 + (double)(rx1_buffer[10]-48)*10 + (double)(rx1_buffer[11]-48) + (double)(rx1_buffer[12]-48)/10 + (double)(rx1_buffer[13]-48)/100;
		if(rx1_buffer[8] == '-') pitch = - pitch;
	   
		yaw = (double)(rx1_buffer[16]-48)*100 + (double)(rx1_buffer[17]-48)*10 + (double)(rx1_buffer[18]-48) + (double)(rx1_buffer[19]-48)/10 + (double)(rx1_buffer[20]-48)/100;
		if(rx1_buffer[15] == '-') yaw = - yaw;
		yaw_rad = yaw*3.1415926535/180;
	}
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Period = 0xFFFF;
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
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC15 PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

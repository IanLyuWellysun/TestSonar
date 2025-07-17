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
#include "icm42607t.h"
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
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef ftt1;
FDCAN_TxHeaderTypeDef ftt2;
FDCAN_TxHeaderTypeDef ftt3;
ICM42607T_REGISTER_MAP icm42607trm;
union {
    struct
	{
        uint8_t AXL;
        uint8_t AXH;
        uint8_t AYL;
        uint8_t AYH;
        uint8_t AZL;
        uint8_t AZH;
        uint8_t GXL;
        uint8_t GXH;
        uint8_t GYL;
        uint8_t GYH;
        uint8_t GZL;
        uint8_t GZH;
    };
    struct
	{
    	int16_t AX;
    	int16_t AY;
    	int16_t AZ;
    	int16_t GX;
    	int16_t GY;
    	int16_t GZ;
	};
    uint8_t data8[12];
    int16_t data16[6];
}imuData;

typedef union
{
	struct
	{
		union
		{
			struct
			{
				uint8_t YAW_RATE_L;
				uint8_t YAW_RATE_H;
			};
			uint16_t YAW_RATE;
		};
		uint8_t CLU_STAT:4;
		uint8_t YAW_RATE_STAT:4;
		uint8_t TEMP_RATE_Z;
		union
		{
			struct
			{
				uint8_t AY_L;
				uint8_t AY_H;
			};
			uint16_t AY;
		};
		uint8_t MSG_CNT:4;
		uint8_t AY_STAT:4;
		uint8_t CRC_;
	};
	uint8_t data[8];
}CAN_0x174;

typedef union
{
	struct
	{
		union
		{
			struct
			{
				uint8_t ROLL_RATE_L;
				uint8_t ROLL_RATE_H;
			};
			uint16_t ROLL_RATE;
		};
		uint8_t CLU_STAT5:4;
		uint8_t ROLL_RATE_STAT:4;
		uint8_t CLU_DIAG;
		union
		{
			struct
			{
				uint8_t AX_L;
				uint8_t AX_H;
			};
			uint16_t AX;
		};
		uint8_t MSG_CNT:4;
		uint8_t AX_STAT:4;
		uint8_t CRC_;
	};
	uint8_t data[8];
}CAN_0x178;

typedef union
{
	struct
	{
		union
		{
			struct
			{
				uint8_t PITCH_RATE_L;
				uint8_t PITCH_RATE_H;
			};
			uint16_t PITCH_RATE;
		};
		uint8_t HW_INDEX:4;
		uint8_t PITCH_RATE_STAT:4;
		uint8_t Reserved;
		union
		{
			struct
			{
				uint8_t AZ_L;
				uint8_t AZ_H;
			};
			uint16_t AZ;
		};
		uint8_t MSG_CNT:4;
		uint8_t AZ_STAT:4;
		uint8_t CRC_;
	};
	uint8_t data[8];
}CAN_0x17C;



typedef struct{
	uint32_t timer;
	union
	{
		CAN_0x174 can0x174;
		CAN_0x178 can0x178;
		CAN_0x17C can0x17C;
		uint8_t data8[8];
	};
	bool lose;
}CAN_MESSAGE;

CAN_MESSAGE canMessage0x174;
CAN_MESSAGE canMessage0x178;
CAN_MESSAGE canMessage0x17C;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void SPI2SWNSSTransmitReceive(uint8_t *txData,uint8_t *rxData,uint16_t length);
void CanOutput(void);
void ReadImuICM42607T(void);
void ConfigICM42607T(void);
void CanInitial(void);
uint16_t GetRealAccelerometer(int16_t inputData);
uint16_t GetRealGyroscope(int16_t inputData);
uint16_t GetRealGx(int16_t inputData);
uint8_t Crc8Saej1850(uint8_t *data,int length);
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
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
  {
	  Error_Handler();
  }
  CanInitial();
  ConfigICM42607T();
  HAL_TIM_Base_Start_IT(&htim6);
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
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 20;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 14;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint8_t Crc8Saej1850(uint8_t *data,int length)
{
	uint8_t t_crc;
    t_crc = 0xFF;//Initial value 0xFF
    for (int f = 0; f < length; f++)
    {
    	t_crc ^= data[f];
        for (int b = 0; b < 8; b++)
        {
            if ((t_crc & 0x80) != 0)
            {
                t_crc <<= 1;
                t_crc ^= 0x1D;//Polynomial 0x1D
            }
            else
            {
                t_crc <<= 1;
            }
        }
    }
    return ~t_crc;//XOR 0xFF
}

void ConfigICM42607T(void)
{
	uint8_t spiTxBuffer[16];
	uint8_t spiRxBuffer[16];

	icm42607trm.BANK0.PWR_MGMT0.data8[0]=0x00;
	icm42607trm.BANK0.GYRO_CONFIG0.data8[0]=0x06;
	icm42607trm.BANK0.ACCEL_CONFIG0.data8[0]=0x06;
	icm42607trm.BANK0.TEMP_CONFIG0.data8[0]=0x00;
	icm42607trm.BANK0.GYRO_CONFIG1.data8[0]=0x31;
	icm42607trm.BANK0.ACCEL_CONFIG1.data8[0]=0x41;

	icm42607trm.BANK0.PWR_MGMT0.ACCEL_MODE=3;
	icm42607trm.BANK0.PWR_MGMT0.GYRO_MODE=3;
	icm42607trm.BANK0.GYRO_CONFIG0.GYRO_UI_FS_SEL=3;
	icm42607trm.BANK0.GYRO_CONFIG0.GYRO_ODR=5;
	icm42607trm.BANK0.ACCEL_CONFIG0.ACCEL_UI_FS_SEL=2;
	icm42607trm.BANK0.ACCEL_CONFIG0.ACCEL_ODR=5;
	icm42607trm.BANK0.GYRO_CONFIG1.GYRO_UI_FILT_BW=7;
	icm42607trm.BANK0.ACCEL_CONFIG1.ACCEL_UI_FILT_BW=7;

	spiTxBuffer[0] = ICM42607T_ADDR_B0_PWR_MGMT0;
	spiTxBuffer[1] = icm42607trm.BANK0.PWR_MGMT0.data8[0];
	spiTxBuffer[2] = icm42607trm.BANK0.GYRO_CONFIG0.data8[0];
	spiTxBuffer[3] = icm42607trm.BANK0.ACCEL_CONFIG0.data8[0];
	spiTxBuffer[4] = icm42607trm.BANK0.TEMP_CONFIG0.data8[0];
	spiTxBuffer[5] = icm42607trm.BANK0.GYRO_CONFIG1.data8[0];
	spiTxBuffer[6] = icm42607trm.BANK0.ACCEL_CONFIG1.data8[0];
	SPI2SWNSSTransmitReceive(spiTxBuffer,spiRxBuffer,7);
	//WhoAmICheck();
}

void ReadImuICM42607T(void)
{
	uint8_t spiTxBuffer[16];
	uint8_t spiRxBuffer[16];

	spiTxBuffer[0] = ICM42607T_ADDR_B0_ACCEL_DATA_X1+0x80;
	SPI2SWNSSTransmitReceive(spiTxBuffer,spiRxBuffer,13);
	imuData.AXL=spiRxBuffer[1+ICM42607T_ADDR_B0_ACCEL_DATA_X0-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.AXH=spiRxBuffer[1+ICM42607T_ADDR_B0_ACCEL_DATA_X1-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.AYL=spiRxBuffer[1+ICM42607T_ADDR_B0_ACCEL_DATA_Y0-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.AYH=spiRxBuffer[1+ICM42607T_ADDR_B0_ACCEL_DATA_Y1-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.AZL=spiRxBuffer[1+ICM42607T_ADDR_B0_ACCEL_DATA_Z0-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.AZH=spiRxBuffer[1+ICM42607T_ADDR_B0_ACCEL_DATA_Z1-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.GXL=spiRxBuffer[1+ICM42607T_ADDR_B0_GYRO_DATA_X0-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.GXH=spiRxBuffer[1+ICM42607T_ADDR_B0_GYRO_DATA_X1-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.GYL=spiRxBuffer[1+ICM42607T_ADDR_B0_GYRO_DATA_Y0-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.GYH=spiRxBuffer[1+ICM42607T_ADDR_B0_GYRO_DATA_Y1-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.GZL=spiRxBuffer[1+ICM42607T_ADDR_B0_GYRO_DATA_Z0-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
	imuData.GZH=spiRxBuffer[1+ICM42607T_ADDR_B0_GYRO_DATA_Z1-ICM42607T_ADDR_B0_ACCEL_DATA_X1];
}

void CanInitial(void)
{
	ftt1.Identifier = 0x174;
	ftt1.IdType = FDCAN_STANDARD_ID;
	ftt1.TxFrameType = FDCAN_DATA_FRAME;
	ftt1.DataLength = FDCAN_DLC_BYTES_8;
	ftt1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	ftt1.BitRateSwitch = FDCAN_BRS_OFF;
	ftt1.FDFormat = FDCAN_CLASSIC_CAN;
	ftt1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	ftt1.MessageMarker = 0;

	ftt2.Identifier = 0x178;
	ftt2.IdType = FDCAN_STANDARD_ID;
	ftt2.TxFrameType = FDCAN_DATA_FRAME;
	ftt2.DataLength = FDCAN_DLC_BYTES_8;
	ftt2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	ftt2.BitRateSwitch = FDCAN_BRS_OFF;
	ftt2.FDFormat = FDCAN_CLASSIC_CAN;
	ftt2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	ftt2.MessageMarker = 0;

	ftt3.Identifier = 0x17C;
	ftt3.IdType = FDCAN_STANDARD_ID;
	ftt3.TxFrameType = FDCAN_DATA_FRAME;
	ftt3.DataLength = FDCAN_DLC_BYTES_8;
	ftt3.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	ftt3.BitRateSwitch = FDCAN_BRS_OFF;
	ftt3.FDFormat = FDCAN_CLASSIC_CAN;
	ftt3.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	ftt3.MessageMarker = 0;
}


void CanOutput(void)
{
	static uint8_t counter=0;

	canMessage0x174.can0x174.AY=GetRealAccelerometer(imuData.AY);
	canMessage0x178.can0x178.AX=GetRealAccelerometer(imuData.AX);
	canMessage0x17C.can0x17C.AZ=GetRealAccelerometer(imuData.AZ);

	canMessage0x174.can0x174.YAW_RATE=GetRealGyroscope(imuData.GZ);
	canMessage0x178.can0x178.ROLL_RATE=GetRealGx(imuData.GX);
	canMessage0x17C.can0x17C.PITCH_RATE=GetRealGyroscope(imuData.GY);

	canMessage0x174.can0x174.MSG_CNT=counter;
	canMessage0x178.can0x178.MSG_CNT=counter;
	canMessage0x17C.can0x17C.MSG_CNT=counter;

	counter++;

	canMessage0x174.can0x174.CRC_=Crc8Saej1850(canMessage0x174.data8, 7);
	canMessage0x178.can0x178.CRC_=Crc8Saej1850(canMessage0x178.data8, 7);
	canMessage0x17C.can0x17C.CRC_=Crc8Saej1850(canMessage0x17C.data8, 7);


	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ftt1, canMessage0x174.data8);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ftt2, canMessage0x178.data8);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ftt3, canMessage0x17C.data8);
}
void SPI2SWNSSTransmitReceive(uint8_t *txData,uint8_t *rxData,uint16_t length)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, txData, rxData, length,1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint16_t GetRealAccelerometer(int16_t inputData) {
	double result=(double)inputData;
	result=result * 8 / 65535*9.8*800+32768;
	if(result>65534)result=65534;
	else if(result<0)result=0;
	uint16_t result16=result;
	return result16;
}

uint16_t GetRealGyroscope(int16_t inputData) {
	double result=(double)inputData;
	result=result* 500 / 65535*200+32768;
	if(result>65534)result=65534;
	else if(result<0)result=0;
	uint16_t result16=result;
	return result16;
}

uint16_t GetRealGx(int16_t inputData) {
	double result=(double)inputData;
	result=(-result)* 500 / 65535*200+32768;
	if(result>65534)result=65534;
	else if(result<0)result=0;
	uint16_t result16=result;
	return result16;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim6.Instance)
    {
    	ReadImuICM42607T();
    	CanOutput();
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

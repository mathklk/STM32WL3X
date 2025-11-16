/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @author  Mathis Kölker
  * @brief
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
#include <math.h>
#include <my_stm32wl3x_hal.h>
#include <stdbool.h>
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

SMRSubGConfig MRSUBG_RadioInitStruct;
MRSubG_PcktBasicFields MRSUBG_PacketSettingsStruct;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

typedef enum {
	idle,
	sampling,
	aborting
} State;

typedef union {
	struct {
		int16_t I;
		int16_t Q;
	} iq;
	struct {
		uint8_t b0;
		uint8_t b1;
		uint8_t b2;
		uint8_t b3;
	} b;
	uint32_t w;
} IQ;

#define DB_SAMPLES 2048
__attribute__((aligned(4))) IQ databuffer0[DB_SAMPLES];
__attribute__((aligned(4))) IQ databuffer1[DB_SAMPLES];
IQ* databuffers[] = {databuffer0, databuffer1};
uint16_t getCurrentDatabufferSampleCount(void) {
	return HAL_MRSUBG_GET_DATABUFFER_COUNT() / sizeof(IQ);
}


typedef struct {
	uint32_t ms;
	uint16_t us;
} Timestamp;

static inline uint32_t getMs(void) {
	return HAL_GetTick();
}
static inline uint16_t getUs(void) {
	return __HAL_TIM_GET_COUNTER(&htim2);
}
static inline uint16_t get64ns(void) {
	return __HAL_TIM_GET_COUNTER(&htim16);
}
static inline Timestamp now(void) {
	Timestamp t = {getMs(), getUs()};
	return t;
}

float iqFrequencyHz = 0;


#if 0
static inline void printBuffer(uint8_t n, char label) {
	printf("DB%c = [\r\n  ", label);
	for (int i = 0; i < DB_SAMPLES; ++i) {
		printf("(%d,%d),", databuffers[n][i].IQ.I, databuffers[n][i].IQ.Q);
	}
	printf("\r\n]\r\n");
}

static inline void printBuffers(void) {
	if (HAL_MRSUBG_GET_CURRENT_DATABUFFER() == 1) {
		printBuffer(0, 'A');
		printBuffer(1, 'B');
	} else {
		printBuffer(1, 'A');
		printBuffer(0, 'B');
	}
	printf("iqf=%f\r\n", iqFrequencyHz);
	printf("sync=(%d, %d)\r\n", syncBuffers, syncCount);
	printf("finish=(%d, %d)\r\n", finishBuffers, finishCount);
}
#endif

enum Granularity {
	GRAN4 = 4,
	GRAN8 = 8,
	GRAN16 = 16
};

// [DBA,2048,4=....]
__attribute__((optimize("unroll-loops")))
static inline
void printBufferHex(uint8_t n, enum Granularity g) {
	IQ* bufN = databuffers[n];

	volatile uint8_t b;

	#define OUT putchar
//	#define OUT b=

	if (g == GRAN4) {
		uint32_t w;
		for (int i = 0; i < DB_SAMPLES; ++i) {
			w = bufN[i].w;
			// Oberste 4 bit von I + oberste 4 bit von Q in einem byte
			OUT( (uint8_t)((w >> 24) & 0xF0) | (w >> 12) );
		}
	} else if (g == GRAN8) {
		IQ* c;
		for (int i = 0; i < DB_SAMPLES; ++i) {
			c = bufN + i;
			// Oberste 8 bit von I als 1 byte
			OUT( (uint8_t)c->b.b0 );
			// Oberste 8 bit von Q als 1 byte
			OUT( (uint8_t)c->b.b3 );
		}
	} else if (g == GRAN16) {
		for (int i = 0; i < DB_SAMPLES; ++i) {
//			c = bufN[i];
//			(uint8_t)( w >> 24 )
//			(uint8_t)( w >> 16 )
//			(uint8_t)( (databuffers[n][i].IQ.I & 0xFF) )
//			(uint8_t)( (databuffers[n][i].IQ.Q & 0xFF00) >> 8 )
//			(uint8_t)( (databuffers[n][i].IQ.Q & 0xFF) )
		}
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_MRSUBG_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MRSUBG_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LD1);
  BSP_LED_Init(LD2);
  BSP_LED_Init(LD3);
  HAL_TIM_Base_Start(&htim2);

  COM_InitTypeDef COM_Init;
  COM_Init.BaudRate= 2e6;
  COM_Init.HwFlowCtl = COM_HWCONTROL_NONE;
  COM_Init.WordLength = COM_WORDLENGTH_8B;
  COM_Init.Parity = COM_PARITY_NONE;
  COM_Init.StopBits = COM_STOPBITS_1;
  BSP_COM_Init(COM1, &COM_Init);

  printf("\r\n\n\n\nCompile time %s - %s\r\n", __DATE__, __TIME__);

  /* Set RX Mode to IQ Mode*/
  __HAL_MRSUBG_SET_RX_MODE(RX_IQ_SAMPLING);

  // Enable interrupts for SYNC_VALID flag
  /*
  MRSubG_RFSEQ_IrqStatus_t enabledInterrupts;
  enabledInterrupts.w = 0;
  enabledInterrupts.bits.SYNC_VALID_F = 1;
  __HAL_MRSUBG_SET_RFSEQ_IRQ_ENABLE(enabledInterrupts.w);
  */


  /* Payload length config */
  __HAL_MRSUBG_SET_DATABUFFER_SIZE(DB_SAMPLES * sizeof(IQ));
  __HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t)&databuffer0);
  __HAL_MRSUBG_SET_DATABUFFER1_POINTER((uint32_t)&databuffer1);

  /* Start RX */
  __HAL_MRSUBG_STROBE_CMD(CMD_RX);

  printf("Radio settings:\r\n");
  printf("    Frequency Base = %ld MHz\r\n", MRSUBG_RadioInitStruct.lFrequencyBase / 1000000);
  printf("    Data rate      = %ld ksps\r\n", MRSUBG_RadioInitStruct.lDatarate / 1000);
  printf("    CHF Bandwidth  = %ld kHz (E=%d, M=%d)\r\n", MRSUBG_RadioInitStruct.lBandwidth / 1000, HAL_MRSUBG_GET_CHF_E(), HAL_MRSUBG_GET_CHF_M());
  printf("    Modulation     = %s\r\n", tr_ModSelect(MRSUBG_RadioInitStruct.xModulationSelect));


  printf("IQ sampling settings:\r\n");
//  float const iqSamplingFrequency = MRSUBG_RadioInitStruct.lFrequencyBase / (8 * exp2(LL_MRSubG_GetChFlt_E()));
  float const samplingFrequencyM = 64ul / (8 * exp2(LL_MRSubG_GetChFlt_E()));
  float const iqFrequencyM = samplingFrequencyM/sizeof(IQ);
  iqFrequencyHz = iqFrequencyM * 1000000;
  printf("    Sampling frequency  = %f Msps (%f MIQ/s, 1IQ per %f µs)\r\n", samplingFrequencyM, iqFrequencyM, 1.0f/iqFrequencyM);
  printf("    Data Buffer Size    = %d samples/buffer * 32 bit/sample * 2 buffers = %d bytes | %f µs per buffer\r\n",
		  DB_SAMPLES,
		  (sizeof(databuffer0)+sizeof(databuffer1)),
		  1000000.0f*DB_SAMPLES/iqFrequencyHz
  );
  int const msgLengthBits =
		  MRSUBG_PacketSettingsStruct.PreambleLength +
		  ( (MRSUBG_PacketSettingsStruct.SyncPresent == ENABLE) ? MRSUBG_PacketSettingsStruct.SyncLength : 0) +
		  ( (MRSUBG_PacketSettingsStruct.FixVarLength == FIXED) ? 0 : (8*(1+MRSUBG_PacketSettingsStruct.LengthWidth)) ) +
		  PAYLOAD_LENGTH +
		  MRSUBG_PacketSettingsStruct.PostambleLength +
		  crcBits(MRSUBG_PacketSettingsStruct.CrcMode)
  ;
  printf("    Message length     = %d bits\r\n"
		  "        Preamble  : %d\r\n"
		  "        Sync      : %d\r\n"
		  "        Len       : %d\r\n"
		  "        Payload   : %d\r\n"
		  "        Postamble : %d\r\n"
		  "        CRC       : %d\r\n"
		  , msgLengthBits
		  , MRSUBG_PacketSettingsStruct.PreambleLength
		  , ( (MRSUBG_PacketSettingsStruct.SyncPresent == ENABLE) ? MRSUBG_PacketSettingsStruct.SyncLength : 0)
		  , ( (MRSUBG_PacketSettingsStruct.FixVarLength == FIXED) ? 0 : (8*(1+MRSUBG_PacketSettingsStruct.LengthWidth)) )
		  , PAYLOAD_LENGTH
		  , MRSUBG_PacketSettingsStruct.PostambleLength
		  , crcBits(MRSUBG_PacketSettingsStruct.CrcMode)
  );
  int const msgLengthUs = (1000000*msgLengthBits) / MRSUBG_RadioInitStruct.lDatarate; // Abhängig von Modulation!
  printf("    Estimated message time = %d µs\r\n", msgLengthUs);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  State state = idle;
  uint32_t const blinkPeriodMs = 500;

  Timestamp lastCycle = {0, 0};
  uint8_t n = 0;
  uint16_t dumpDiffs[256];
  uint16_t cycleDiffs[256];

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if (getMs() % blinkPeriodMs < blinkPeriodMs/2) {
		BSP_LED_On(LD1);
	} else {
		BSP_LED_Off(LD1);
	}

    uint32_t const irq = __HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS();

    if (
		( irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER0_USED_F ) ||
		( irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER1_USED_F )
	) {
    	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER0_USED_F );
    	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER1_USED_F );

    	Timestamp const tBeforeDump = now();
    	uint8_t const currentBuffer = HAL_MRSUBG_GET_CURRENT_DATABUFFER();
    	printf("[DB,2048,4=");
    	printBufferHex(currentBuffer, GRAN4);
    	printf("]\r\n");
    	Timestamp const tAfterDump = now();
    	cycleDiffs[n] = (uint16_t)(tAfterDump.us - lastCycle.us);
    	dumpDiffs[n] = (uint16_t)(tAfterDump.us - tBeforeDump.us);
    	++n;
    	lastCycle = tAfterDump;

    	if (n == 0) {
    		float dumpSum = 0;
    		float cycleSum = 0;
    		for (uint16_t i = 0; i < 256; ++i) {
    			dumpSum += dumpDiffs[i];
    			cycleSum += cycleDiffs[i];
    		}
    		float const avgDumpSec  = dumpSum /(256*1000000);
    		float const avgCycleSec = cycleSum/(256*1000000);
    		printf("\r\nCycle time %f us, dump took %f us, %f IQ/s\r\n", avgCycleSec*1000000 , avgDumpSec*1000000, DB_SAMPLES/avgDumpSec);
    	}

    }

	#if 0
    if ( (state == idle) && (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SYNC_VALID_F) ) {
    	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SYNC_VALID_F );

    	state = sampling;
    	syncTimestamp = now();
    	syncCount = getCurrentDatabufferSampleCount();
    	syncBuffers = HAL_MRSUBG_GET_NUMBER_OF_DATABUFFERS();
    	printf("SYNC %ld ms | %d us\r\n", syncTimestamp.ms,syncTimestamp.us);
    }

    if ( (state == sampling) && (getUs() - msgLengthUs*2 > syncTimestamp.us) ) {
    	finishTimestamp = now();
		finishCount = getCurrentDatabufferSampleCount();
		finishBuffers = HAL_MRSUBG_GET_NUMBER_OF_DATABUFFERS();

		uint32_t diffBuffers = finishBuffers - syncBuffers;
		uint32_t diffMs = finishTimestamp.ms - syncTimestamp.ms;
		uint16_t diffUs = finishTimestamp.us - syncTimestamp.us;
		printf("Produced %d buffers in %d ms | %d us\r\n", diffBuffers, diffMs, diffUs);

    	__HAL_MRSUBG_STROBE_CMD(CMD_SABORT);
    	state = aborting;
    }

    if ( (state == aborting) && (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F) ) {
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F );

    	//printBuffersHex(GRAN16);

		state = idle;
		__HAL_MRSUBG_STROBE_CMD(CMD_RX);
	}
	#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource and SYSCLKDivider
  */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_RC64MPLL;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_1) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV4;
  PeriphClkInitStruct.KRMRateMultiplier = 4;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief MRSUBG Initialization Function
  * @param None
  * @retval None
  */
static void MX_MRSUBG_Init(void)
{

  /* USER CODE BEGIN MRSUBG_Init 0 */

  /* USER CODE END MRSUBG_Init 0 */

  /* USER CODE BEGIN MRSUBG_Init 1 */

  /* USER CODE END MRSUBG_Init 1 */

  /** Configures the radio parameters
  */
  MRSUBG_RadioInitStruct.lFrequencyBase = 433000000;
  MRSUBG_RadioInitStruct.xModulationSelect = MOD_OOK;
  MRSUBG_RadioInitStruct.lDatarate = 38400;
  MRSUBG_RadioInitStruct.lFreqDev = 20000;
  MRSUBG_RadioInitStruct.lBandwidth = 50000;
  MRSUBG_RadioInitStruct.dsssExp = 0;
  MRSUBG_RadioInitStruct.outputPower = 14;
  MRSUBG_RadioInitStruct.PADrvMode = PA_DRV_TX_HP;
  HAL_MRSubG_Init(&MRSUBG_RadioInitStruct);

  /** Configures the packet parameters
  */
  MRSUBG_PacketSettingsStruct.PreambleLength = 32;
  MRSUBG_PacketSettingsStruct.PostambleLength = 0;
  MRSUBG_PacketSettingsStruct.SyncLength = 31;
  MRSUBG_PacketSettingsStruct.SyncWord = 0x88888888;
  MRSUBG_PacketSettingsStruct.FixVarLength = FIXED;
  MRSUBG_PacketSettingsStruct.PreambleSequence = PRE_SEQ_0101;
  MRSUBG_PacketSettingsStruct.PostambleSequence = POST_SEQ_0101;
  MRSUBG_PacketSettingsStruct.CrcMode = PKT_CRC_MODE_8BITS;
  MRSUBG_PacketSettingsStruct.Coding = CODING_NONE;
  MRSUBG_PacketSettingsStruct.DataWhitening = DISABLE;
  MRSUBG_PacketSettingsStruct.LengthWidth = BYTE_LEN_1;
  MRSUBG_PacketSettingsStruct.SyncPresent = ENABLE;
  HAL_MRSubG_PacketBasicInit(&MRSUBG_PacketSettingsStruct);
  /* USER CODE BEGIN MRSUBG_Init 2 */

  /* USER CODE END MRSUBG_Init 2 */

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
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF-1;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 0xFFFF-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim16, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_MRSubG_IRQ_Callback(void)
{
  /*
	// read status register and
	irq = __HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS();

	for (int i = 0; i < N_MRSUBG_INTERRUPTS; ++i) {
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(enabledMrsubgInterrupts[i]);
	}

	//__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SYNC_VALID_F);
	if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SYNC_VALID_F) {
		packetReceived = true;
	}
  */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  printf("####ERROR#### (printed in main.c:Error_Handler()\r\n");
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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

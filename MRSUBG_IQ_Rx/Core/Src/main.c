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

/* USER CODE BEGIN PV */

typedef enum {
	idle,
	sampling
} State;

typedef union {
	struct {
		uint16_t I;
		uint16_t Q;
	} IQ;
	uint32_t w;
} IQ;

#define DB_SAMPLES 2048
__attribute__((aligned(4))) IQ databuffer0[DB_SAMPLES];
__attribute__((aligned(4))) IQ databuffer1[DB_SAMPLES];
IQ* databuffers[] = {databuffer0, databuffer1};

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
static inline void sync(Timestamp* ts) {
	ts->ms = getMs();
	ts->us = getUs();
}

static inline void printBuffer(uint8_t n, bool partial) {
	printf("DB%s = [\r\n  ", partial ? "B" : "A");
	IQ const*const databuffer = databuffers[n];
	for (int i = 0; i < DB_SAMPLES; ++i) {
		printf("(%d,%d),", databuffer[i].IQ.I, databuffer[i].IQ.Q);
	}
	printf("\r\n] # Databuffer%d %ld/%d\r\n",
		n,
		(partial ? (HAL_MRSUBG_GET_DATABUFFER_COUNT()/sizeof(IQ)) : DB_SAMPLES),
		DB_SAMPLES
	);
	if (partial) {
		printf("n=%ld\r\n", (HAL_MRSUBG_GET_DATABUFFER_COUNT()/sizeof(IQ)));
	}
}

static inline void printBuffers(void) {
	if (HAL_MRSUBG_GET_CURRENT_DATABUFFER() == 1) {
		printBuffer(0, false);
		printBuffer(1, true);
	} else {
		printBuffer(1, false);
		printBuffer(0, true);
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_MRSUBG_Init(void);
static void MX_TIM2_Init(void);
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
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LD1);
  BSP_LED_Init(LD2);
  BSP_LED_Init(LD3);
  HAL_TIM_Base_Start(&htim2);

  COM_InitTypeDef COM_Init;
  COM_Init.BaudRate= 1000000;
  COM_Init.HwFlowCtl = COM_HWCONTROL_NONE;
  COM_Init.WordLength = COM_WORDLENGTH_8B;
  COM_Init.Parity = COM_PARITY_NONE;
  COM_Init.StopBits = COM_STOPBITS_1;
  BSP_COM_Init(COM1, &COM_Init);

  printf("\r\n\n\n\nHello World!\r\n");
  printf("Compile time %s - %s\r\n", __DATE__, __TIME__);

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
  printf("    CHF Bandwidth  = %ld kHz (CHFLT_E = 0x%X)\r\n", MRSUBG_RadioInitStruct.lBandwidth / 1000, LL_MRSubG_GetChFlt_E());
  printf("    Modulation     = %s\r\n", tr_ModSelect(MRSUBG_RadioInitStruct.xModulationSelect));


  printf("IQ sampling settings:\r\n");
  float const iqSamplingFrequency = MRSUBG_RadioInitStruct.lFrequencyBase / (8 * exp2(LL_MRSubG_GetChFlt_E()));
  printf("    Sampling frequency  = %f Msps\r\n", iqSamplingFrequency/1000000);
  printf("    Data Buffer Size    = %d samples/buffer * 32 bit/sample * 2 buffers = %d bytes total\r\n",
		  DB_SAMPLES,
		  (sizeof(databuffer0)+sizeof(databuffer1))
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

  State state = idle;
  uint32_t const blinkPeriodMs = 500;
  uint32_t const rxLedPeriodMs = 100;
  Timestamp rxTimestamp = {0, 0};

  //MRSubGFSMState lastState = STATE_IDLE;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if (getMs() % blinkPeriodMs < blinkPeriodMs/2) {
		BSP_LED_On(LD1);
	} else {
		BSP_LED_Off(LD1);
	}
	if (getMs() - rxLedPeriodMs < rxTimestamp.ms) {
		BSP_LED_On(LD2);
	} else {
		BSP_LED_Off(LD2);
	}

//	MRSubGFSMState currentState = LL_MRSubG_GetRadioFSMState();
//	if (currentState != lastState) {
//		printf("Current FSM state: %s\r\n", tr(currentState));
//		lastState = currentState;
//	}

    uint32_t const irq = __HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS();

    if (state == idle && irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SYNC_VALID_F) {
    	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SYNC_VALID_F );

    	printf("Valid SYNC received\r\n");
    	//printf("@ buffer %ld count %ld (%ld databuffers used)\r\n",  HAL_MRSUBG_GET_CURRENT_DATABUFFER(), HAL_MRSUBG_GET_DATABUFFER_COUNT(), HAL_MRSUBG_GET_NUMBER_OF_DATABUFFERS() );
    	state = sampling;
    	sync(&rxTimestamp);
    }

    if (state == sampling && getUs() - msgLengthUs < rxTimestamp.us) {
    	__HAL_MRSUBG_STROBE_CMD(CMD_SABORT);
    	printBuffers();
    }

    if (state == sampling && irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F) {
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F );
//		printf("SABORT done\r\n");
//		printf("@ buffer %d count %ld\r\n",  __HAL_MRSUBG_GET_CURRENT_DATABUFFER(), __HAL_MRSUBG_GET_DATABUFFER_COUNT() );
		state = idle;
		__HAL_MRSUBG_STROBE_CMD(CMD_RX);
	}
//    if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER0_USED_F) {
//       	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER0_USED_F );
//       	printf("Databuffer0 is used\r\n");
//    }
//    if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER1_USED_F) {
//       	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER1_USED_F );
//       	printf("Databuffer1 is used\r\n");
//    }



	#if 0

    // Kopie vom MRSUBG_BasicGeneric_Rx Beispiel

    if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F ) {
      BSP_LED_On(LD2);
      /* Clear the IRQ flag */
      __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F);

      /* print the received data */
      printf("RX - Data received: [ ");

      for(uint8_t i=0; i<DB_SIZE; i++)
        printf("%d/%d ", databuffer0[i].IQ.I, databuffer0[i].IQ.Q);
      printf("]\r\n");

      /* Restart RX */
      __HAL_MRSUBG_STROBE_CMD(CMD_RX);
    }
    else if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F) {
      BSP_LED_On(LD3);
      printf("CRC Error\n\r");

      /* Clear the IRQ flag */
      __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F);

      /* Restart RX */
      __HAL_MRSUBG_STROBE_CMD(CMD_RX);
    }
    else if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_TIMEOUT_F) {
      BSP_LED_On(LD3);
      printf("RX Timeout\n\r");

      /* Clear the IRQ flag */
      __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_TIMEOUT_F);

      /* Restart RX */
      __HAL_MRSUBG_STROBE_CMD(CMD_RX);
    }



    /* Periodically read RTC and print to UART without blocking radio handling */
    // if ((HAL_GetTick() - lastRtcTick) >= RTC_PRINT_INTERVAL) {
    //   /* Read RTC time then date (per HAL guidelines) */
    //   if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK) {
    //     if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK) {
    //       /* year in RTC is offset from 2000 */
    //       int year = 2000 + sDate.Year;
    //       snprintf(rtcBuf, sizeof(rtcBuf), "RTC: %02d:%02d:%02d %02d/%02d/%04d\r\n",
    //                sTime.Hours, sTime.Minutes, sTime.Seconds,
    //                sDate.Date, sDate.Month, year);
    //       printf("%s", rtcBuf);
    //     }
    //   }
    //   lastRtcTick = HAL_GetTick();
    // }

    /* pause between receptions */
    HAL_Delay(250);
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
  MRSUBG_RadioInitStruct.xModulationSelect = MOD_2FSK;
  MRSUBG_RadioInitStruct.lDatarate = 38400;
  MRSUBG_RadioInitStruct.lFreqDev = 20000;
  MRSUBG_RadioInitStruct.lBandwidth = 100000;
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
  MRSUBG_PacketSettingsStruct.DataWhitening = ENABLE;
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

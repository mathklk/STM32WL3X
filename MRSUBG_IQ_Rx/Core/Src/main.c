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


/* Timer */

typedef struct {
    uint32_t ms;
    uint16_t us;
} Timestamp;

// In Millisekunden
static inline uint32_t getMs(void) {
    return HAL_GetTick();
}
// In Mikrosekunden
static inline uint16_t getUs(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}
// In 1/64 µs
static inline uint16_t get64ns(void) {
    return __HAL_TIM_GET_COUNTER(&htim16);
}
static inline Timestamp now(void) {
    Timestamp t = {getMs(), getUs()};
    return t;
}


// MRSUBG
typedef enum {
    idle,
    starting,
    sampling,
    stopping
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

#define NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER 3500
__attribute__((aligned(4))) IQ databuffer0[NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER];
__attribute__((aligned(4))) IQ databuffer1[NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER];
IQ* databuffers[] = {databuffer0, databuffer1};

uint16_t getNumberOfIQSamplesInCurrentDatabuffer(void) {
    return HAL_MRSUBG_GET_DATABUFFER_COUNT() / sizeof(IQ);
}


enum Granularity {
    GRAN4 = 4,
    GRAN8 = 8,
    GRAN16 = 16
};

// [DBA,2048,4=....]
__attribute__((optimize("unroll-loops")))
static inline
void printBufferHex(uint8_t n, enum Granularity g) {
    #define OUT putchar

    if (g == GRAN4) {
        IQ* bufN = databuffers[n];
        uint32_t w;
        for (int i = 0; i < NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER; ++i) {
            w = bufN[i].w;
            // Oberste 4 bit von I + oberste 4 bit von Q in einem byte
            OUT( (uint8_t)((w >> 24) & 0xF0) | (w >> 12) );
        }
    } else if (g == GRAN8) {
        IQ* bufN = databuffers[n];
        IQ* c;
        for (int i = 0; i < NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER; ++i) {
            c = bufN + i;
            // Oberste 8 bit von I als 1 byte
            OUT( (uint8_t)c->b.b0 );
            // Oberste 8 bit von Q als 1 byte
            OUT( (uint8_t)c->b.b3 );
        }
    } else if (g == GRAN16) {
        // Gesamten Buffer Byteweise
        uint8_t* bufAsBytes = (uint8_t*)databuffers[n];
        for (size_t i = 0; i < NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER * sizeof(IQ); ++i) {
            OUT( bufAsBytes[i] );
        }
    }
}

#define NOP asm volatile ("nop");
volatile uint8_t *const pb_b0 =  (uint8_t*)&GPIOB->ODR;
volatile uint8_t *const pb_b1 = ((uint8_t*)&GPIOB->ODR )+ 1;

__attribute__((optimize("unroll-loops")))
static inline
void parallelOut16(uint8_t n) {
    uint8_t *buf = (uint8_t*)databuffers[n];
    uint16_t count = NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER * sizeof(IQ);

//	for (uint16_t i = 0; i < count; ++i) {
//		*pb_b0 = bufAsU8[i];
//		*pb_b1 = 0x01;
//		*pb_b1 = 0x00;
//	}
    for (uint16_t i = 0; i < count; i += 4) {
        *pb_b0 = buf[i+0]; *pb_b1 = 1; *pb_b1 = 0;
        *pb_b0 = buf[i+1]; *pb_b1 = 1; *pb_b1 = 0;
        *pb_b0 = buf[i+2]; *pb_b1 = 1; *pb_b1 = 0;
        *pb_b0 = buf[i+3]; *pb_b1 = 1; *pb_b1 = 0;
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

  // Overwrite the Channel Filter
//  HAL_MRSUBG_SET_CHFLT_E(0);
  HAL_MRSUBG_SET_CHFLT_M(15);

  /* Set RX Mode to IQ Mode*/
  __HAL_MRSUBG_SET_RX_MODE(RX_IQ_SAMPLING);

  /* Payload length config */
  __HAL_MRSUBG_SET_DATABUFFER_SIZE(NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER * sizeof(IQ));
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
  float const iqFrequencyHz = iqFrequencyM * 1000000;
  float const speedOfLight = 300000000;
  printf("    Sampling frequency  = %f Msps (%f MIQ/s, 1IQ per %f µs)\r\n", samplingFrequencyM, iqFrequencyM, 1.0f/iqFrequencyM);
  printf("    Data Buffer Size    = %d IQ samples per buffer | %f ms stored in total | %f meter\r\n",
          NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER,
          2*1000.0f*NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER/iqFrequencyHz,
          2*speedOfLight/iqFrequencyHz
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

  volatile BufferInfo atTwoBuffersDetected;
  volatile BufferInfo atFullyAborted;

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

    if (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_COMMAND_REJECTED_F) {
        __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_COMMAND_REJECTED_F);
        printf("COMMAND_REJECTED\r\n");
        continue;
    }

    if ((state == idle) && (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SYNC_VALID_F) ) {
      __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SYNC_VALID_F );
      __HAL_MRSUBG_STROBE_CMD(CMD_SABORT);
      BSP_LED_On(LD2);
      state = starting;
      continue;
    }
    if ( (state == starting) && (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F)) {
        __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F);
        __HAL_MRSUBG_STROBE_CMD(CMD_RX);
        state = sampling;
        continue;
    }
    if ( (state == sampling) && HAL_MRSUBG_GET_BUFFER_INFO().numberOfBuffersUsed >= 2) {
        atTwoBuffersDetected = HAL_MRSUBG_GET_BUFFER_INFO();
        __HAL_MRSUBG_STROBE_CMD(CMD_SABORT);
        state = stopping;
        continue;
    }
    if ( (state == stopping) && (irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F)) {
        __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F);
        atFullyAborted = HAL_MRSUBG_GET_BUFFER_INFO();

        printf("@TwoBuffersDetected: #%d  Nr. %ld  +%ld\r\n", atTwoBuffersDetected.buffer, atTwoBuffersDetected.numberOfBuffersUsed, atTwoBuffersDetected.iqSampleCount);
        printf("@FullyAborted:       #%d  Nr. %ld  +%ld\r\n", atFullyAborted.buffer, atFullyAborted.numberOfBuffersUsed, atFullyAborted.iqSampleCount);

        enum Granularity const gran = GRAN16;
        bool const currentBuffer = !HAL_MRSUBG_GET_CURRENT_DATABUFFER(); // Negieren, weil zum Zeitpunkt des stoppens schon der nächste Buffer angefangen ist
        printf("[DB,%d,%d=", NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER, gran);
        printBufferHex( !currentBuffer, gran);
        printf("]\r\n");
        printf("[DB,%d,%d=", NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER, gran);
        printBufferHex(currentBuffer, gran);
        printf("]\r\n");
        printf("TRANSMISSION_COMPLETE\r\n");

        state = idle;
        __HAL_MRSUBG_STROBE_CMD(CMD_RX);
        BSP_LED_Off(LD2);
        continue;
    }


#if 0
    if ( state == sample && (
        ( irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER0_USED_F ) ||
        ( irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER1_USED_F )
    )) {
        __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER0_USED_F );
        __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG( MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_DATABUFFER1_USED_F );
        ++numberOfDatabuffersReceived;

        if (numberOfDatabuffersReceived == 2) {
//    		__HAL_MRSUBG_SET_DATABUFFER_SIZE(0);
            __HAL_MRSUBG_STROBE_CMD(CMD_SABORT);
//    		state = output;
        }
    }

    if (false && irq & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F) {
        __HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SABORT_DONE_F);

        enum Granularity const gran = GRAN16;

        // Negieren, weil der "current buffer" schon inkrementiert ist, die aktuellsten Daten aber im letzten sind. Die ersten ~20 µs könnten schon überschrieben sein
        bool const currentBuffer = !HAL_MRSUBG_GET_CURRENT_DATABUFFER();
        printf("[DB,%d,%d=", NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER, gran);
        printBufferHex( !currentBuffer, gran);
        printf("]\r\n");
        printf("[DB,%d,%d=", NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER, gran);
        printBufferHex(currentBuffer, gran);
        printf("]\r\n");

        state = idle;
        __HAL_MRSUBG_SET_DATABUFFER_SIZE(NUMBER_OF_IQ_SAMPLES_IN_DATABUFFER * sizeof(IQ));
        __HAL_MRSUBG_STROBE_CMD(CMD_RX);
    }

    //-----


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
  MRSUBG_RadioInitStruct.xModulationSelect = MOD_ASK;
  MRSUBG_RadioInitStruct.lDatarate = 38400;
  MRSUBG_RadioInitStruct.lFreqDev = 20000;
  MRSUBG_RadioInitStruct.lBandwidth = 888000;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6
                           PB7 PB8 PB0 PB1
                           PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_3|PWR_GPIO_BIT_4|PWR_GPIO_BIT_5|PWR_GPIO_BIT_6
                          |PWR_GPIO_BIT_7|PWR_GPIO_BIT_8|PWR_GPIO_BIT_0|PWR_GPIO_BIT_1
                          |PWR_GPIO_BIT_2);

  /**/
  HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_3|PWR_GPIO_BIT_4|PWR_GPIO_BIT_5|PWR_GPIO_BIT_6
                          |PWR_GPIO_BIT_7|PWR_GPIO_BIT_8|PWR_GPIO_BIT_0|PWR_GPIO_BIT_1
                          |PWR_GPIO_BIT_2);

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

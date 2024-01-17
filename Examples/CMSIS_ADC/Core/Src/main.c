/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Program to test Bare Metal Register Programming ADCs w/ DMA
  * @author         : Alex Bashara
  * @date           : 12/16/2023
  * @note           : Code Built for Nucleo Test Board
  ******************************************************************************/

/* Includes */
#include "main.h"
#include "cmsis_os2.h"
#include "stm32h743xx.h"

uint32_t ADC_Values[16]; // Buffer to store ADC Values

/* Create RTOS Threads */
/**
 * @brief Basic Test Thread for RTOS
 * @retval None
 */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Thread for ADC Usage
 * @retval None
 */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
  .name = "adcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* Private Function Prototypes */
void StartDefaultTask(void *argument);
void SystemClock_Config(void);
void StartADCTask(void *argument);
void ADC_Init(void);

/**
 * @brief Initialization of the ADC with DMA
 * @note Page 930 in RM0433 Covers enable procedure for ADC
 * @retval None
 */
void ADC_Init() {
  RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN; // Enable ADC Clock
  // Enable GPIO Clocks
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN
              | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN 
              | RCC_AHB4ENR_GPIOFEN | RCC_AHB4ENR_GPIOGEN | RCC_AHB4ENR_GPIOHEN 
              | RCC_AHB4ENR_GPIOIEN | RCC_AHB4ENR_GPIOJEN | RCC_AHB4ENR_GPIOKEN;

  // ADC Power Up Sequence p923 RM0433
  ADC1->CR &= ~ADC_CR_ADEN; // Make sure ADC is disabled
  ADC1->CR &= ~ADC_CR_DEEPPWD; // Make sure ADC is not in Deep Power Down Mode
  ADC1->CR |= ADC_CR_ADVREGEN; // Enable ADC Voltage Regulator
  while (!(ADC1->ISR & ADC_ISR_LDORDY)); // Wait for ADC to be ready

  // ADC Calibration
  ADC1->CR &= ~ADC_CR_ADCALDIF; // Single Ended Calibration
  ADC1->CR |= ADC_CR_ADCALLIN; // Linear Calibration
  ADC1->CR |= ADC_CR_ADCAL; // Start Calibration
  while (ADC1->CR & ADC_CR_ADCAL); // Wait for Calibration to Complete
  // Can read calibration values from ADC1->CALFACT

  // ADC Enable
  ADC1->ISR |= ADC_ISR_ADRDY; // Clear ADRDY Flag by writing '1'
  ADC1->CR |= ADC_CR_ADEN; // Enable ADC
  while(!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait for ADC to be ready
  ADC1->ISR |= ADC_ISR_ADRDY; // Clear ADRDY Flag by writing '1'

  /* ADC Configuration */
  // Set PCSEL Values to enable Channels
  ADC1->PCSEL |= ADC_PCSEL_PCSEL_0 | ADC_PCSEL_PCSEL_1 | ADC_PCSEL_PCSEL_2 
              | ADC_PCSEL_PCSEL_3 | ADC_PCSEL_PCSEL_4 | ADC_PCSEL_PCSEL_5 
              | ADC_PCSEL_PCSEL_6 | ADC_PCSEL_PCSEL_7 | ADC_PCSEL_PCSEL_8 
              | ADC_PCSEL_PCSEL_9 | ADC_PCSEL_PCSEL_10 | ADC_PCSEL_PCSEL_11 
              | ADC_PCSEL_PCSEL_12 | ADC_PCSEL_PCSEL_13 | ADC_PCSEL_PCSEL_14 
              | ADC_PCSEL_PCSEL_15;
  
  //Configure ADC Channels to Single Ended Mode
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_0;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_1;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_2;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_3;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_4;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_5;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_6;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_7;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_8;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_9;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_10;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_11;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_12;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_13;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_14;
  ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_15;

  // Set ADC Conversion Sequence and Size
  ADC1->SQR1 = (15UL << ADC_SQR1_L_Pos); // Number of conversions to 16
  ADC1->SQR1 = (0UL << ADC_SQR1_SQ1_Pos);
  ADC1->SQR1 = (1UL << ADC_SQR1_SQ2_Pos);
  ADC1->SQR1 = (2UL << ADC_SQR1_SQ3_Pos);
  ADC1->SQR1 = (3UL << ADC_SQR1_SQ4_Pos);
  ADC1->SQR2 = (4UL << ADC_SQR2_SQ5_Pos);
  ADC1->SQR2 = (5UL << ADC_SQR2_SQ6_Pos);
  ADC1->SQR2 = (6UL << ADC_SQR2_SQ7_Pos);
  ADC1->SQR2 = (7UL << ADC_SQR2_SQ8_Pos);
  ADC1->SQR2 = (8UL << ADC_SQR2_SQ9_Pos);
  ADC1->SQR3 = (9UL << ADC_SQR3_SQ10_Pos);
  ADC1->SQR3 = (10UL << ADC_SQR3_SQ11_Pos);
  ADC1->SQR3 = (11UL << ADC_SQR3_SQ12_Pos);
  ADC1->SQR3 = (12UL << ADC_SQR3_SQ13_Pos);
  ADC1->SQR3 = (13UL << ADC_SQR3_SQ14_Pos);
  ADC1->SQR4 = (14UL << ADC_SQR4_SQ15_Pos);
  ADC1->SQR4 = (15UL << ADC_SQR4_SQ16_Pos);


// Set sample times p933 RM0433
// TODO: Tune fast and slow channel times to optimize speed
  ADC1->SMPR1 = (4UL << ADC_SMPR1_SMP0_Pos) | (4UL << ADC_SMPR1_SMP1_Pos) | (4UL << ADC_SMPR1_SMP2_Pos) 
              | (4UL << ADC_SMPR1_SMP3_Pos) | (4UL << ADC_SMPR1_SMP4_Pos) | (4UL << ADC_SMPR1_SMP5_Pos) 
              | (4UL << ADC_SMPR1_SMP6_Pos) | (4UL << ADC_SMPR1_SMP7_Pos) | (4UL << ADC_SMPR1_SMP8_Pos) 
              | (4UL << ADC_SMPR1_SMP9_Pos);
  ADC1->SMPR2 = (4UL << ADC_SMPR2_SMP10_Pos) | (4UL << ADC_SMPR2_SMP11_Pos) | (4UL << ADC_SMPR2_SMP12_Pos) 
              | (4UL << ADC_SMPR2_SMP13_Pos) | (4UL << ADC_SMPR2_SMP14_Pos) | (4UL << ADC_SMPR2_SMP15_Pos) 
              | (4UL << ADC_SMPR2_SMP16_Pos);

  // Set ADC Continuous Conversion Mode
  ADC1->CR |= ADC_CR_ADSTART;
  ADC1->CFGR |= ADC_CFGR_CONT;

  ADC1->CFGR = (3UL << ADC_CFGR_DMNGT_Pos); // DMA Circular Mode Set to 11

  // DMA Configuration p653 RM0433
  DMA1_Stream0->CR &= ~DMA_SxCR_EN; // Disable DMA Stream 1
  while(DMA1_Stream0->CR |= DMA_SxCR_EN); // Wait for stream to stop
  DMA1_Stream0->PAR |= (uint32_t) &ADC1->DR; // Set Peripheral Address to ADC1 DR
  DMA1_Stream0->M0AR = (uint32_t)(&ADC_Values); // Set Memory Address to ADC_Values Buffer
  DMA1_Stream0->NDTR = (16UL << DMA_SxNDT_Pos); // Set Number of Data Transfers to 16
  
  // DMA Mux 1 Config p700 RM0433
  DMAMUX1_Channel0->CCR = (DMA_REQUEST_ADC1 << DMAMUX_CxCR_DMAREQ_ID_Pos); // Set DMA Request ID to 9 (ADC1)
  DMAMUX1_Channel0->CCR = (2UL << DMAMUX_CxCR_NBREQ_Pos); // Set Number of DMA Requests to 2 - 1 = 1

  DMA1_Stream0->CR &= ~DMA_SxCR_PFCTRL; // Disable Peripheral Flow Control
  DMA1_Stream0->CR = (3UL << DMA_SxCR_PL_Pos); // Set Priority to Very High
  // Fifo config goes here if needed
  DMA1_Stream0->CR |= DMA_SxCR_CIRC; // Enable Circular Mode
  DMA1_Stream0->CR = (2UL << DMA_SxCR_MSIZE_Pos); // Set Memory Data Size to 32 bits (uint32_t ADC_Val Buffer)
  DMA1_Stream0->CR = (1UL << DMA_SxCR_PSIZE_Pos); // Set Peripheral Data Size to 16 bits (16 bit ADC resolution)
  DMA1_Stream0->CR = (0UL << DMA_SxCR_DIR_Pos); // Set Direction to Peripheral to Memory
  DMA1_Stream0->CR &= ~DMA_SxCR_PFCTRL; // DMA handles Flow Control
  
  DMA1_Stream0->CR |= DMA_SxCR_EN; // Enable DMA Stream 1

  ADC1->CR |= ADC_CR_ADSTART; // Start ADC Conversion
}

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartDefaultTask(void *argument)
{
  /* Infinite loop */
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Function to read from ADC buffer
 * @note Read from ADC DMA Buffer and update Telem data struct
 * @param argument: Not used
 * @retval None
 */
void StartADCTask(void *argument)
{
  /* Infinite loop */
  for(;;) {
    osDelay(1);
  }
}

/**
  * @brief  Function to Initialize the MCU
  * @note Basiclly calls Peripherial Initialization Functions
  * @retval int
  */
int main(void)
{
  // TODO: Need to configure clocks
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  // HAL Sysclock config
  SystemClock_Config();

  // Init RTOS
  osKernelInitialize();

  ADC_Init();

  /* BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  /* BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* BEGIN RTOS_TIMERS */

  /* BEGIN RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  adcTaskHandle = osThreadNew(StartADCTask, NULL, &adcTask_attributes);

  /* BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* BEGIN RTOS_EVENTS */
  /* add events, ... */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  Error_Handler();
  /* Infinite loop */
  while (1) {
    // Put Thread code in Thread Functions
  }
}

/**
  * @brief HAL System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable*/
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  // TODO: Remove once Tested************
  // while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {} (Original HAL Code)
  while(!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY_Msk)) {} // Check for VOSRDY flag

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* Error_Handler_Debug */
  __disable_irq();
  while (1) {
  }
}

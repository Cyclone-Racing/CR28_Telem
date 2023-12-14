/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Program to test Bare Metal Register Programming ADCs w/ DMA
  * @author         : Alex Bashara
  * @author         : Cyclone Racing
  * @note           : Code Built for Nucleo Test Board
  ******************************************************************************/

/* Includes */
#include "main.h"
#include "cmsis_os2.h"
#include "stm32h743xx.h"

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
void StartADCTask(void *argument);
void ADC_Init(void);

/**
 * @brief Initialization of the ADC
 * @note Page 930 on RM0433 Covers enable procedure
 * 
 */
void ADC_Init() {
  RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN; // Enable ADC Clock
  // Enable GPIO Clocks
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN
              | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN 
              | RCC_AHB4ENR_GPIOFEN | RCC_AHB4ENR_GPIOGEN | RCC_AHB4ENR_GPIOHEN 
              | RCC_AHB4ENR_GPIOIEN | RCC_AHB4ENR_GPIOJEN | RCC_AHB4ENR_GPIOKEN;
  // ADC1->ADC_CCR |= ADC_CCR_PRESC_Msk | 0b0010; // Set ADC Prescaler to 4

  ADC1->CR &= ~ADC_CR_ADEN; // Make sure ADC is disabled
  ADC1->CR &= ~ADC_CR_DEEPPWD; // Make sure ADC is not in Deep Power Down Mode
  ADC1->CR |= ADC_CR_ADVREGEN; // Enable ADC Voltage Regulator
  ADC1->CR |= ADC_CR_ADSTART; // Continous Conversion Mode
  while (!(ADC1->ISR & ADC_ISR_LDORDY)); // Wait for ADC to be ready
  
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
 * @brief Function to read from ADC
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
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  // HAL Sysclock config TODO: Test if this is needed
  SystemClock_Config();

  // Init RTOS
  osKernelInitialize();

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

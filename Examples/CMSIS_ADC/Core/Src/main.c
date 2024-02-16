/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Program to test Bare Metal Register Programming ADCs w/ DMA
  * @author         : Alex Bashara
  * @date           : 12/16/2023
  * @note           : Code Built for Nucleo H7 Test Board
  ******************************************************************************/

/* Includes */
#include "main.h"
#include "cmsis_os2.h"
#include "stm32h743xx.h"
#include "gpio.h"
#include "adc.h"

uint32_t ADC_Values[16]; // Buffer to store ADC Values

/**
 * @brief Thread to blink LED
 * @note Low Priority Thread used for system status
 */
osThreadId_t BlinkTaskHandle;
const osThreadAttr_t BlinkTask_attributes = {
  .name = "BlinkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/**
 * @brief Thread for ADC Usage
 * @note High Priority Thread used for ADC Data Collection
 */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
  .name = "ADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

// Function Defines
void StartADCTask(void *argument);
void StartBlinkTask(void *argument);

/**
  * @brief  Function to Initialize the MCU
  * @note Basiclly calls Peripherial Initialization Functions
  * @retval int
  */
int main(void) {

  GPIOB_Init(); // Initialize GPIO
  ADC_Init(); // Initialize ADC
  DMA_Init(); // Initialize DMA
  osKernelInitialize(); // Initialize RTOS Kernel

  /* Create the thread(s) */
  BlinkTaskHandle = osThreadNew(StartBlinkTask, NULL, &BlinkTask_attributes);
  ADCTaskHandle = osThreadNew(StartADCTask, NULL, &ADCTask_attributes);

  osKernelStart(); // Start Kernel

  while (1) {
    Error_Handler();
  }
}

/**
 * @brief Function to read from ADC buffer
 * @note Read from ADC DMA Buffer and update Telem data struct
 * @param argument: Not used
 * @retval None
 */
void StartADCTask(void *argument) {
  /* Infinite loop */
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Function to blink LED
 * @note Blink LED to indicate system is running Low Priority
 * @param argument: Not used
 * @retval None
 */
void StartBlinkTask(void *argument) {
  for(;;) {
    Toggle_Pin(GPIOB, 0); // Green LD1
    osDelay(1000);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @note TODO: Make a more robust error handler
  */
void Error_Handler(void) {
  /* Error_Handler_Debug */
  __disable_irq();
  while (1) {
  }
}

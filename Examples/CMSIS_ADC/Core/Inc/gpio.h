#ifndef GPIO_H
#define GPIO_H

#include "stm32h743xx.h"

/**
 * @brief Initialize the LED GPIO Pins
 * 
 */
void static inline GPIOB_Init() {
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN; // Enable GPIOB clock

  // LD1 (green): GPIOB pin 0
  // LD2 (blue): GPIOB pin 7
  // LD3 (red): GPIOB pin 14

  // Clear output mode bits
  GPIOB->MODER &= ~(0x3 << GPIO_MODER_MODE0_Pos);
  GPIOB->MODER &= ~(0x3 << GPIO_MODER_MODE7_Pos);
  GPIOB->MODER &= ~(0x3 << GPIO_MODER_MODE14_Pos);

  // set to general purpose output mode
  GPIOB->MODER |= (0x1 << GPIO_MODER_MODE0_Pos);
  GPIOB->MODER |= (0x1 << GPIO_MODER_MODE7_Pos);
  GPIOB->MODER |= (0x1 << GPIO_MODER_MODE14_Pos);

  // set to push-pull output mode
  GPIOB->OTYPER &= GPIO_OTYPER_OT0_Msk;
  GPIOB->OTYPER &= GPIO_OTYPER_OT7_Msk;
  GPIOB->OTYPER &= GPIO_OTYPER_OT14_Msk;
}

/**
 * @brief Toggle the GPIO Pin State
 * 
 * @param Port GPIO Port of the pin to toggle
 * @param Pin Number of pin to toggle
 */
void static inline Toggle_Pin(GPIO_TypeDef* Port, int Pin) {
    Port->ODR ^= (1 << Pin);
}

/**
 * @brief Get status bit of GPIO Pin
 * 
 * @param Port GPIO Port of the pin to read
 * @param Pin Pin number to read
 * @return int the bit value of the pin
 */
int static inline Read_Pin(GPIO_TypeDef* Port, int Pin) {
  return Port->IDR &= (1 << Pin);
}


#endif // GPIO_H

#ifndef ADC_H
#define ADC_H

#include "stm32h743xx.h"

/**
 * @brief Initialization of the ADC for use with DMA
 * @note Page 930 in RM0433 Covers enable procedure for ADC
 * @note Uses all 16 Channels of the ADC
 * @retval None
 */
void static inline ADC_Init() {
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
 * @brief Configures the DMA for use with the ADC
 * 
 */
void static inline DMA_ADC_Init() {

}

#endif // ADC_H

/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines for MKS XDrive Mini
  ******************************************************************************
  */
#include "main.h"
#include "stm32f4xx_it.h"
#include "usbd.h"

extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim7;

/*
 * Current control loop of MotorXDrive (ADC2/ADC3 injected end of conversion).
 * Weak default only clears the flags so the target also links without the driver.
 */
__weak void XDrive_Tim1Update(void){}

__weak void XDrive_AdcIsr(void){
	ADC2->SR &= ~ADC_SR_JEOC;
	ADC3->SR &= ~ADC_SR_JEOC;
}

// Bridge off as fast as possible: main output disable + gate driver disable
static inline void bridgeOff(void){
	TIM1->BDTR &= ~TIM_BDTR_MOE;
	DRV_EN_GATE_GPIO_Port->BSRR = (uint32_t)DRV_EN_GATE_Pin << 16U;
}

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  bridgeOff();
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  bridgeOff();
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  bridgeOff();
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  bridgeOff();
  while (1)
  {
  }
}

void DebugMon_Handler(void)
{
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/* Encoder index (PC9) */
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ENCODER_Z_Pin);
}

/*
 * ADC1/2/3 share one interrupt:
 * - ADC2 JEOC: current samples ready -> FOC loop
 * - ADC1 OVR (enabled by HAL_ADC_Start_DMA): DMA stopped. Clear it and let the
 *   driver thread restart the DMA (adc1Overrun), otherwise the flag would retrigger forever.
 */
volatile uint8_t adc1Overrun = 0;
void ADC_IRQHandler(void)
{
  if(ADC2->SR & ADC_SR_JEOC){
    XDrive_AdcIsr();
  }
  if(ADC1->SR & ADC_SR_OVR){
    ADC1->SR = ~ADC_SR_OVR;
    ADC1->CR1 &= ~ADC_CR1_OVRIE;
    adc1Overrun = 1;
  }
}

/* TIM1 update: only used by the MotorXDrive software ADC trigger fallback */
void TIM1_UP_TIM10_IRQHandler(void)
{
  if(TIM1->SR & TIM_SR_UIF){
    TIM1->SR = ~TIM_SR_UIF;
    XDrive_Tim1Update();
  }
}

void TIM8_UP_TIM13_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim13);
}

void TIM7_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim7);
}

void DMA2_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void OTG_FS_IRQHandler(void)
{
	tud_int_handler(0);
}

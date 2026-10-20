/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        MSP init for MKS XDrive Mini (clocks, alternate functions, DMA)
  ******************************************************************************
  */
#include "main.h"

extern DMA_HandleTypeDef hdma_adc1;

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // PA6 -> ADC12_IN6 VBUS_S
    GPIO_InitStruct.Pin = VBUS_S_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VBUS_S_GPIO_Port, &GPIO_InitStruct);

    hdma_adc1.Instance = DMA2_Stream4;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);
  }
  else if(hadc->Instance==ADC2 || hadc->Instance==ADC3)
  {
    if(hadc->Instance==ADC2){
      __HAL_RCC_ADC2_CLK_ENABLE();
    }else{
      __HAL_RCC_ADC3_CLK_ENABLE();
    }
    __HAL_RCC_GPIOC_CLK_ENABLE();
    // PC0 -> ADC123_IN10 (M0_SO1), PC1 -> ADC123_IN11 (M0_SO2)
    GPIO_InitStruct.Pin = M0_SO1_Pin|M0_SO2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI3)
  {
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    // PC10 SCK, PC11 MISO, PC12 MOSI
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if(htim_base->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
  else if(htim_base->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
  else if(htim_base->Instance==TIM13)
  {
    __HAL_RCC_TIM13_CLK_ENABLE();
  }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // PB4 (NJTRST after reset) -> TIM3_CH1, PB5 -> TIM3_CH2. Board has 3.3k pull-ups.
    GPIO_InitStruct.Pin = ENCODER_A_Pin|ENCODER_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // PA8/PA9/PA10 -> TIM1_CH1/2/3 (high side), PB13/14/15 -> TIM1_CH1N/2N/3N (low side)
    GPIO_InitStruct.Pin = M0_AH_Pin|M0_BH_Pin|M0_CH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M0_AL_Pin|M0_BL_Pin|M0_CL_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(htim->Instance==TIM2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // PB10 -> TIM2_CH3 (AUX_L), PB11 -> TIM2_CH4 (AUX_H)
    GPIO_InitStruct.Pin = AUX_L_Pin|AUX_H_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hpcd->Instance==USB_OTG_FS)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // PA11 DM, PA12 DP. PA9 is NOT configured (it is the phase B PWM output)
    GPIO_InitStruct.Pin = OTG_FS_DM_Pin|OTG_FS_DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  }
}

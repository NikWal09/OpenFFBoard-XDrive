/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Pin map for MKS XDrive Mini V1.0 (STM32F405RGT6)
  *
  * Target written by hand (no CubeMX project). Pin assignment taken from the
  * board schematic, see Targets/F405RG_XDRIVE/README.md.
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);
void MX_USB_OTG_FS_PCD_Init(void);
void RebootDFU();
void ExitDFU();

/* ---- Motor M0: DRV8301 + 3 phase bridge (TIM1 6-PWM) -------------------- */
#define M0_AH_Pin GPIO_PIN_8          // TIM1_CH1
#define M0_AH_GPIO_Port GPIOA
#define M0_BH_Pin GPIO_PIN_9          // TIM1_CH2 (no USB VBUS sense on this board!)
#define M0_BH_GPIO_Port GPIOA
#define M0_CH_Pin GPIO_PIN_10         // TIM1_CH3
#define M0_CH_GPIO_Port GPIOA
#define M0_AL_Pin GPIO_PIN_13         // TIM1_CH1N
#define M0_AL_GPIO_Port GPIOB
#define M0_BL_Pin GPIO_PIN_14         // TIM1_CH2N
#define M0_BL_GPIO_Port GPIOB
#define M0_CL_Pin GPIO_PIN_15         // TIM1_CH3N
#define M0_CL_GPIO_Port GPIOB

#define DRV_EN_GATE_Pin GPIO_PIN_12   // high = DRV8301 enabled
#define DRV_EN_GATE_GPIO_Port GPIOB
#define DRV_NFAULT_Pin GPIO_PIN_2     // low = fault
#define DRV_NFAULT_GPIO_Port GPIOD
#define DRV_NCS_Pin GPIO_PIN_13       // SPI3 chip select of DRV8301
#define DRV_NCS_GPIO_Port GPIOC
// SPI3: SCK PC10, MISO PC11, MOSI PC12 (AF6)

/* ---- Brake resistor half bridge (LM5109B), TIM2 CH3/CH4 ------------------ */
#define AUX_L_Pin GPIO_PIN_10         // TIM2_CH3
#define AUX_L_GPIO_Port GPIOB
#define AUX_H_Pin GPIO_PIN_11         // TIM2_CH4
#define AUX_H_GPIO_Port GPIOB

/* ---- Analog --------------------------------------------------------------- */
#define VBUS_S_Pin GPIO_PIN_6         // ADC12_IN6, divider 18k/1k (x19)
#define VBUS_S_GPIO_Port GPIOA
#define M0_SO1_Pin GPIO_PIN_0         // ADC123_IN10, phase C current
#define M0_SO1_GPIO_Port GPIOC
#define M0_SO2_Pin GPIO_PIN_1         // ADC123_IN11, phase B current
#define M0_SO2_GPIO_Port GPIOC

/* ---- Encoder J5 (MT6701 ABZ), TIM3 encoder mode ----------------------------- */
#define ENCODER_A_Pin GPIO_PIN_4      // TIM3_CH1 (AF2), NJTRST after reset
#define ENCODER_A_GPIO_Port GPIOB
#define ENCODER_B_Pin GPIO_PIN_5      // TIM3_CH2 (AF2)
#define ENCODER_B_GPIO_Port GPIOB
#define ENCODER_Z_Pin GPIO_PIN_9      // index, EXTI9
#define ENCODER_Z_GPIO_Port GPIOC
#define ENCODER_Z_EXTI_IRQn EXTI9_5_IRQn

/* ---- USB ---------------------------------------------------------------------- */
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA

/* ---- Pins required by common OpenFFBoard code but not present on this board ----
 * GPIOH pins 2..7 are not bonded out on LQFP64 (only PH0/PH1 exist), so writing
 * them has no physical effect.
 */
#define LED_SYS_Pin GPIO_PIN_2
#define LED_SYS_GPIO_Port GPIOH
#define LED_CLIP_Pin GPIO_PIN_3
#define LED_CLIP_GPIO_Port GPIOH
#define LED_ERR_Pin GPIO_PIN_4
#define LED_ERR_GPIO_Port GPIOH
#define DRV_BRAKE_Pin GPIO_PIN_5     // generic brake output of voltagesense.cpp - real brake is AUX (TIM2)
#define DRV_BRAKE_GPIO_Port GPIOH
#define BUTTON_A_Pin GPIO_PIN_6
#define BUTTON_A_GPIO_Port GPIOH

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

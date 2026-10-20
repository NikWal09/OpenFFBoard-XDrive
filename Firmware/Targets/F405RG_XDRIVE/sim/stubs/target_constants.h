// Simulation stand-in for Targets/F405RG_XDRIVE/Core/Inc/target_constants.h + main.h
#ifndef INC_TARGET_CONSTANTS_H_
#define INC_TARGET_CONSTANTS_H_

#include "sim_hal.h"

#define XDRIVEDRIVER

#define DRV_EN_GATE_Pin GPIO_PIN_12
#define DRV_EN_GATE_GPIO_Port GPIOB
#define DRV_NFAULT_Pin GPIO_PIN_2
#define DRV_NFAULT_GPIO_Port GPIOD
#define DRV_NCS_Pin GPIO_PIN_13
#define DRV_NCS_GPIO_Port GPIOC
#define ENCODER_Z_Pin GPIO_PIN_9
#define ENCODER_Z_GPIO_Port GPIOC

#define ADC1_CHANNELS 3
extern volatile uint32_t ADC1_BUF[ADC1_CHANNELS];

#endif

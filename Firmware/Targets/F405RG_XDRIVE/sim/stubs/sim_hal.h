/*
 * Host simulation of the few STM32 registers / HAL calls used by MotorXDrive.
 */
#ifndef SIM_HAL_H
#define SIM_HAL_H

#include <cstdint>
#include <cstddef>

struct TIM_TypeDef { volatile uint32_t CR1, CCER, BDTR, CCR1, CCR2, CCR3, CCR4, ARR, CNT, RCR, DIER, SR; };
struct ADC_TypeDef { volatile uint32_t SR, CR1, CR2, JDR1; };
struct GPIO_TypeDef { volatile uint32_t IDR, ODR, BSRR; };

extern TIM_TypeDef sim_tim1, sim_tim2, sim_tim3;
extern ADC_TypeDef sim_adc1, sim_adc2, sim_adc3;
extern GPIO_TypeDef sim_gpiob, sim_gpioc, sim_gpiod;

#define TIM1 (&sim_tim1)
#define TIM2 (&sim_tim2)
#define TIM3 (&sim_tim3)
#define ADC1 (&sim_adc1)
#define ADC2 (&sim_adc2)
#define ADC3 (&sim_adc3)
#define GPIOB (&sim_gpiob)
#define GPIOC (&sim_gpioc)
#define GPIOD (&sim_gpiod)

#define TIM_CR1_CEN   (1u << 0)
#define TIM_CR1_DIR   (1u << 4)
#define TIM_SR_UIF    (1u << 0)
#define TIM_DIER_UIE  (1u << 0)
#define ADC_CR2_JEXTEN   (3u << 20)
#define ADC_CR2_JSWSTART (1u << 22)
#define TIM1_UP_TIM10_IRQn 25
#define TIM_BDTR_MOE  (1u << 15)
#define TIM_CCER_CC1E  (1u << 0)
#define TIM_CCER_CC1NE (1u << 2)
#define TIM_CCER_CC2E  (1u << 4)
#define TIM_CCER_CC2NE (1u << 6)
#define TIM_CCER_CC3E  (1u << 8)
#define TIM_CCER_CC3NE (1u << 10)
#define TIM_CCER_CC4E  (1u << 12)
#define ADC_SR_JEOC   (1u << 2)
#define ADC_SR_OVR    (1u << 5)
#define ADC_CR1_OVRIE (1u << 26)

#define GPIO_PIN_2  ((uint16_t)0x0004)
#define GPIO_PIN_9  ((uint16_t)0x0200)
#define GPIO_PIN_12 ((uint16_t)0x1000)
#define GPIO_PIN_13 ((uint16_t)0x2000)
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

struct SPI_HandleTypeDef { int dummy; };
struct ADC_HandleTypeDef { ADC_TypeDef* Instance; };
typedef int IRQn_Type;
#define ADC_IRQn 18

void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef* h, uint8_t* tx, uint8_t* rx, uint16_t size, uint32_t timeout);
int HAL_ADCEx_InjectedStart(ADC_HandleTypeDef* h);
int HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* h);
int HAL_ADC_Stop_DMA(ADC_HandleTypeDef* h);
int HAL_ADC_Start_DMA(ADC_HandleTypeDef* h, uint32_t* buf, uint32_t len);
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t pre, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
void HAL_NVIC_DisableIRQ(IRQn_Type irq);
uint32_t HAL_GetTick();

inline uint32_t __get_PRIMASK(){ return 0; }
inline void __disable_irq(){}
inline void __set_PRIMASK(uint32_t){}

#endif

/*
 * target_constants.h
 *
 * MKS XDrive Mini V1.0 (MKS ODrive Mini) - STM32F405RGT6
 * Single axis direct drive wheel: FOC driver MotorXDrive (DRV8301 + MT6701 ABZ encoder).
 * No buttons, analog inputs or CAN - those are handled by separate controllers.
 */

#ifndef INC_TARGET_CONSTANTS_H_
#define INC_TARGET_CONSTANTS_H_

#include "main.h"

// Hardware name string
#define HW_TYPE "MKS XDrive Mini (F405)"
#define HW_TYPE_INT 10
#define FW_DEVID 0x413 // F405/F407 share the same device id

// Main classes
#define DEFAULTMAIN 1 // FFBWheel
#define FFBWHEEL
//#define FFBWHEEL_USE_1AXIS_DESC

// Motor driver of this board
#define XDRIVEDRIVER
#define DEFAULT_DRVTYPE 13 // MotorXDrive, see MotorDriver::all_drivers
#define FORCE_DEFAULT_DRVTYPE // drvtype None/invalid in flash -> MotorXDrive (only driver on this board)

// Endstop at full ilimit regardless of axis.power, with damping (see Axis::updateTorque)
#define ENDSTOP_FULL_TORQUE
#define ENDSTOP_DAMPING 0.5f // per deg/s and esgain step: esgain 127 -> full torque at ~500 deg/s

//----------------------
#define TIM_PWM htim1 // 3 phase bridge, used directly by MotorXDrive
#define TIM_ENC htim3 // MT6701 ABZ
#define TIM_BRAKE htim2 // AUX brake resistor half bridge
#define TIM_MICROS_HALTICK htim7 // HAL tick timer, counts microseconds
#define TIM_FFB htim13 // FFB update rate timer (1us steps)

// ADC1: VBUS, chip temperature, VREFINT. Triggered by TIM4 CC4 at 10kHz
#define ADC1_CHANNELS 3
extern ADC_HandleTypeDef hadc1;
extern volatile uint32_t ADC1_BUF[ADC1_CHANNELS];
#define VSENSE_HADC hadc1
#define VSENSE_ADC_BUF ADC1_BUF
#define ADC_CHAN_VINT 0 // Only one bus voltage sense on this board
#define ADC_CHAN_VEXT 0
#define TEMPSENSOR_ADC_VAL ADC1_BUF[1]
#define ADC_INTREF_VAL ADC1_BUF[2]
#define VOLTAGE_MULT_DEFAULT 19.0 // 18k/1k divider: mV at pin * 19 = bus mV

// Required by headers of features that are disabled on this board
#define UART_BUF_SIZE 1
#define ADC_PINS 0 // No analog input pins (pedals are a separate device)
#define BUTTON_PINS 0 // No button pins (wheel buttons are a separate device)
#define AIN_HADC hadc1 // LocalAnalog is always compiled; with ADC_PINS 0 it reads nothing
#define ADC_CHAN_FPIN 0

// Flash. 2 pages used, sectors 1 and 2
#define USE_EEPROM_EMULATION
#define PAGE0_ID               FLASH_SECTOR_1
#define PAGE1_ID               FLASH_SECTOR_2
#define EEPROM_START_ADDRESS  ((uint32_t)0x08004000) /* EEPROM emulation start address: from sector1*/
#define PAGE_SIZE             (uint32_t)0x4000  /* Page size = 16KByte */

// System
// BKPSRAM positions
#define DFU_JUMP_MAGIC_ADR BKPSRAM_BASE + 0

#define CCRAM_SEC ".ccmram"

#define SIGBNATUREBASEADR 0 // First block in OTP

#endif /* INC_TARGET_CONSTANTS_H_ */

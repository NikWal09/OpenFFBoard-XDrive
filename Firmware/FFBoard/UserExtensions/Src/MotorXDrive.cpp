/*
 * MotorXDrive.cpp
 *
 * Onboard FOC driver for the MKS XDrive Mini V1.0. See MotorXDrive.h for an overview.
 *
 * Sign conventions:
 *  - Positive iq produces torque towards increasing electrical angle.
 *  - sensorDirection is chosen by the calibration so that sensorDirection*raw also
 *    increases in that direction. getPos() = posSign()*(raw-centerRaw) therefore grows
 *    with positive torque, which the OpenFFBoard axis expects.
 *  - invert flips position and torque together, so the loop stays stable.
 */

#include "MotorXDrive.h"
#ifdef XDRIVEDRIVER

#include <cmath>
#include "flash_helpers.h"
#include "eeprom_addresses.h"
#include "critical.hpp"

extern SPI_HandleTypeDef hspi3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern "C" volatile uint8_t adc1Overrun; // set by ADC_IRQHandler
extern ADC_HandleTypeDef hadc3;

// ---- hardware constants -------------------------------------------------------------
static constexpr float PWM_FREQ = 20000.0f;
static constexpr float TS = 1.0f / PWM_FREQ;
static constexpr uint32_t PWM_ARR = 4200;           // TIM1 auto reload (center aligned)
static constexpr float SHUNT_OHM = 0.0005f;         // R16/R17
static constexpr float AMP_GAIN = 40.0f;            // DRV8301 CR2 GAIN = 40 V/V
static constexpr float ADC_TO_AMP = 3.3f / 4096.0f / (AMP_GAIN * SHUNT_OHM);
static constexpr float VBUS_SCALE = 3.3f / 4096.0f * 19.0f; // 18k/1k divider
static constexpr float VBUS_MIN = 10.0f;            // below: no torque
static constexpr float VBUS_MAX = 25.0f;            // above: overvoltage fault (19.5V supply)
static constexpr float BRAKE_ON_V = 21.0f;          // brake resistor on above
static constexpr float BRAKE_OFF_V = 20.2f;         // and off below
static constexpr uint32_t BRAKE_DEADTIME = 42;      // TIM2 ticks @84MHz = 0.5us gap between AUX_L and AUX_H
static constexpr uint32_t BRAKE_MAX_TICKS = 5 * 20000; // 5 s continuous braking = fault
static constexpr float MAX_DUTY = 0.92f;            // keeps the low side on long enough to sample current
static constexpr float TWO_PI = 6.28318531f;

static constexpr uint32_t ms(uint32_t t) { return t * 20; } // ISR ticks

// DRV8301 registers
static constexpr uint8_t DRV_STATUS1 = 0x00;
static constexpr uint8_t DRV_STATUS2 = 0x01;
static constexpr uint8_t DRV_CTRL1 = 0x02;
static constexpr uint8_t DRV_CTRL2 = 0x03;
// CTRL1: gate current 1.7A, 6-PWM, OCP latch shutdown, OC_ADJ 9 (0.175V Vds)
static constexpr uint16_t DRV_CTRL1_VAL = (9 << 6) | (1 << 4);
// CTRL2: report OT+OC on nOCTW, gain 40 V/V, no DC cal, cycle by cycle OC off time
static constexpr uint16_t DRV_CTRL2_VAL = (2 << 2);

// ---- class registration ------------------------------------------------------------
bool MotorXDrive::inUse = false;
MotorXDrive* MotorXDrive::instance = nullptr;
ClassIdentifier MotorXDrive::info = {
	.name = "MKS XDrive (FOC)",
	.id = CLSID_MOT_XDRIVE,
};

const ClassIdentifier MotorXDrive::getInfo(){
	return info;
}

// ---- fast sine ----------------------------------------------------------------------
static constexpr uint32_t SIN_LUT_SIZE = 1024;
static float sinLut[SIN_LUT_SIZE + 1];
static bool sinLutReady = false;

float MotorXDrive::fastSin(float x){
	float idx = x * ((float)SIN_LUT_SIZE / TWO_PI);
	float fl = floorf(idx);
	float frac = idx - fl;
	uint32_t i = (uint32_t)((int32_t)fl) & (SIN_LUT_SIZE - 1);
	return sinLut[i] + (sinLut[i + 1] - sinLut[i]) * frac;
}

// ---- flash writes stall the CPU (and this ISR): keep the bridge off meanwhile ------------
static volatile uint32_t flashBusy = 0;
void Flash_WriteBegin(){
	flashBusy++;
	TIM1->BDTR &= ~TIM_BDTR_MOE; // all FETs off before the flash stalls code execution
}
void Flash_WriteEnd(){
	if(flashBusy) flashBusy--;
}

// ---- ISR glue -------------------------------------------------------------------------
/*
 * Fallback trigger (see enableSoftTrigger): TIM1 update interrupt. At the update after the
 * counter peak (counting down) the low side FETs conduct: start both current conversions.
 */
extern "C" void XDrive_Tim1Update(void){
	if(MotorXDrive::instance && MotorXDrive::instance->softTrigger && (TIM1->CR1 & TIM_CR1_DIR)){
		ADC2->CR2 |= ADC_CR2_JSWSTART;
		ADC3->CR2 |= ADC_CR2_JSWSTART;
	}
}

void MotorXDrive::enableSoftTrigger(){
	ADC2->CR2 &= ~ADC_CR2_JEXTEN; // software start only works without an external trigger
	ADC3->CR2 &= ~ADC_CR2_JEXTEN;
	TIM1->RCR = 0;                // update event at both the peak and the valley
	softTrigger = true;
	TIM1->SR = ~TIM_SR_UIF;
	TIM1->DIER |= TIM_DIER_UIE;
	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

extern "C" void XDrive_AdcIsr(void){
	if(MotorXDrive::instance){
		MotorXDrive::instance->isr();
	}else{
		ADC2->SR = ~ADC_SR_JEOC;
		ADC3->SR = ~ADC_SR_JEOC;
	}
}

// ---- construction -----------------------------------------------------------------------
MotorXDrive::MotorXDrive() : CommandHandler("xdrive", CLSID_MOT_XDRIVE, 0),
		Thread("XDRIVE", XDRIVE_THREAD_MEM, XDRIVE_THREAD_PRIO) {
	// Note: called inside a critical section by the axis -> no blocking calls here
	inUse = true;
	if(!sinLutReady){
		for(uint32_t i = 0; i <= SIN_LUT_SIZE; i++){
			sinLut[i] = sinf(TWO_PI * (float)i / (float)SIN_LUT_SIZE);
		}
		sinLutReady = true;
	}
	restoreFlash();
	computeGains();
	lastCnt = TIM3->CNT;
	raw = 0;
	centerRaw = 0; // power up position is the center until the index restores the saved one
	registerCommands();
	instance = this;
	this->Start();
}

MotorXDrive::~MotorXDrive(){
	bridgeDisable();
	HAL_GPIO_WritePin(DRV_EN_GATE_GPIO_Port, DRV_EN_GATE_Pin, GPIO_PIN_RESET);
	HAL_NVIC_DisableIRQ(ADC_IRQn);
	instance = nullptr;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0xFFFF;
	inUse = false;
}

void MotorXDrive::registerCommands(){
	CommandHandler::registerCommands();
	registerCommand("poles", XDrive_commands::poles, "Motor pole pairs", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("cpr", XDrive_commands::cpr, "Encoder counts per revolution", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("vlimit", XDrive_commands::vlimit, "Max phase voltage (mV)", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("ilimit", XDrive_commands::ilimit, "Max phase current (mA)", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("sensorDirection", XDrive_commands::sensorDirection, "Encoder direction -1/1 (set by calibration)", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("invert", XDrive_commands::invert, "Invert torque and position", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("offset", XDrive_commands::offset, "Electrical offset (encoder counts)", CMDFLAG_GET);
	registerCommand("cal", XDrive_commands::cal, "Run full motor calibration", CMDFLAG_GET);
	registerCommand("test", XDrive_commands::test, "Self test: +360, -720, +360 deg", CMDFLAG_GET);
	registerCommand("mode", XDrive_commands::mode, "0 voltage, 1 current FOC", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("index", XDrive_commands::index, "Restore center from encoder index", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("state", XDrive_commands::state, "Driver state", CMDFLAG_GET);
	registerCommand("errors", XDrive_commands::errors, "Error flags. Set 0 to clear", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("vbus", XDrive_commands::vbus, "Bus voltage (mV)", CMDFLAG_GET);
	registerCommand("iq", XDrive_commands::iq, "Measured torque current (mA)", CMDFLAG_GET);
	registerCommand("res", XDrive_commands::res, "Phase resistance (0.1 mOhm)", CMDFLAG_GET);
	registerCommand("ind", XDrive_commands::ind, "Phase inductance (uH)", CMDFLAG_GET);
	registerCommand("bw", XDrive_commands::bw, "Current loop bandwidth (rad/s)", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("drvstatus", XDrive_commands::drvstatus, "DRV8301 status registers", CMDFLAG_GET);
	registerCommand("brake", XDrive_commands::brake, "Brake resistor duty (%)", CMDFLAG_GET);
	registerCommand("center", XDrive_commands::center, "Center relative to index (counts)", CMDFLAG_GET);
	registerCommand("indexfound", XDrive_commands::indexfound, "Encoder index seen since power up", CMDFLAG_GET);
	registerCommand("calratio", XDrive_commands::calratio, "Measured/expected encoder counts in calibration (x1000)", CMDFLAG_GET);
	registerCommand("isrcount", XDrive_commands::isrcount, "Current loop interrupts since power up", CMDFLAG_GET);
	registerCommand("trigger", XDrive_commands::trigger, "ADC trigger: 0 TIM1 TRGO, 1 software fallback", CMDFLAG_GET);
	registerCommand("dbg", XDrive_commands::dbg, "Registers: TIM1 CR1 CNT CCER BDTR, ADC2 CR2 SR, ADC3 CR2", CMDFLAG_GET);
}

// ---- flash ------------------------------------------------------------------------------
void MotorXDrive::saveFlash(){
	uint16_t conf = 0;
	conf |= (sensorDirection < 0) ? 1 << 0 : 0;
	conf |= invert ? 1 << 1 : 0;
	conf |= calibrated ? 1 << 2 : 0;
	conf |= useIndex ? 1 << 3 : 0;
	conf |= (currentSign < 0) ? 1 << 4 : 0;
	conf |= swapBC ? 1 << 5 : 0;
	conf |= (mode & 0x3) << 6;
	Flash_Write(ADR_XDRIVE_CONF1, conf);
	Flash_Write(ADR_XDRIVE_POLES, poles);
	Flash_Write(ADR_XDRIVE_CPR, cpr);
	Flash_Write(ADR_XDRIVE_VLIMIT, vlimit_mV);
	Flash_Write(ADR_XDRIVE_ILIMIT, ilimit_mA);
	Flash_Write(ADR_XDRIVE_OFFSET, (uint16_t)(elecOffsetRaw & 0xffff));
	Flash_Write(ADR_XDRIVE_RES, res_100uOhm);
	Flash_Write(ADR_XDRIVE_IND, ind_uH);
	Flash_Write(ADR_XDRIVE_BW, bandwidth);
	Flash_Write(ADR_XDRIVE_CENTER, (uint16_t)centerFromIndex);
}

void MotorXDrive::restoreFlash(){
	uint16_t v = 0;
	if(Flash_Read(ADR_XDRIVE_CONF1, &v)){
		sensorDirection = (v & (1 << 0)) ? -1 : 1;
		invert = v & (1 << 1);
		calibrated = v & (1 << 2);
		useIndex = v & (1 << 3);
		currentSign = (v & (1 << 4)) ? -1 : 1;
		swapBC = v & (1 << 5);
		mode = (v >> 6) & 0x3;
		if(mode > 1) mode = 1;
	}
	if(Flash_Read(ADR_XDRIVE_POLES, &v) && v > 0 && v <= 64) poles = v;
	if(Flash_Read(ADR_XDRIVE_CPR, &v) && v >= 64) cpr = v;
	if(Flash_Read(ADR_XDRIVE_VLIMIT, &v) && v >= 500) vlimit_mV = v;
	if(Flash_Read(ADR_XDRIVE_ILIMIT, &v) && v >= 500) ilimit_mA = v;
	if(Flash_Read(ADR_XDRIVE_RES, &v)) res_100uOhm = v;
	if(Flash_Read(ADR_XDRIVE_IND, &v)) ind_uH = v;
	if(Flash_Read(ADR_XDRIVE_BW, &v) && v >= 100) bandwidth = v;
	if(Flash_Read(ADR_XDRIVE_CENTER, &v)) centerFromIndex = (int16_t)v;
}

void MotorXDrive::computeGains(){
	float r = res_100uOhm > 0 ? res_100uOhm * 1e-4f : 0.2f;
	float l = ind_uH > 0 ? ind_uH * 1e-6f : 400e-6f;
	kp = l * (float)bandwidth;
	ki = r * (float)bandwidth;
}

// ---- DRV8301 ----------------------------------------------------------------------------
uint16_t MotorXDrive::drvTransfer(uint16_t word){
	uint16_t rx = 0;
	HAL_GPIO_WritePin(DRV_NCS_GPIO_Port, DRV_NCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&word, (uint8_t*)&rx, 1, 10);
	HAL_GPIO_WritePin(DRV_NCS_GPIO_Port, DRV_NCS_Pin, GPIO_PIN_SET);
	for(volatile int i = 0; i < 50; i++); // nSCS high time > 500 ns
	return rx;
}

uint16_t MotorXDrive::drvRead(uint8_t addr){
	uint16_t cmd = 0x8000 | ((addr & 0xf) << 11);
	drvTransfer(cmd);
	return drvTransfer(cmd) & 0x7ff; // answer comes with the next frame
}

void MotorXDrive::drvWrite(uint8_t addr, uint16_t data){
	drvTransfer(((addr & 0xf) << 11) | (data & 0x7ff));
}

void MotorXDrive::setupGateDriver(){
	HAL_GPIO_WritePin(DRV_EN_GATE_GPIO_Port, DRV_EN_GATE_Pin, GPIO_PIN_RESET);
	Delay(5);
	HAL_GPIO_WritePin(DRV_EN_GATE_GPIO_Port, DRV_EN_GATE_Pin, GPIO_PIN_SET);
	Delay(20); // DRV8301 power up
	drvWrite(DRV_CTRL1, DRV_CTRL1_VAL);
	drvWrite(DRV_CTRL2, DRV_CTRL2_VAL);
	uint16_t c1 = drvRead(DRV_CTRL1);
	uint16_t c2 = drvRead(DRV_CTRL2);
	drvStatus1 = drvRead(DRV_STATUS1);
	drvStatus2 = drvRead(DRV_STATUS2);
	if(c1 != DRV_CTRL1_VAL || c2 != DRV_CTRL2_VAL){
		enterFault(XDriveError::drvSpi);
	}
}

void MotorXDrive::startPwmAndAdc(){
	bridgeDisable();
	// Enable all channels (incl. CH4 = ADC trigger, its pin PA11 belongs to USB) but keep MOE off
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE
			| TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC4E;
	TIM1->CR1 |= TIM_CR1_CEN;
	HAL_ADCEx_InjectedStart(&hadc3);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	HAL_NVIC_SetPriority(ADC_IRQn, 1, 0); // above FreeRTOS syscall priority: never call RTOS from the ISR
	HAL_NVIC_EnableIRQ(ADC_IRQn);
}

void MotorXDrive::bridgeEnable(){
	if(!(TIM1->BDTR & TIM_BDTR_MOE)){
		TIM1->CCR1 = PWM_ARR / 2;
		TIM1->CCR2 = PWM_ARR / 2;
		TIM1->CCR3 = PWM_ARR / 2;
		TIM1->BDTR |= TIM_BDTR_MOE;
	}
}

void MotorXDrive::bridgeDisable(){
	TIM1->BDTR &= ~TIM_BDTR_MOE; // outputs go to idle level: all FETs off
	TIM1->CCR1 = PWM_ARR / 2;
	TIM1->CCR2 = PWM_ARR / 2;
	TIM1->CCR3 = PWM_ARR / 2;
}

// ---- modulation ----------------------------------------------------------------------
void MotorXDrive::applyVoltageAB(float valpha, float vbeta, bool compensate){
	if(vbus < 1.0f){
		bridgeDisable();
		return;
	}
	float va = valpha;
	float vb = -0.5f * valpha + 0.8660254f * vbeta;
	float vc = -0.5f * valpha - 0.8660254f * vbeta;
	if(compensate && deadComp > 0){
		// Dead time: during the blanking time the diodes set the phase voltage, which
		// lowers it by ~deadComp in the direction of the phase current. Add it back.
		auto sgn = [](float i){ return fminf(fmaxf(i * 5.0f, -1.0f), 1.0f); }; // linear within +-0.2 A
		va += deadComp * sgn(lastIa);
		vb += deadComp * sgn(lastIb);
		vc += deadComp * sgn(lastIc);
	}
	float vmax = fmaxf(va, fmaxf(vb, vc));
	float vmin = fminf(va, fminf(vb, vc));
	float vcm = 0.5f * (vmax + vmin); // space vector: center the phase voltages
	float inv = 1.0f / vbus;
	float da = fminf(fmaxf(0.5f + (va - vcm) * inv, 0.0f), MAX_DUTY);
	float db = fminf(fmaxf(0.5f + (vb - vcm) * inv, 0.0f), MAX_DUTY);
	float dc = fminf(fmaxf(0.5f + (vc - vcm) * inv, 0.0f), MAX_DUTY);
	TIM1->CCR1 = (uint32_t)(da * PWM_ARR);
	TIM1->CCR2 = (uint32_t)(db * PWM_ARR);
	TIM1->CCR3 = (uint32_t)(dc * PWM_ARR);
	bridgeEnable();
}

void MotorXDrive::applyVoltageDQ(float vd, float vq, float theta){
	float c = fastCos(theta), s = fastSin(theta);
	applyVoltageAB(vd * c - vq * s, vd * s + vq * c);
}

float MotorXDrive::electricalAngle(int32_t rawpos){
	int64_t e = (int64_t)(sensorDirection * rawpos - elecOffsetRaw) * poles;
	int32_t m = (int32_t)(e % cpr);
	if(m < 0) m += cpr;
	return (float)m * (TWO_PI / (float)cpr);
}

int32_t MotorXDrive::rawSnapshot(){
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	uint16_t cnt = TIM3->CNT;
	int32_t r = raw + (int16_t)(cnt - lastCnt);
	__set_PRIMASK(primask);
	return r;
}

// ---- brake resistor --------------------------------------------------------------------
void MotorXDrive::updateBrake(){
	if(errors & XDriveError::brakeTimeout){
		braking = false;
	}else if(!braking && vbus > BRAKE_ON_V){
		braking = true;
	}else if(braking && vbus < BRAKE_OFF_V){
		braking = false;
	}
	if(braking){
		if(++brakeOnTicks > BRAKE_MAX_TICKS){
			braking = false;
			enterFault(XDriveError::brakeTimeout);
		}
		brakeDuty = fminf(fmaxf((vbus - BRAKE_OFF_V) / 1.5f, 0.05f), 0.9f);
	}else{
		brakeOnTicks = 0;
		brakeDuty = 0;
	}
	if(brakeDuty <= 0){
		TIM2->CCR3 = 0;      // AUX_L never on
		TIM2->CCR4 = 0xFFFF; // AUX_H never on
	}else{
		// Center aligned TIM2: AUX_H (resistor) on around the top (CNT >= CCR4),
		// AUX_L around the bottom (CNT < CCR3) recharges the bootstrap capacitor.
		// CCR3 = CCR4 - dead time keeps a gap on both the rising and the falling slope.
		uint32_t arr = TIM2->ARR;
		uint32_t highOn = (uint32_t)((1.0f - brakeDuty) * arr);
		if(highOn < BRAKE_DEADTIME) highOn = BRAKE_DEADTIME;
		// CCR preload is on: both values are latched together at the next update event
		TIM2->CCR4 = highOn;
		TIM2->CCR3 = highOn - BRAKE_DEADTIME;
	}
}

void MotorXDrive::setErrorFlag(uint32_t flags, bool set){
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	errors = set ? (errors | flags) : (errors & ~flags);
	__set_PRIMASK(primask);
}

void MotorXDrive::startAdcCal(){
	bridgeDisable();
	adcAccB = adcAccC = 0;
	adcCount = 0;
	state = XDriveState::AdcCal;
}

/*
 * verrAlpha: voltage error measured on the alpha axis with currents (+I, -I/2, -I/2).
 * Per phase error V gives 4/3*V on alpha, so the per phase compensation is 3/4 of it.
 */
void MotorXDrive::updateDeadComp(float verrAlpha){
	deadComp = fminf(fmaxf(0.75f * verrAlpha, 0.0f), 0.6f);
}

void MotorXDrive::enterFault(uint32_t err){
	errors |= err;
	bridgeDisable();
	state = XDriveState::Fault;
}

// ---- 20 kHz control loop ---------------------------------------------------------------
void MotorXDrive::isr(){
	isrCount = isrCount + 1;
	int32_t adcB = ADC2->JDR1; // PC1, DRV8301 SO2
	int32_t adcC = ADC3->JDR1; // PC0, DRV8301 SO1
	ADC2->SR = ~ADC_SR_JEOC;
	ADC3->SR = ~ADC_SR_JEOC;

	uint16_t cnt = TIM3->CNT;
	int16_t d = (int16_t)(cnt - lastCnt);
	lastCnt = cnt;
	raw = raw + d;
	velocity = velocity + 0.01f * ((float)d * PWM_FREQ - velocity);
	vbus = vbus + 0.05f * ((float)ADC1_BUF[0] * VBUS_SCALE - vbus);
	updateBrake();

	if(state == XDriveState::AdcCal){
		adcAccB += adcB;
		adcAccC += adcC;
		if(++adcCount >= 4000){
			adcOffsetB = adcAccB / (int32_t)adcCount;
			adcOffsetC = adcAccC / (int32_t)adcCount;
			if(abs(adcOffsetB - 2048) > 400 || abs(adcOffsetC - 2048) > 400){
				enterFault(XDriveError::adcOffset);
			}else{
				state = XDriveState::Idle;
			}
		}
		return;
	}

	float ib = (float)(adcB - adcOffsetB) * ADC_TO_AMP * currentSign;
	float ic = (float)(adcC - adcOffsetC) * ADC_TO_AMP * currentSign;
	if(swapBC){
		float t = ib; ib = ic; ic = t;
	}
	float ia = -ib - ic;
	lastIa = ia; lastIb = ib; lastIc = ic;
	float ialpha = ia;
	float ibeta = (ib - ic) * 0.57735027f;

	// --- protections ---
	if(vbus > VBUS_MAX && state != XDriveState::Fault){
		enterFault(XDriveError::overvoltage);
	}
	if(state != XDriveState::Init && state != XDriveState::Fault && vbus > VBUS_MIN
			&& !(DRV_NFAULT_GPIO_Port->IDR & DRV_NFAULT_Pin)){
		enterFault(XDriveError::drvFault);
	}
	float iTrip = ilimit_mA * 0.0015f + 5.0f;
	if((fabsf(ia) > iTrip || fabsf(ib) > iTrip || fabsf(ic) > iTrip) && (TIM1->BDTR & TIM_BDTR_MOE)){
		enterFault(XDriveError::overcurrent);
	}
	bool vbusOk = vbus > VBUS_MIN;
	if(vbusOk){
		errors &= ~XDriveError::undervoltage;
	}else{
		errors |= XDriveError::undervoltage;
	}

	if(flashBusy){ // flash program/erase in progress: no output (see Flash_WriteBegin)
		bridgeDisable();
		return;
	}

	switch(state){
	case XDriveState::Align:
	case XDriveState::Calibrating:
		if(!vbusOk){
			bridgeDisable();
			state = XDriveState::Idle;
			break;
		}
		calibrationStep(ialpha, ibeta);
		break;

	case XDriveState::Running:
		if(!enabled || estop || !vbusOk){
			bridgeDisable();
			intD = intQ = 0;
			break;
		}
		runCurrentLoop(iqRef, electricalAngle(raw), ialpha, ibeta);
		break;

	case XDriveState::Test:
		if(estop || !vbusOk){
			bridgeDisable();
			state = XDriveState::Idle;
			break;
		}
		testStep(ialpha, ibeta, electricalAngle(raw));
		break;

	default: // Init, Idle, Fault
		bridgeDisable();
		break;
	}
}

void MotorXDrive::runCurrentLoop(float iqTarget, float theta, float ialpha, float ibeta){
	float c = fastCos(theta), s = fastSin(theta);
	float id = ialpha * c + ibeta * s;
	float iq = -ialpha * s + ibeta * c;
	iqMeas = iqMeas + 0.02f * (iq - iqMeas);
	float vmax = fminf(vlimit_mV * 0.001f, 0.45f * vbus);
	float imax = ilimit_mA * 0.001f;
	iqTarget = fminf(fmaxf(iqTarget, -imax), imax);

	float vd, vq;
	if(mode == 0){
		// Voltage mode: torque roughly proportional to vq at low speed
		vd = 0;
		vq = iqTarget / imax * vmax;
	}else{
		float ed = -id;
		float eq = iqTarget - iq;
		vd = kp * ed + intD;
		vq = kp * eq + intQ;
		float mag = sqrtf(vd * vd + vq * vq);
		if(mag > vmax){
			float k = vmax / mag; // saturated: scale down, do not integrate further
			vd *= k;
			vq *= k;
		}else{
			intD = fminf(fmaxf(intD + ki * TS * ed, -vmax), vmax);
			intQ = fminf(fmaxf(intQ + ki * TS * eq, -vmax), vmax);
		}
	}
	applyVoltageAB(vd * c - vq * s, vd * s + vq * c, true);
}

/*
 * Alignment (fullCal=false) and full calibration (fullCal=true), driven in voltage mode.
 * Current is only measured, so this also works before the current sign is known.
 */
void MotorXDrive::calibrationStep(float ialpha, float ibeta){
	stepTicks++;
	const float ical = fminf(fmaxf(ilimit_mA * 0.0005f, 2.0f), 8.0f);
	const float vcalMax = fminf(vlimit_mV * 0.001f, 0.3f * vbus);
	const float rampPerTick = 2.0f * TS; // 2 V/s
	const float imag = sqrtf(ialpha * ialpha + ibeta * ibeta);
	const float expectedCounts = 2.0f * (float)cpr / (float)poles; // 2 electrical turns

	switch(calStep){
	case XDriveCalStep::RampLow:
		calV += rampPerTick;
		applyVoltageDQ(calV, 0, 0);
		if(imag >= 0.5f * ical){
			calVlow = calV;
			calStep = XDriveCalStep::HoldLow;
			stepTicks = 0; accA = 0; accN = 0;
		}else if(calV >= vcalMax){
			enterFault(XDriveError::calNoCurrent);
		}
		break;

	case XDriveCalStep::HoldLow:
		applyVoltageDQ(calVlow, 0, 0);
		if(stepTicks > ms(300)){
			accA += ialpha; accN++;
		}
		if(stepTicks >= ms(500)){
			calIlow = accA / accN;
			calV = calVlow;
			calStep = XDriveCalStep::RampHigh;
			stepTicks = 0;
		}
		break;

	case XDriveCalStep::RampHigh:
		calV += rampPerTick;
		applyVoltageDQ(calV, 0, 0);
		if(imag >= ical || calV >= vcalMax){
			calVhigh = calV;
			calStep = XDriveCalStep::HoldHigh;
			stepTicks = 0; accA = 0; accN = 0; accRaw = 0;
		}
		break;

	case XDriveCalStep::HoldHigh:
		applyVoltageDQ(calVhigh, 0, 0);
		if(stepTicks > ms(500)){
			accA += ialpha; accRaw += raw; accN++;
		}
		if(stepTicks >= ms(750)){
			calIhigh = accA / accN;
			int32_t alignedRaw = (int32_t)(accRaw / (int64_t)accN);
			if(!fullCal){
				// Dead time voltage from this alignment and the stored resistance
				if(res_100uOhm > 0){
					updateDeadComp(calVhigh - res_100uOhm * 1e-4f * calIhigh);
				}
				// Power up alignment: electrical angle 0 is here
				elecOffsetRaw = sensorDirection * alignedRaw;
				aligned = true;
				calStep = XDriveCalStep::Done;
				intD = intQ = 0;
				state = enabled && !estop ? XDriveState::Running : XDriveState::Idle;
				if(state == XDriveState::Idle) bridgeDisable();
				break;
			}
			// Current sign: positive valpha must give positive ialpha
			if(calIhigh < 0){
				currentSign = -currentSign;
				calIhigh = -calIhigh;
				calIlow = -calIlow;
			}
			// Two point resistance measurement (dead time voltage error cancels out)
			float di = calIhigh - calIlow;
			float r = di > 0.3f ? (calVhigh - calVlow) / di : calVhigh / fmaxf(calIhigh, 0.1f);
			if(r < 0.005f || r > 5.0f){
				enterFault(XDriveError::calResistance);
				break;
			}
			res_100uOhm = (uint16_t)fminf(r * 1e4f, 65000.0f);
			calR = r;
			calVerr = calVhigh - r * calIhigh; // dead time / FET voltage error, removed from the L measurement
			updateDeadComp(calVerr);
			calStep = XDriveCalStep::Inductance;
			stepTicks = 0; lSum = 0; lDiSum = 0; accN = 0;
			calV = fmaxf(0.5f * calVhigh, 0.3f); // step size
		}
		break;

	case XDriveCalStep::Inductance: {
		// Voltage steps of +-calV around the hold voltage, 40 ticks each, 40 repetitions.
		// RL circuit: L*di/dt = V - Verr - R*i  ->  L = sum((V - Verr - R*i_avg)*dt) / sum(di),
		// evaluated between 2 and 12 ticks after every step. Ratio of sums: unbiased with noisy di.
		uint32_t phase = stepTicks % 80;
		bool high = phase < 40;
		float v = calVhigh + (high ? calV : -calV);
		applyVoltageDQ(v, 0, 0);
		uint32_t inStep = phase % 40;
		if(stepTicks > 80){ // the very first step is only calV high, skip it
			if(inStep == 2){
				lPrevI = ialpha;
			}else if(inStep == 12){
				float di = ialpha - lPrevI;
				float iavg = 0.5f * (ialpha + lPrevI);
				float flux = (v - calVerr - calR * iavg) * 10.0f * TS; // volt-seconds
				lSum += high ? flux : -flux;
				lDiSum += high ? di : -di;
				accN++;
			}
		}
		if(stepTicks >= 80 * 41){
			float l = lDiSum > 0.1f ? lSum / lDiSum : 0;
			if(l < 10e-6f || l > 20e-3f){
				l = 400e-6f; // implausible, keep a safe default
			}
			ind_uH = (uint16_t)fminf(l * 1e6f, 65000.0f);
			calStep = XDriveCalStep::Sign90;
			stepTicks = 0; accB = 0; accN = 0;
		}
		break;
	}

	case XDriveCalStep::Sign90: {
		// Turn the field to +90 deg electrical; ibeta must become positive
		float t = fminf((float)stepTicks / (float)ms(500), 1.0f);
		calTheta = 1.5707963f * t;
		applyVoltageDQ(calVhigh, 0, calTheta);
		if(stepTicks > ms(800)){
			accB += ibeta; accN++;
		}
		if(stepTicks >= ms(1000)){
			if(accB / accN < 0){
				swapBC = !swapBC;
			}
			sweepStartRaw = raw;
			calStep = XDriveCalStep::SweepFwd;
			stepTicks = 0;
		}
		break;
	}

	case XDriveCalStep::SweepFwd: {
		// Two electrical turns forward in 2 s, then wait 0.3 s
		float t = fminf((float)stepTicks / (float)ms(2000), 1.0f);
		calTheta = 1.5707963f + 2.0f * TWO_PI * t;
		applyVoltageDQ(calVhigh, 0, calTheta);
		if(stepTicks >= ms(2300)){
			int32_t delta = raw - sweepStartRaw;
			// The rotor follows the field closely, so the ratio is accurate to ~1%.
			// One pole pair off (14 instead of 15) is 7% and must be rejected: the
			// electrical angle would drift a full turn per revolution and reverse the torque.
			float ratio = fabsf((float)delta) / expectedCounts;
			calRatio = ratio;
			if(ratio < 0.96f || ratio > 1.04f){
				enterFault(XDriveError::calCprPoles);
				break;
			}
			sensorDirection = delta > 0 ? 1 : -1;
			calStep = XDriveCalStep::SweepBack;
			stepTicks = 0;
		}
		break;
	}

	case XDriveCalStep::SweepBack: {
		// Back to electrical angle 0 (2 turns + 90 deg) in 2.25 s
		float t = fminf((float)stepTicks / (float)ms(2250), 1.0f);
		calTheta = (1.5707963f + 2.0f * TWO_PI) * (1.0f - t);
		applyVoltageDQ(calVhigh, 0, calTheta);
		if(stepTicks >= ms(2250)){
			calStep = XDriveCalStep::Settle;
			stepTicks = 0; accRaw = 0; accN = 0;
		}
		break;
	}

	case XDriveCalStep::Settle:
		applyVoltageDQ(calVhigh, 0, 0);
		if(stepTicks > ms(500)){
			accRaw += raw; accN++;
		}
		if(stepTicks >= ms(750)){
			elecOffsetRaw = sensorDirection * (int32_t)(accRaw / (int64_t)accN);
			calibrated = true;
			aligned = true;
			computeGains();
			saveCalPending = true;
			calStep = XDriveCalStep::Done;
			intD = intQ = 0;
			state = enabled && !estop ? XDriveState::Running : XDriveState::Idle;
			if(state == XDriveState::Idle) bridgeDisable();
		}
		break;

	default:
		state = XDriveState::Idle;
		break;
	}
}

/*
 * Self test from the specification: +360 deg right, 720 deg left, +360 deg right (back to 0).
 * Smooth cosine profiles, PD position loop on top of the current loop.
 */
void MotorXDrive::testStep(float ialpha, float ibeta, float theta){
	float t = (float)(testTicks++) * TS;
	float target;
	auto smooth = [](float x){ return 0.5f * (1.0f - fastCos(3.14159265f * fminf(fmaxf(x, 0.0f), 1.0f))); };
	if(t < 3.0f)        target = 360.0f * smooth(t / 3.0f);
	else if(t < 3.5f)   target = 360.0f;
	else if(t < 9.5f)   target = 360.0f - 720.0f * smooth((t - 3.5f) / 6.0f);
	else if(t < 10.0f)  target = -360.0f;
	else if(t < 13.0f)  target = -360.0f + 360.0f * smooth((t - 10.0f) / 3.0f);
	else                target = 0;

	float pos = (float)(sensorDirection * (raw - testStartRaw)) * 360.0f / (float)cpr;
	float vel = (float)sensorDirection * velocity * 360.0f / (float)cpr;
	float err = target - pos;
	if(fabsf(err) > 90.0f){
		enterFault(XDriveError::testAborted);
		return;
	}
	float imax = fminf(ilimit_mA * 0.001f, 6.0f);
	float iq = fminf(fmaxf(testKp * err - testKd * vel, -imax), imax);
	runCurrentLoop(iq, theta, ialpha, ibeta);
	if(t >= 13.5f){
		intD = intQ = 0;
		state = enabled && !estop ? XDriveState::Running : XDriveState::Idle;
		if(state == XDriveState::Idle) bridgeDisable();
	}
}

void MotorXDrive::startAlign(bool full){
	bridgeDisable();
	fullCal = full;
	calStep = XDriveCalStep::RampLow;
	stepTicks = 0;
	calV = 0;
	accA = accB = 0;
	accRaw = 0;
	accN = 0;
	intD = intQ = 0;
	if(full){
		aligned = false;
	}
	state = full ? XDriveState::Calibrating : XDriveState::Align; // last: ISR picks it up
}

// ---- management thread ---------------------------------------------------------------
void MotorXDrive::reportErrors(){
	// Report new latched errors once to the OpenFFBoard error list
	uint32_t newErr = errors & XDriveError::latched & ~reportedErrors;
	if(newErr){
		reportedErrors |= newErr;
		if(newErr & XDriveError::overvoltage){
			ErrorHandler::addError(Error(ErrorCode::overvoltage, ErrorType::critical, "XDrive bus overvoltage"));
		}
		if(newErr & XDriveError::brakeTimeout){
			ErrorHandler::addError(Error(ErrorCode::brakeResistorFailure, ErrorType::critical, "XDrive brake resistor on too long"));
		}
		if(newErr & XDriveError::adcOffset){
			ErrorHandler::addError(Error(ErrorCode::adcCalibrationError, ErrorType::critical, "XDrive current sensor offset"));
		}
		if(newErr & (XDriveError::calNoCurrent | XDriveError::calCprPoles | XDriveError::calResistance)){
			ErrorHandler::addError(Error(ErrorCode::encoderAlignmentFailed, ErrorType::critical, "XDrive calibration failed"));
		}
		if(newErr & XDriveError::noCurrentIsr){
			ErrorHandler::addError(Error(ErrorCode::adcCalibrationError, ErrorType::critical, "XDrive no current sampling interrupt"));
		}
		if(newErr & (XDriveError::drvFault | XDriveError::drvSpi | XDriveError::overcurrent | XDriveError::testAborted)){
			ErrorHandler::addError(Error(ErrorCode::systemError, ErrorType::critical, "XDrive driver fault"));
		}
	}
}

void MotorXDrive::Run(){
	Delay(100);
	startPwmAndAdc(); // ISR runs from now on, bridge stays off in Init

	uint32_t lastStatusRead = 0;
	while(true){
		Delay(20);
		reportErrors();
		if(adc1Overrun){ // bus voltage DMA stopped after an overrun: restart it
			adc1Overrun = 0;
			HAL_ADC_Stop_DMA(&hadc1);
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_BUF, ADC1_CHANNELS);
		}
		float vbusNow = (float)ADC1_BUF[0] * VBUS_SCALE;

		// The DRV8301 is powered from the bus: configure it only when the supply is on,
		// and again after the supply was off (its registers reset on undervoltage).
		if(gateReady && vbusNow < VBUS_MIN - 1.0f){
			gateReady = false;
			bridgeDisable();
			if(state != XDriveState::Fault){
				state = XDriveState::Init;
			}
		}
		if(!gateReady){
			if(vbusNow > VBUS_MIN + 1.0f && state != XDriveState::Fault){
				Delay(200); // let the supply settle
				setupGateDriver();
				if(state != XDriveState::Fault){
					gateReady = true;
					startAdcCal();
				}
			}
			continue;
		}

		XDriveState s = state;
		// Current sampling must run at 20 kHz. If the ADC trigger from TIM1 does not arrive,
		// switch to the software trigger once, then give up with an error.
		if(s == XDriveState::AdcCal){
			if(!adcCalWatch){
				adcCalWatch = HAL_GetTick();
				adcCalIsrStart = isrCount;
			}else if(HAL_GetTick() - adcCalWatch > 1000 && isrCount - adcCalIsrStart < 1000){
				if(!softTrigger){
					enableSoftTrigger();
					adcCalWatch = 0;
				}else{
					enterFault(XDriveError::noCurrentIsr);
				}
			}
		}else{
			adcCalWatch = 0;
		}
		if(requestCal && (s == XDriveState::Idle || s == XDriveState::Running)){
			requestCal = false;
			startAlign(true);
			continue;
		}
		if(requestTest && aligned && (s == XDriveState::Idle || s == XDriveState::Running)){
			requestTest = false;
			testStartRaw = rawSnapshot();
			testTicks = 0;
			intD = intQ = 0;
			state = XDriveState::Test;
			continue;
		}
		if(s == XDriveState::Idle && calibrated && !aligned && enabled && !estop
				&& vbus > VBUS_MIN && !(errors & XDriveError::latched)){
			startAlign(false); // power up alignment
		}else if(s == XDriveState::Idle && aligned && enabled && !estop){
			state = XDriveState::Running;
		}

		setErrorFlag(XDriveError::notCalibrated, !calibrated);

		if(saveCalPending){
			saveCalPending = false;
			saveFlash();
		}
		if(saveCenterPending){
			saveCenterPending = false;
			Flash_Write(ADR_XDRIVE_CENTER, (uint16_t)centerFromIndex);
		}

		// DRV8301 status for diagnostics (on fault immediately, otherwise every second)
		if((errors & XDriveError::drvFault) || HAL_GetTick() - lastStatusRead > 1000){
			lastStatusRead = HAL_GetTick();
			drvStatus1 = drvRead(DRV_STATUS1);
			drvStatus2 = drvRead(DRV_STATUS2);
		}
	}
}

// ---- MotorDriver ---------------------------------------------------------------------
void MotorXDrive::turn(int16_t power){
	float t = (float)power / 32767.0f * (ilimit_mA * 0.001f);
	iqRef = invert ? -t : t;
}

void MotorXDrive::stopMotor(){
	enabled = false;
	iqRef = 0;
	if(state == XDriveState::Running){
		bridgeDisable();
	}
}

void MotorXDrive::startMotor(){
	enabled = true; // thread aligns / starts running
}

void MotorXDrive::emergencyStop(bool reset){
	if(reset){
		estop = false;
	}else{
		estop = true;
		iqRef = 0;
		bridgeDisable();
		if(state == XDriveState::Calibrating || state == XDriveState::Align || state == XDriveState::Test){
			state = XDriveState::Idle;
		}
	}
}

bool MotorXDrive::motorReady(){
	return state == XDriveState::Running && aligned;
}

// ---- Encoder -------------------------------------------------------------------------
EncoderType MotorXDrive::getEncoderType(){
	return useIndex ? EncoderType::incrementalIndex : EncoderType::incremental;
}

uint32_t MotorXDrive::getCpr(){
	return cpr;
}

int32_t MotorXDrive::getPos(){
	return posSign() * (raw - centerRaw);
}

void MotorXDrive::setPos(int32_t pos){
	int32_t r = rawSnapshot();
	centerRaw = r - pos * posSign();
	centerSetByUser = true;
	if(indexSeen){
		int32_t off = centerRaw - indexRaw;
		int32_t c = (int32_t)cpr;
		off = ((off % c) + c + c / 2) % c - c / 2; // closest equivalent within one revolution
		centerFromIndex = (int16_t)off;
		saveCenterPending = true;
	}
}

void MotorXDrive::exti(uint16_t GPIO_Pin){
	if(GPIO_Pin != ENCODER_Z_Pin || indexSeen){
		return;
	}
	indexRaw = rawSnapshot();
	indexSeen = true;
	if(!useIndex){
		return;
	}
	int32_t c = (int32_t)cpr;
	if(centerSetByUser){
		// Center was set before the index was seen: remember it relative to the index now
		int32_t off = centerRaw - indexRaw;
		off = ((off % c) + c + c / 2) % c - c / 2;
		centerFromIndex = (int16_t)off;
		saveCenterPending = true;
	}else if(centerFromIndex != 0){
		// Restore the saved center. The wheel was powered up within +-180 deg of the
		// center, so pick the revolution closest to the power up position.
		int32_t cand = indexRaw + centerFromIndex;
		int32_t diff = centerRaw - cand;
		int32_t k = (diff >= 0 ? diff + c / 2 : diff - c / 2) / c;
		centerRaw = cand + k * c;
	}
}

// ---- commands ----------------------------------------------------------------------------
CommandStatus MotorXDrive::command(const ParsedCommand& cmd, std::vector<CommandReply>& replies){
	switch(static_cast<XDrive_commands>(cmd.cmdId)){
	case XDrive_commands::poles:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(poles);
		}else if(cmd.type == CMDtype::set && cmd.val >= 1 && cmd.val <= 64){
			poles = cmd.val;
			calibrated = false; // calibration is only valid for the pole count it was made with
			aligned = false;
			if(state == XDriveState::Running || state == XDriveState::Test) state = XDriveState::Idle;
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::cpr:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(cpr);
		}else if(cmd.type == CMDtype::set && cmd.val >= 64 && cmd.val <= 65535){
			cpr = cmd.val;
			calibrated = false;
			aligned = false;
			if(state == XDriveState::Running || state == XDriveState::Test) state = XDriveState::Idle;
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::vlimit:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(vlimit_mV);
		}else if(cmd.type == CMDtype::set && cmd.val >= 500 && cmd.val <= 24000){
			vlimit_mV = cmd.val;
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::ilimit:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(ilimit_mA);
		}else if(cmd.type == CMDtype::set && cmd.val >= 500 && cmd.val <= 30000){
			ilimit_mA = cmd.val;
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::sensorDirection:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(sensorDirection);
		}else if(cmd.type == CMDtype::set && (cmd.val == 1 || cmd.val == -1)){
			if(cmd.val != sensorDirection){
				sensorDirection = cmd.val;
				aligned = false; // electrical offset depends on the direction: align again
				if(state == XDriveState::Running || state == XDriveState::Test) state = XDriveState::Idle;
			}
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::invert:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(invert ? 1 : 0);
		}else if(cmd.type == CMDtype::set){
			// keep the wheel where it is: position is mirrored around the current point
			int32_t p = getPos();
			invert = cmd.val != 0;
			setPos(p);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::offset:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(elecOffsetRaw);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::cal:
		if(cmd.type == CMDtype::get && state != XDriveState::Fault && state != XDriveState::Init){
			requestCal = true;
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::test:
		if(cmd.type == CMDtype::get && aligned && state != XDriveState::Fault){
			requestTest = true;
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::mode:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(mode);
		}else if(cmd.type == CMDtype::set && cmd.val >= 0 && cmd.val <= 1){
			intD = intQ = 0;
			mode = cmd.val;
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::index:
		return handleGetSet(cmd, replies, useIndex);

	case XDrive_commands::state:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((uint8_t)state);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::errors:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(errors);
		}else if(cmd.type == CMDtype::set && cmd.val == 0){
			setErrorFlag(0xffffffff, false);
			reportedErrors = 0;
			ErrorHandler::clearError(ErrorCode::systemError);
			ErrorHandler::clearError(ErrorCode::overvoltage);
			ErrorHandler::clearError(ErrorCode::brakeResistorFailure);
			ErrorHandler::clearError(ErrorCode::adcCalibrationError);
			ErrorHandler::clearError(ErrorCode::encoderAlignmentFailed);
			if(state == XDriveState::Fault){
				// thread reconfigures the (possibly latched) DRV8301 and measures offsets again
				bridgeDisable();
				gateReady = false;
				state = XDriveState::Init;
			}
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::vbus:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((int32_t)((float)ADC1_BUF[0] * VBUS_SCALE * 1000.0f)); // works without the ISR
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::iq:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((int32_t)(iqMeas * (invert ? -1000.0f : 1000.0f)));
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::res:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(res_100uOhm);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::ind:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(ind_uH);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::bw:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(bandwidth);
		}else if(cmd.type == CMDtype::set && cmd.val >= 100 && cmd.val <= 5000){
			bandwidth = cmd.val;
			computeGains();
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::drvstatus:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(((uint32_t)drvStatus1 << 16) | drvStatus2);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::brake:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((int32_t)(brakeDuty * 100.0f));
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::center:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(centerFromIndex);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::isrcount:
		if(cmd.type != CMDtype::get) return CommandStatus::ERR;
		replies.emplace_back(isrCount);
		break;

	case XDrive_commands::trigger:
		if(cmd.type != CMDtype::get) return CommandStatus::ERR;
		replies.emplace_back(softTrigger ? 1 : 0);
		break;

	case XDrive_commands::dbg:
		if(cmd.type != CMDtype::get) return CommandStatus::ERR;
		replies.emplace_back(TIM1->CR1);
		replies.emplace_back(TIM1->CNT);
		replies.emplace_back(TIM1->CCER);
		replies.emplace_back(TIM1->BDTR);
		replies.emplace_back(ADC2->CR2);
		replies.emplace_back(ADC2->SR);
		replies.emplace_back(ADC3->CR2);
		break;

	case XDrive_commands::calratio:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((int32_t)(calRatio * 1000.0f));
		}else{
			return CommandStatus::ERR;
		}
		break;

	case XDrive_commands::indexfound:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(indexSeen ? 1 : 0);
		}else{
			return CommandStatus::ERR;
		}
		break;

	default:
		return CommandStatus::NOT_FOUND;
	}
	return CommandStatus::OK;
}

#endif /* XDRIVEDRIVER */

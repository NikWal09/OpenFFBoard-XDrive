/*
 * Host simulation of MotorXDrive (the real driver source, compiled against stubs).
 *
 * Plant: surface PMSM hub motor (hoverboard) + wheel inertia, DRV8301 SPI registers,
 * low side current amplifiers with offset and noise, quadrature encoder with index.
 * The driver thread's Delay() advances simulated time, every 50 us the ADC "interrupt"
 * runs MotorXDrive::isr().
 *
 * Every hardware permutation is tested: motor phase order, encoder direction, current
 * amplifier sign and which ADC sees which phase. The driver must find all of them in
 * its calibration and then produce torque in the direction of increasing axis position.
 *
 * Build & run (Linux / WSL):  make -C Firmware/Targets/F405RG_XDRIVE/sim run
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <functional>
#include <random>
#include "MotorXDrive.h"

// --- registers / globals expected by the driver ------------------------------------
TIM_TypeDef sim_tim1, sim_tim2, sim_tim3;
ADC_TypeDef sim_adc1, sim_adc2, sim_adc3;
GPIO_TypeDef sim_gpiob, sim_gpioc, sim_gpiod;
volatile uint32_t ADC1_BUF[ADC1_CHANNELS];
SPI_HandleTypeDef hspi3;
ADC_HandleTypeDef hadc1{ADC1}, hadc2{ADC2}, hadc3{ADC3};
extern "C" { volatile uint8_t adc1Overrun = 0; }

// Command ids: order of MotorXDrive::XDrive_commands
enum Cmd : uint32_t { poles, cpr, vlimit, ilimit, sensorDirection, invert, offset, cal, test, mode, index_,
	state, errors, vbus, iq, res, ind, bw, drvstatus, brake, center, indexfound, calratio,
	isrcount, trigger, dbg, fwrev, vsupply };

static constexpr double PI2 = 6.283185307179586;
static constexpr int CPR = 4096;

// --- plant -----------------------------------------------------------------------------
struct Plant {
	// motor (per phase)
	double R = 0.20, L = 400e-6, psi = 0.0356; int p = 15;
	double J = 0.03, b = 0.003, coulomb = 0.08;
	double th0 = 0.7;       // electrical angle offset of the rotor magnets
	// state
	double thm = 0, wm = 0, id = 0, iq = 0;
	double Text = 0;        // external torque (hand)
	// hardware variant
	bool motorSwapBC = false; // motor phases B/C connected the other way round
	int encDir = 1;           // encoder counting direction
	int ampSign = 1;          // current amplifier polarity
	bool adcSwap = false;     // ADC2 sees phase C instead of B
	double indexAngle = 2.0;  // mechanical angle of the Z pulse
	double vbus = 19.5;
	double deadV = 0.23;      // bridge dead time voltage error (2*td*f*Vbus, td ~300ns)
	double iPrevBoard[3] = {0, 0, 0};
	bool brakeOverlap = false; // AUX_L and AUX_H on at the same time (must never happen)
	bool hwTrigger = true;     // TIM1 -> ADC injected hardware trigger works
	bool swTrigger = true;     // JSWSTART works
	// encoder counter (TIM3 restarts at 0 on MCU reset)
	double bootThm = 0;
	std::mt19937 rng{1234};
	int offB = 2048 + 23, offC = 2048 - 17; // amplifier offsets
} plant;

static uint64_t simTicks = 0;
static bool adcIrqOn = false;
static bool tim1UpIrqOn = false;
extern "C" void XDrive_Tim1Update(void);
static std::function<void()> onMs;           // scenario hook, every simulated ms
static std::map<uint16_t, uint16_t> flash;
static uint16_t drvRegs[4];
static uint16_t drvLastResp = 0;
static bool enGate = false;
static int errorsReported = 0;
struct SimDone {};

uint32_t HAL_GetTick(){ return (uint32_t)(simTicks / 20); }
void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState s){
	if(port == DRV_EN_GATE_GPIO_Port && pin == DRV_EN_GATE_Pin){
		enGate = s == GPIO_PIN_SET;
		if(!enGate){ drvRegs[2] = drvRegs[3] = 0; } // DRV8301 resets its registers
	}
}
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, uint8_t* tx, uint8_t* rx, uint16_t, uint32_t){
	uint16_t w = *(uint16_t*)tx;
	*(uint16_t*)rx = enGate ? drvLastResp : 0;
	if(!enGate) return 0;
	uint8_t addr = (w >> 11) & 0xf;
	if(addr > 3) addr = 0;
	if(w & 0x8000){
		drvLastResp = (addr << 11) | drvRegs[addr];
	}else{
		if(addr >= 2) drvRegs[addr] = w & 0x7ff;
		drvLastResp = (addr << 11) | drvRegs[addr];
	}
	return 0;
}
int HAL_ADCEx_InjectedStart(ADC_HandleTypeDef*){ return 0; }
int HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef*){ return 0; }
int HAL_ADC_Stop_DMA(ADC_HandleTypeDef*){ return 0; }
int HAL_ADC_Start_DMA(ADC_HandleTypeDef*, uint32_t*, uint32_t){ return 0; }
void HAL_NVIC_SetPriority(IRQn_Type, uint32_t, uint32_t){}
void HAL_NVIC_EnableIRQ(IRQn_Type irq){ if(irq == ADC_IRQn) adcIrqOn = true; if(irq == TIM1_UP_TIM10_IRQn) tim1UpIrqOn = true; }
void HAL_NVIC_DisableIRQ(IRQn_Type irq){ if(irq == ADC_IRQn) adcIrqOn = false; }
bool Flash_Write(uint16_t adr, uint16_t dat){ flash[adr] = dat; return true; }
bool Flash_Read(uint16_t adr, uint16_t* buf){
	auto it = flash.find(adr);
	if(it == flash.end()) return false;
	*buf = it->second; return true;
}
void ErrorHandler::addError(const Error& e){ errorsReported++; printf("      [error] %s\n", e.info.c_str()); }

// One PWM period of the plant, then the "ADC interrupt"
static void plantTick(){
	Plant& P = plant;
	const double dt = 50e-6;
	const int sub = 5;
	const double h = dt / sub;
	bool bridge = TIM1->BDTR & TIM_BDTR_MOE;
	double d[3] = { TIM1->CCR1 / 4200.0, TIM1->CCR2 / 4200.0, TIM1->CCR3 / 4200.0 };
	double vk[3];
	for(int k = 0; k < 3; k++){
		double sgn = P.iPrevBoard[k] > 0.05 ? 1 : (P.iPrevBoard[k] < -0.05 ? -1 : P.iPrevBoard[k] / 0.05);
		vk[k] = P.vbus * d[k] - P.deadV * sgn;
	}
	double avg = (vk[0] + vk[1] + vk[2]) / 3.0;
	double vBoard[3]; for(int k = 0; k < 3; k++) vBoard[k] = vk[k] - avg;
	// brake half bridge: center aligned TIM2, L on while CNT < CCR3, H on while CNT >= CCR4
	if(TIM2->CCR3 > 0 && TIM2->CCR4 <= TIM2->ARR && TIM2->CCR3 > TIM2->CCR4) P.brakeOverlap = true;
	// board phase k drives motor phase map[k]
	int map[3] = {0, P.motorSwapBC ? 2 : 1, P.motorSwapBC ? 1 : 2};
	double vm[3]; for(int k = 0; k < 3; k++) vm[map[k]] = vBoard[k];
	double va = (2.0/3.0) * (vm[0] - 0.5*vm[1] - 0.5*vm[2]);
	double vb = (2.0/3.0) * (0.8660254 * (vm[1] - vm[2]));

	double indexPrev = std::floor((P.thm - P.indexAngle) / PI2);
	for(int s = 0; s < sub; s++){
		double the = P.p * P.thm + P.th0;
		double c = std::cos(the), sn = std::sin(the);
		double we = P.p * P.wm;
		if(bridge){
			double vd = va * c + vb * sn, vq = -va * sn + vb * c;
			double did = (vd - P.R * P.id + we * P.L * P.iq) / P.L;
			double diq = (vq - P.R * P.iq - we * P.L * P.id - we * P.psi) / P.L;
			P.id += did * h; P.iq += diq * h;
		}else{
			P.id = P.iq = 0; // FETs off (diode conduction ignored)
		}
		double T = 1.5 * P.p * P.psi * P.iq;
		double Tf = -P.b * P.wm - P.coulomb * std::tanh(P.wm / 0.05);
		P.wm += (T + Tf + P.Text) / P.J * h;
		P.thm += P.wm * h;
	}
	simTicks++;

	// encoder
	double cntD = P.encDir * (P.thm - P.bootThm) / PI2 * CPR;
	TIM3->CNT = (uint32_t)((int32_t)std::lround(cntD)) & 0xffff;
	// currents of the motor phases -> board phases
	double the = P.p * P.thm + P.th0;
	double ia = P.id * std::cos(the) - P.iq * std::sin(the);
	double ib = P.id * std::sin(the) + P.iq * std::cos(the);
	double im[3] = { ia, -0.5*ia + 0.8660254*ib, -0.5*ia - 0.8660254*ib };
	double iBoard[3]; for(int k = 0; k < 3; k++){ iBoard[k] = im[map[k]]; P.iPrevBoard[k] = iBoard[k]; }
	std::normal_distribution<double> noise(0, 1.5);
	auto adc = [&](double i, int off){
		double v = off + P.ampSign * i * 0.0005 * 40.0 / 3.3 * 4096.0 + noise(P.rng);
		return (uint32_t)std::min(4095.0, std::max(0.0, std::round(v)));
	};
	uint32_t adcPhB = adc(iBoard[1], P.offB), adcPhC = adc(iBoard[2], P.offC);
	ADC2->JDR1 = P.adcSwap ? adcPhC : adcPhB;
	ADC3->JDR1 = P.adcSwap ? adcPhB : adcPhC;
	ADC1_BUF[0] = (uint32_t)std::lround(P.vbus / 19.0 / 3.3 * 4096.0);
	// conversion trigger: hardware (TRGO) or the software fallback from the TIM1 update IRQ
	bool converted = P.hwTrigger && (ADC2->CR2 & ADC_CR2_JEXTEN || true) && !(MotorXDrive::instance && MotorXDrive::instance->softTrigger);
	if(MotorXDrive::instance && MotorXDrive::instance->softTrigger && tim1UpIrqOn && (TIM1->DIER & TIM_DIER_UIE)){
		TIM1->CR1 |= TIM_CR1_DIR; // update after the peak: counting down
		XDrive_Tim1Update();
		converted = P.swTrigger && (ADC2->CR2 & ADC_CR2_JSWSTART);
		ADC2->CR2 &= ~ADC_CR2_JSWSTART; ADC3->CR2 &= ~ADC_CR2_JSWSTART;
	}
	if(converted){
		ADC2->SR |= ADC_SR_JEOC;
		if(adcIrqOn && MotorXDrive::instance) MotorXDrive::instance->isr();
	}
	double indexNow = std::floor((P.thm - P.indexAngle) / PI2);
	if(indexNow != indexPrev && MotorXDrive::instance) MotorXDrive::instance->exti(ENCODER_Z_Pin);
}

void cpp_freertos::Thread::Delay(uint32_t ms){
	for(uint32_t m = 0; m < ms; m++){
		for(int t = 0; t < 20; t++) plantTick();
		if(onMs) onMs();
	}
}

// --- helpers ----------------------------------------------------------------------------
static MotorXDrive* drv = nullptr;
static int64_t get(uint32_t id){
	std::vector<CommandReply> r; ParsedCommand c; c.cmdId = id; c.type = CMDtype::get;
	drv->command(c, r);
	return r.empty() ? 0 : r[0].val;
}
static bool exec(uint32_t id){
	std::vector<CommandReply> r; ParsedCommand c; c.cmdId = id; c.type = CMDtype::get;
	return drv->command(c, r) == CommandStatus::OK;
}
static double simTime(){ return simTicks / 20000.0; }

static int failures = 0;
static void check(bool ok, const char* what, double a = 0, double b = 0){
	if(!ok){ failures++; printf("    FAIL %s (%.3f / %.3f)\n", what, a, b); }
}

/*
 * Runs the driver thread until `script` (called every ms with the elapsed time of this run)
 * returns true or the time limit is hit.
 */
static void runDriver(std::function<bool(double)> script, double limit){
	double t0 = simTime();
	onMs = [&](){
		double t = simTime() - t0;
		if(script(t) || t > limit) throw SimDone{};
	};
	try{ drv->Run(); }catch(SimDone&){}
	onMs = nullptr;
}

static void powerUp(double thm){
	// MCU reset: timers restart, motor state keeps (mechanics), currents decay
	plant.thm = thm; plant.wm = 0; plant.id = plant.iq = 0; plant.Text = 0;
	plant.bootThm = thm;
	sim_tim1 = {}; sim_tim2 = {}; sim_tim3 = {};
	sim_gpiod.IDR = DRV_NFAULT_Pin; // nFAULT high = ok
	sim_tim2.ARR = 2100;
	adcIrqOn = false; tim1UpIrqOn = false; enGate = false;
	drv = new MotorXDrive();
	drv->startMotor(); // the axis does this right after creating the driver
}

static void powerDown(){
	delete drv; drv = nullptr;
}

// --- scenario per hardware variant -------------------------------------------------------
static void scenario(bool motorSwap, int encDir, int ampSign, bool adcSwap, uint32_t seed){
	plant = Plant{};
	plant.motorSwapBC = motorSwap; plant.encDir = encDir; plant.ampSign = ampSign; plant.adcSwap = adcSwap;
	plant.rng.seed(seed);
	std::uniform_real_distribution<double> u(0, PI2);
	plant.th0 = u(plant.rng);
	plant.indexAngle = u(plant.rng);
	flash.clear();
	errorsReported = 0;
	printf("  motorSwap=%d encDir=%+d ampSign=%+d adcSwap=%d\n", motorSwap, encDir, ampSign, adcSwap);

	// ---- first power up: not calibrated -> calibration -----------------------------
	powerUp(u(plant.rng) * 0.1);
	runDriver([](double t){ return t > 1.0; }, 2);
	check(get(state) == 2, "idle after power up", get(state), 2);
	check(get(errors) & (1 << 9), "notCalibrated flag");

	check(exec(cal), "cal accepted");
	runDriver([](double t){ int64_t s = get(state); return t > 0.1 && (s == 5 || s == 7); }, 25);
	check(get(state) == 5, "running after calibration", get(state), 5);
	if(get(state) != 5){ printf("    errors=0x%llx\n", (unsigned long long)get(errors)); powerDown(); return; }
	double r = get(res) * 1e-4, l = get(ind) * 1e-6;
	if(getenv("SIMDEBUG")) printf("      R=%.4f L=%.1fuH th0=%.2f\n", r, l*1e6, plant.th0);
	check(std::fabs(r - plant.R) / plant.R < 0.1, "resistance", r, plant.R);
	check(std::fabs(l - plant.L) / plant.L < 0.15, "inductance", l * 1e6, plant.L * 1e6);

	// ---- positive torque must increase the axis position ---------------------------
	int32_t p0 = drv->getPos();
	drv->turn(6000);
	runDriver([](double t){ return t > 0.3; }, 1);
	drv->turn(0);
	int32_t p1 = drv->getPos();
	check(p1 - p0 > 150, "positive torque -> position up", p1 - p0, 150);
	double iqm = get(iq) / 1000.0;

	// ---- axis spring (as OpenFFBoard does): must settle, not run away -------------------
	static int32_t lastPos = 0;
	lastPos = drv->getPos();
	runDriver([](double t){
		int32_t pos = drv->getPos();
		int32_t vel = pos - lastPos; // counts per ms
		lastPos = pos;
		int32_t tq = -40 * pos - 400 * vel; // spring + damper, like axis idlespring/axisdamper
		drv->turn((int16_t)std::max(-32767, std::min(32767, tq)));
		return t > 3.0;
	}, 4);
	drv->turn(0);
	check(std::abs(drv->getPos()) < 60, "spring settles at 0", drv->getPos(), 60);
	check(get(state) == 5, "still running after spring");

	// ---- self test ---------------------------------------------------------------------
	check(exec(test), "test accepted");
	double maxPos = 0, minPos = 0;
	runDriver([&](double t){
		double deg = (plant.thm - plant.bootThm) * 57.2958;
		maxPos = std::max(maxPos, deg); minPos = std::min(minPos, deg);
		return t > 0.2 && get(state) != 6;
	}, 20);
	check(get(state) == 5, "running after self test", get(state), 5);
	check(std::fabs(maxPos - minPos) > 650, "self test moved +-360 deg", maxPos, minPos);

	// ---- set the center away from power up, after the index was seen -----------------
	plant.Text = 0.6; // hand turns the wheel
	runDriver([](double t){ return t > 0.8; }, 1);
	plant.Text = -0.6;
	runDriver([](double t){ return t > 0.8; }, 1);
	plant.Text = 0;
	runDriver([](double t){ return t > 0.2 && std::fabs(plant.wm) < 0.02; }, 6); // wheel stops
	check(get(indexfound) == 1, "index found while turning");
	drv->setPos(0); // axis.zeroenc
	double thCenter = plant.thm;
	runDriver([](double t){ return t > 0.1; }, 1); // thread stores the center
	int32_t q0 = drv->getPos();
	double thq0 = plant.thm;
	plant.Text = 0.4;
	runDriver([](double t){ return t > 0.3; }, 1);
	plant.Text = 0;
	double slope = (drv->getPos() - q0) / ((plant.thm - thq0) * CPR / PI2); // +1 or -1
	check(std::fabs(std::fabs(slope) - 1) < 0.05, "position slope 1 count per count", slope, 1);
	slope = slope > 0 ? 1 : -1;
	check(std::fabs(q0 - slope * (thq0 - thCenter) * CPR / PI2) < 5, "zero at the center", q0, 0);
	drv->saveFlash(); // sys.save
	powerDown();

	// ---- second power up: 70 deg away from the center ------------------------------------
	powerUp(thCenter + 1.2);
	runDriver([](double t){ return t > 0.2 && get(state) == 5; }, 5);
	check(get(state) == 5, "running after power up alignment", get(state), 5);
	p0 = drv->getPos();
	drv->turn(6000);
	runDriver([](double t){ return t > 0.3; }, 1);
	drv->turn(0);
	check(drv->getPos() - p0 > 150, "torque direction after reboot", drv->getPos() - p0, 150);
	// turn by hand until the index passes, then the saved center must be restored
	plant.Text = -0.8;
	runDriver([](double t){ return get(indexfound) == 1 && t > 0.05; }, 4);
	plant.Text = 0;
	runDriver([](double t){ return t > 0.3; }, 1);
	double expected = slope * (plant.thm - thCenter) * CPR / PI2;
	check(std::fabs(drv->getPos() - expected) < 25, "center restored from index", drv->getPos(), expected);
	check(errorsReported == 0, "no errors reported", errorsReported, 0);
	powerDown();
}


// --- additional situations ------------------------------------------------------------
static void setCmd(uint32_t id, int64_t val){
	std::vector<CommandReply> r; ParsedCommand c; c.cmdId = id; c.type = CMDtype::set; c.val = val;
	drv->command(c, r);
}

static void calibrateFresh(){
	powerUp(0.3);
	runDriver([](double t){ return t > 1.0; }, 2);
	exec(cal);
	runDriver([](double t){ int64_t s = get(state); return t > 0.1 && (s == 5 || s == 7); }, 25);
}

static void testMotor(double R, double L, const char* name){
	printf("  motor %s: R=%.2f L=%.0fuH\n", name, R, L * 1e6);
	plant = Plant{}; plant.R = R; plant.L = L; flash.clear(); errorsReported = 0;
	calibrateFresh();
	check(get(state) == 5, "running after calibration", get(state), 5);
	double r = get(res) * 1e-4, l = get(ind) * 1e-6;
	check(std::fabs(r - R) / R < 0.1, "resistance", r, R);
	check(std::fabs(l - L) / L < 0.15, "inductance", l * 1e6, L * 1e6);
	int32_t p0 = drv->getPos();
	drv->turn(6000);
	runDriver([](double t){ return t > 0.3; }, 1);
	drv->turn(0);
	check(drv->getPos() - p0 > 150, "torque direction", drv->getPos() - p0, 150);
	// torque step: the measured current must follow within a few ms
	runDriver([](double t){ return t > 1.5; }, 2); // let the wheel coast down
	drv->turn(8000);
	double target = 8000.0 / 32767.0 * 10.0;
	runDriver([](double t){
		if(getenv("SIMDEBUG")) printf("      t=%.4f iq=%.3f id=%.3f wm=%.2f\n", t, plant.iq, plant.id, plant.wm);
		return t > 0.004; }, 1);
	check(std::fabs(plant.iq - target) < 0.15 * target, "current step settles in 4 ms", plant.iq, target);
	drv->turn(0);
	powerDown();
}

static void testWrongPoles(){
	printf("  wrong pole count (14 configured, motor has 15)\n");
	plant = Plant{}; flash.clear(); errorsReported = 0;
	powerUp(0.3);
	runDriver([](double t){ return t > 1.0; }, 2);
	setCmd(poles, 14);
	exec(cal);
	runDriver([](double t){ int64_t s = get(state); return t > 0.1 && (s == 5 || s == 7); }, 25);
	check(get(state) == 7, "calibration rejected", get(state), 7);
	check(get(errors) & XDriveError::calCprPoles, "calCprPoles flag", get(calratio), 0);
	powerDown();
}

static void testPowerSequence(){
	printf("  supply off at power up, then on, then lost while running\n");
	plant = Plant{}; flash.clear(); errorsReported = 0;
	calibrateFresh();
	drv->saveFlash();
	powerDown();
	plant.vbus = 0; // only USB powers the MCU
	powerUp(0.5);
	runDriver([](double t){ return t > 2.0; }, 3);
	check(get(state) == 0, "waits in Init without supply", get(state), 0);
	check(!(get(errors) & XDriveError::latched), "no latched error without supply", get(errors), 0);
	check(!(TIM1->BDTR & TIM_BDTR_MOE), "bridge off without supply");
	plant.vbus = 19.5;
	runDriver([](double t){ return t > 0.2 && get(state) == 5; }, 5);
	check(get(state) == 5, "running after supply on", get(state), 5);
	plant.vbus = 6;
	runDriver([](double t){ return t > 0.5; }, 1);
	check(!(TIM1->BDTR & TIM_BDTR_MOE), "bridge off after supply loss");
	drvRegs[2] = drvRegs[3] = 0; // DRV8301 lost its configuration
	plant.vbus = 19.5;
	runDriver([](double t){ return t > 0.2 && get(state) == 5; }, 5);
	check(get(state) == 5, "running again after supply returns", get(state), 5);
	check(drvRegs[3] == (2 << 2), "DRV8301 gain reconfigured", drvRegs[3], 8);
	int32_t p0 = drv->getPos();
	drv->turn(6000);
	runDriver([](double t){ return t > 0.3; }, 1);
	drv->turn(0);
	check(drv->getPos() - p0 > 150, "torque direction after supply loss", drv->getPos() - p0, 150);
	powerDown();
}

static void testBrakeAndFaults(){
	printf("  brake resistor, nFAULT, recovery\n");
	plant = Plant{}; flash.clear(); errorsReported = 0;
	calibrateFresh();
	plant.brakeOverlap = false;
	plant.vbus = 21.6;
	runDriver([](double t){ return t > 0.05; }, 1);
	check(get(brake) > 0, "brake on above 21 V", get(brake), 0);
	plant.vbus = 20.0;
	runDriver([](double t){ return t > 0.05; }, 1);
	check(get(brake) == 0, "brake off below 20.2 V", get(brake), 0);
	check(!plant.brakeOverlap, "AUX_L and AUX_H never on together");
	plant.vbus = 21.6;
	runDriver([](double t){ return t > 5.5; }, 6);
	check(get(errors) & XDriveError::brakeTimeout, "brake timeout after 5 s", get(errors), 0);
	check(get(brake) == 0, "brake off after timeout", get(brake), 0);
	check(get(state) == 7, "fault state", get(state), 7);
	check(!plant.brakeOverlap, "AUX_L and AUX_H never on together (long)");
	plant.vbus = 19.5;
	setCmd(errors, 0);
	runDriver([](double t){ return t > 0.2 && get(state) == 5; }, 5);
	check(get(state) == 5, "running after clearing errors", get(state), 5);

	sim_gpiod.IDR = 0; // DRV8301 nFAULT low
	runDriver([](double t){ return t > 0.05; }, 1);
	check(get(state) == 7 && (get(errors) & XDriveError::drvFault), "nFAULT -> fault", get(state), 7);
	check(!(TIM1->BDTR & TIM_BDTR_MOE), "bridge off on nFAULT");
	sim_gpiod.IDR = DRV_NFAULT_Pin;
	setCmd(errors, 0);
	runDriver([](double t){ return t > 0.2 && get(state) == 5; }, 5);
	check(get(state) == 5, "running after nFAULT cleared", get(state), 5);
	powerDown();
}

static void testSupply24V(){
	printf("  24 V supply: brake and overvoltage follow the measured supply, same pole count keeps calibration\n");
	plant = Plant{}; plant.vbus = 24.0; flash.clear(); errorsReported = 0;
	calibrateFresh();
	check(get(state) == 5, "calibrated at 24 V", get(state), 5);
	check(std::llabs(get(vsupply) - 24000) < 300, "supply measured", get(vsupply), 24000);
	runDriver([](double t){ return t > 6.0; }, 7);
	check(get(brake) == 0, "brake stays off at 24 V", get(brake), 0);
	check(!(get(errors) & XDriveError::latched), "no error at 24 V after 6 s", get(errors), 0);
	setCmd(poles, 15); // the app may write back the unchanged value
	setCmd(cpr, 4096);
	runDriver([](double t){ return t > 0.1; }, 1);
	check(!(get(errors) & XDriveError::notCalibrated) && get(state) == 5, "same poles/cpr keep calibration", get(errors), 0);
	plant.vbus = 25.8;
	runDriver([](double t){ return t > 0.05; }, 1);
	check(get(brake) > 0, "brake on above supply + 1.5 V", get(brake), 0);
	plant.vbus = 24.5;
	runDriver([](double t){ return t > 0.05; }, 1);
	check(get(brake) == 0, "brake off below supply + 0.8 V", get(brake), 0);
	plant.vbus = 28.5;
	runDriver([](double t){ return t > 0.05; }, 1);
	check(get(state) == 7 && (get(errors) & XDriveError::overvoltage), "overvoltage above supply + 4 V", get(errors), 8);
	powerDown();

	plant = Plant{}; plant.vbus = 36.0; flash.clear(); errorsReported = 0;
	powerUp(0.3);
	runDriver([](double t){ return t > 1.0; }, 2);
	check(get(state) == 7 && (get(errors) & XDriveError::overvoltage), "unsupported 36 V supply rejected", get(errors), 8);
	check(!(TIM1->BDTR & TIM_BDTR_MOE), "bridge off with 36 V supply");
	powerDown();
}

static void testAdcTrigger(){
	printf("  hardware ADC trigger dead -> software fallback, then both dead -> error\n");
	plant = Plant{}; plant.hwTrigger = false; flash.clear(); errorsReported = 0;
	powerUp(0.3);
	runDriver([](double t){ return t > 3.0; }, 4);
	check(get(trigger) == 1, "switched to software trigger", get(trigger), 1);
	check(get(state) == 2, "idle after fallback", get(state), 2);
	check(get(isrcount) > 20000, "current loop running", get(isrcount), 20000);
	exec(cal);
	runDriver([](double t){ int64_t s = get(state); return t > 0.1 && (s == 5 || s == 7); }, 25);
	check(get(state) == 5, "calibration with software trigger", get(state), 5);
	int32_t p0 = drv->getPos();
	drv->turn(6000);
	runDriver([](double t){ return t > 0.3; }, 1);
	drv->turn(0);
	check(drv->getPos() - p0 > 150, "torque with software trigger", drv->getPos() - p0, 150);
	powerDown();

	plant = Plant{}; plant.hwTrigger = false; plant.swTrigger = false; flash.clear(); errorsReported = 0;
	powerUp(0.3);
	runDriver([](double t){ return t > 4.0; }, 5);
	check(get(state) == 7 && (get(errors) & XDriveError::noCurrentIsr), "no interrupts -> error 4096", get(errors), 4096);
	powerDown();
}

int main(){
	printf("MotorXDrive host simulation\n");
	uint32_t seed = 1;
	for(int ms = 0; ms < 2; ms++)
	for(int ed = -1; ed <= 1; ed += 2)
	for(int as = -1; as <= 1; as += 2)
	for(int ad = 0; ad < 2; ad++){
		scenario(ms, ed, as, ad, seed++);
	}
	testMotor(0.10, 200e-6, "low R/L");
	testMotor(0.45, 1100e-6, "high R/L");
	testWrongPoles();
	testPowerSequence();
	testBrakeAndFaults();
	testAdcTrigger();
	testSupply24V();
	printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
	return failures ? 1 : 0;
}

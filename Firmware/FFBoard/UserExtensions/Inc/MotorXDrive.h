/*
 * MotorXDrive.h
 *
 * Onboard FOC motor driver for the MKS XDrive Mini V1.0 (STM32F405 + DRV8301).
 * Single BLDC motor (hoverboard hub motor) with an MT6701 incremental ABZ encoder.
 *
 * - Current FOC at 20 kHz (ADC2/ADC3 injected, low side shunts via DRV8301 amplifiers),
 *   voltage mode as fallback (xdrive.mode=0).
 * - Calibration (xdrive.cal): ADC offsets, phase resistance/inductance, current sign,
 *   phase B/C order, encoder direction and electrical offset. Stored in flash.
 *   On every power up only a short alignment (a few degrees of wheel motion) is done.
 * - Wheel center relative to the encoder index (Z): after the first index pulse the
 *   center saved with axis.zeroenc is restored automatically.
 * - Brake resistor on the AUX half bridge, driven from the bus voltage.
 * - Self test (xdrive.test): +360 deg, -720 deg, +360 deg with a PD position loop.
 *
 * The class is also the encoder of the axis (hasIntegratedEncoder).
 */

#ifndef USEREXTENSIONS_INC_MOTORXDRIVE_H_
#define USEREXTENSIONS_INC_MOTORXDRIVE_H_

#include "target_constants.h"
#ifdef XDRIVEDRIVER

#include "MotorDriver.h"
#include "Encoder.h"
#include "CommandHandler.h"
#include "PersistentStorage.h"
#include "ExtiHandler.h"
#include "thread.hpp"
#include "ErrorHandler.h"

#define XDRIVE_THREAD_MEM 384
#define XDRIVE_THREAD_PRIO 25
#define XDRIVE_FW_REV 2026102302 // xdrive.fwrev: YYYYMMDDnn of the last behaviour change, lets the app show what is flashed

enum class XDriveState : uint8_t {
	Init = 0,       // waiting for the gate driver setup
	AdcCal = 1,     // measuring current sensor offsets, bridge off
	Idle = 2,       // bridge off (free wheel)
	Align = 3,      // power up alignment of the electrical angle
	Calibrating = 4,// full calibration (xdrive.cal)
	Running = 5,    // torque control
	Test = 6,       // self test movement (xdrive.test)
	Fault = 7       // latched fault, bridge off. Clear with xdrive.errors=0
};

enum class XDriveCalStep : uint8_t {
	RampLow, HoldLow, RampHigh, HoldHigh, Inductance, Sign90, SweepFwd, SweepBack, Settle, Done
};

// Bit flags reported by xdrive.errors
namespace XDriveError {
	constexpr uint32_t drvFault        = 1 << 0;  // DRV8301 nFAULT (overcurrent, undervoltage, overtemp)
	constexpr uint32_t drvSpi          = 1 << 1;  // DRV8301 configuration readback mismatch
	constexpr uint32_t adcOffset       = 1 << 2;  // current sensor offsets implausible
	constexpr uint32_t overvoltage     = 1 << 3;
	constexpr uint32_t undervoltage    = 1 << 4;  // not latched
	constexpr uint32_t brakeTimeout    = 1 << 5;  // brake resistor on for too long
	constexpr uint32_t calNoCurrent    = 1 << 6;  // calibration could not reach current (motor not connected?)
	constexpr uint32_t calCprPoles     = 1 << 7;  // encoder movement does not match poles/cpr
	constexpr uint32_t calResistance   = 1 << 8;  // resistance out of range
	constexpr uint32_t notCalibrated   = 1 << 9;  // not latched
	constexpr uint32_t overcurrent     = 1 << 10; // software current limit exceeded
	constexpr uint32_t testAborted     = 1 << 11; // self test lost the target position
	constexpr uint32_t noCurrentIsr    = 1 << 12; // no current sampling interrupts (ADC trigger)
	constexpr uint32_t latched = drvFault | drvSpi | adcOffset | overvoltage | brakeTimeout
			| calNoCurrent | calCprPoles | calResistance | overcurrent | testAborted | noCurrentIsr;
}

class MotorXDrive : public MotorDriver, public Encoder, public PersistentStorage, public CommandHandler,
		public ExtiHandler, public cpp_freertos::Thread {
	enum class XDrive_commands : uint32_t {
		poles, cpr, vlimit, ilimit, sensorDirection, invert, offset, cal, test, mode, index,
		state, errors, vbus, iq, res, ind, bw, drvstatus, brake, center, indexfound, calratio,
		isrcount, trigger, dbg, fwrev, vsupply
	};
public:
	MotorXDrive();
	virtual ~MotorXDrive();

	static ClassIdentifier info;
	const ClassIdentifier getInfo() override;
	static bool isCreatable() { return !inUse; }
	static bool inUse;

	// MotorDriver
	void turn(int16_t power) override;
	void stopMotor() override;
	void startMotor() override;
	void emergencyStop(bool reset = false) override;
	bool motorReady() override;
	Encoder* getEncoder() override { return static_cast<Encoder*>(this); }
	bool hasIntegratedEncoder() override { return true; }

	// Encoder
	EncoderType getEncoderType() override;
	int32_t getPos() override;
	void setPos(int32_t pos) override;
	uint32_t getCpr() override;

	// Persistent storage
	void saveFlash() override;
	void restoreFlash() override;

	// Commands
	void registerCommands();
	CommandStatus command(const ParsedCommand& cmd, std::vector<CommandReply>& replies) override;
	std::string getHelpstring() override { return "MKS XDrive Mini onboard FOC driver\n"; }

	void exti(uint16_t GPIO_Pin) override;
	void Run() override;

	void isr(); // 20 kHz current loop, called from the ADC interrupt
	void enableSoftTrigger(); // fallback: start the current conversions from the TIM1 update interrupt
	volatile uint32_t isrCount = 0;
	volatile bool softTrigger = false;
	static MotorXDrive* instance;

private:
	// --- hardware -----------------------------------------------------------------
	void setupGateDriver();
	uint16_t drvTransfer(uint16_t word);
	uint16_t drvRead(uint8_t addr);
	void drvWrite(uint8_t addr, uint16_t data);
	void startPwmAndAdc();
	void bridgeEnable();
	void bridgeDisable();
	void applyVoltageAB(float valpha, float vbeta, bool compensate = false);
	void applyVoltageDQ(float vd, float vq, float theta);
	void updateBrake();

	// --- control ------------------------------------------------------------------
	float electricalAngle(int32_t rawpos);
	void runCurrentLoop(float iqRef, float theta, float ialpha, float ibeta);
	void calibrationStep(float ialpha, float ibeta);
	void testStep(float ialpha, float ibeta, float theta);
	void startAlign(bool full);
	void enterFault(uint32_t err);
	void setErrorFlag(uint32_t flags, bool set); // thread side, atomic against the ISR
	void startAdcCal();
	void updateDeadComp(float verrAlpha);
	void reportErrors();
	void computeGains();
	int32_t posSign() const { return (sensorDirection * (invert ? -1 : 1)); }
	int32_t rawSnapshot();
	static float fastSin(float x);
	static float fastCos(float x) { return fastSin(x + 1.5707963f); }

	// --- configuration (flash) --------------------------------------------------
	uint16_t poles = 15;
	uint16_t cpr = 4096;
	uint16_t vlimit_mV = 8000;
	uint16_t ilimit_mA = 10000;
	int8_t sensorDirection = 1;
	bool invert = false;
	bool calibrated = false;
	bool useIndex = true;
	int8_t currentSign = 1;
	bool swapBC = false;
	uint8_t mode = 1; // 0 = voltage, 1 = current FOC
	uint16_t res_100uOhm = 0; // phase resistance in 0.1 mOhm
	uint16_t ind_uH = 0;      // phase inductance in uH
	uint16_t bandwidth = 1000; // current loop bandwidth rad/s
	int16_t centerFromIndex = 0;

	// --- runtime ------------------------------------------------------------------
	volatile XDriveState state = XDriveState::Init;
	volatile uint32_t errors = 0;
	volatile uint32_t reportedErrors = 0;
	volatile bool enabled = false;
	volatile bool gateReady = false; // DRV8301 configured for the current power cycle
	uint32_t adcCalWatch = 0, adcCalIsrStart = 0; // watchdog for the current sampling interrupt
	volatile bool estop = false;
	volatile bool aligned = false;
	volatile bool requestCal = false;
	volatile bool requestTest = false;
	volatile bool saveCalPending = false;
	volatile bool saveCenterPending = false;
	volatile float iqRef = 0;   // A
	volatile int32_t raw = 0;   // extended encoder count
	uint16_t lastCnt = 0;
	volatile int32_t centerRaw = 0;
	volatile int32_t indexRaw = 0;
	volatile bool indexSeen = false;
	bool centerSetByUser = false;
	int32_t elecOffsetRaw = 0;
	volatile float vbus = 0;    // V
	volatile float velocity = 0; // counts/s, filtered
	volatile float iqMeas = 0;  // A, filtered
	volatile float brakeDuty = 0;
	volatile float vSupply = 0; // supply voltage measured when it came on; brake and overvoltage thresholds follow it
	bool braking = false;
	uint32_t brakeOnTicks = 0;
	uint16_t drvStatus1 = 0, drvStatus2 = 0;

	// current sensing
	int32_t adcOffsetB = 2048, adcOffsetC = 2048;
	int32_t adcAccB = 0, adcAccC = 0;
	uint32_t adcCount = 0;

	// current controller
	float kp = 0, ki = 0;
	float deadComp = 0; // per phase dead time voltage compensation (V), measured at alignment
	float lastIa = 0, lastIb = 0, lastIc = 0;
	float intD = 0, intQ = 0;

	// calibration / alignment state machine
	XDriveCalStep calStep = XDriveCalStep::RampLow;
	bool fullCal = false;
	uint32_t stepTicks = 0;
	float calV = 0, calVlow = 0, calIlow = 0, calVhigh = 0, calIhigh = 0;
	float calTheta = 0;
	float calR = 0, calVerr = 0;
	float calRatio = 0; // measured / expected encoder movement of the last calibration
	float accA = 0, accB = 0;
	int64_t accRaw = 0;
	uint32_t accN = 0;
	float lPrevI = 0, lSum = 0, lDiSum = 0;
	int32_t sweepStartRaw = 0;

	// self test
	uint32_t testTicks = 0;
	int32_t testStartRaw = 0;
	float testKp = 0.12f;  // A/deg
	float testKd = 0.004f; // A/(deg/s)
};

#endif /* XDRIVEDRIVER */
#endif /* USEREXTENSIONS_INC_MOTORXDRIVE_H_ */

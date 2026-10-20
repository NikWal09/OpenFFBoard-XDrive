/*
 * Minimal host versions of the OpenFFBoard base classes used by MotorXDrive.
 * Only the interface matters; behaviour is provided by sim.cpp where needed.
 */
#ifndef OPENFFBOARD_STUBS_H
#define OPENFFBOARD_STUBS_H

#include <cstdint>
#include <string>
#include <vector>
#include "sim_hal.h"

#define CLSID_MOT_XDRIVE 0x8D

enum class ClassVisibility : uint8_t { visible, hidden, debug };
struct ClassIdentifier {
	const char* name;
	uint16_t id;
	ClassVisibility visibility = ClassVisibility::visible;
};

// ---- Encoder / MotorDriver ----
enum class EncoderType : uint8_t { NONE = 0, incremental = 1, incrementalIndex = 2, absolute = 3 };

class Encoder {
public:
	virtual ~Encoder() {}
	virtual const ClassIdentifier getInfo() = 0;
	virtual EncoderType getEncoderType() { return EncoderType::NONE; }
	virtual int32_t getPos() { return 0; }
	virtual void setPos(int32_t) {}
	virtual uint32_t getCpr() { return cpr; }
protected:
	uint32_t cpr = 0;
};

class MotorDriver {
public:
	virtual ~MotorDriver() {}
	virtual const ClassIdentifier getInfo() = 0;
	virtual void turn(int16_t) {}
	virtual void stopMotor() {}
	virtual void startMotor() {}
	virtual void emergencyStop(bool reset = false) {}
	virtual bool motorReady() { return true; }
	virtual Encoder* getEncoder() = 0;
	virtual bool hasIntegratedEncoder() { return false; }
};

class PersistentStorage {
public:
	virtual ~PersistentStorage() {}
	virtual void saveFlash() {}
	virtual void restoreFlash() {}
};

class ExtiHandler {
public:
	virtual ~ExtiHandler() {}
	virtual void exti(uint16_t) {}
};

// ---- Commands ----
enum class CommandStatus : uint8_t { NONE, OK, NOT_FOUND, ERR, NO_REPLY, BROADCAST };
enum class CMDtype : uint8_t { none, set, setat, get, getat, info, help, err };
constexpr uint32_t CMDFLAG_GET = 1, CMDFLAG_SET = 2;

struct ParsedCommand {
	uint32_t cmdId = 0;
	CMDtype type = CMDtype::get;
	int64_t val = 0;
	int64_t adr = 0;
};

struct CommandReply {
	int64_t val = 0;
	int64_t adr = 0;
	CommandReply(int64_t v) : val(v) {}
	CommandReply(int64_t v, int64_t a) : val(v), adr(a) {}
};

class CommandHandler {
public:
	CommandHandler(const char*, uint16_t, uint8_t = 0) {}
	virtual ~CommandHandler() {}
	virtual CommandStatus command(const ParsedCommand&, std::vector<CommandReply>&) { return CommandStatus::NOT_FOUND; }
	virtual std::string getHelpstring() { return ""; }
	virtual void registerCommands() {}
	template<typename ID>
	void registerCommand(const char*, ID, const char*, uint32_t) {}

	template<typename TVal>
	static CommandStatus handleGetSet(const ParsedCommand& cmd, std::vector<CommandReply>& replies, TVal& value){
		if(cmd.type == CMDtype::set){
			value = static_cast<TVal>(cmd.val);
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(value);
		}else{
			return CommandStatus::ERR;
		}
		return CommandStatus::OK;
	}
};

// ---- Errors ----
enum class ErrorCode : uint32_t { none = 0, systemError = 5, overvoltage = 11, brakeResistorFailure = 3,
	adcCalibrationError = 21, encoderAlignmentFailed = 20 };
enum class ErrorType : uint8_t { none, warning, critical, temporary };
struct Error {
	ErrorCode code; ErrorType type; std::string info;
	Error(ErrorCode c, ErrorType t, std::string i) : code(c), type(t), info(i) {}
};
class ErrorHandler {
public:
	static void addError(const Error& e);
	static void clearError(ErrorCode) {}
};

// ---- Flash ----
bool Flash_Write(uint16_t adr, uint16_t dat);
bool Flash_Read(uint16_t adr, uint16_t* buf);
void Flash_WriteBegin();
void Flash_WriteEnd();

// ---- RTOS thread: Delay() advances the simulation ----
namespace cpp_freertos {
class Thread {
public:
	Thread(const char*, uint16_t, uint8_t) {}
	virtual ~Thread() {}
	void Start() {}
	virtual void Run() = 0;
	static void Delay(uint32_t ms);
};
}

#endif

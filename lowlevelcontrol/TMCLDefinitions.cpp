#include "TMCLDefinitions.hpp"
#include <sstream>

using namespace youbot;
using namespace youbot::intrinsic;
using namespace youbot::intrinsic::TMCL;

std::string TMCL::RecvStatusToString(ReplyStatus in) {
  std::stringstream ss;
  if (in == ReplyStatus::NO_ERROR_)
	ss << " NO_ERROR";
  if (in == ReplyStatus::INVALID_COMMAND)
	ss << " INVALID_COMMAND";
  if (in == ReplyStatus::WRONG_TYPE)
	ss << " WRONG_TYPE";
  if (in == ReplyStatus::INVALID_VALUE)
	ss << " INVALID_VALUE";
  if (in == ReplyStatus::CONFIGURATION_EEPROM_LOCKED)
	ss << " CONFIGURATION_EEPROM_LOCKED";
  if (in == ReplyStatus::COMMAND_NOT_AVAILABLE)
	ss << " COMMAND_NOT_AVAILABLE";
  if (in == ReplyStatus::REPLY_WRITE_PROTECTED)
	ss << " REPLY_WRITE_PROTECTED";
  return ss.str();
}

std::string TMCL::StatusErrorFlagsToString(uint32_t in) {
  std::stringstream ss;
  if (in & (uint32_t)StatusErrorFlags::OVER_CURRENT)
	ss << " OVER_CURRENT";
  if (in & (uint32_t)StatusErrorFlags::UNDER_VOLTAGE)
	ss << " UNDER_VOLTAGE";
  if (in & (uint32_t)StatusErrorFlags::OVER_VOLTAGE)
	ss << " OVER_VOLTAGE";
  if (in & (uint32_t)StatusErrorFlags::OVER_TEMPERATURE)
	ss << " OVER_TEMPERATURE";
  if (in & (uint32_t)StatusErrorFlags::MOTOR_HALTED)
	ss << " MOTOR_HALTED";
  if (in & (uint32_t)StatusErrorFlags::HALL_SENSOR_ERROR)
	ss << " HALL_SENSOR_ERROR";
  if (in & (uint32_t)StatusErrorFlags::PWM_MODE_ACTIVE)
	ss << " PWM_MODE_ACTIVE";
  if (in & (uint32_t)StatusErrorFlags::VELOCITY_MODE_ACTIVE)
	ss << " VELOCITY_MODE_ACTIVE";
  if (in & (uint32_t)StatusErrorFlags::POSITION_MODE_ACTIVE)
	ss << " POSITION_MODE_ACTIVE";
  if (in & (uint32_t)StatusErrorFlags::TORQUE_MODE_ACTIVE)
	ss << " TORQUE_MODE_ACTIVE";
  if (in & (uint32_t)StatusErrorFlags::POSITION_REACHED)
	ss << " POSITION_REACHED";
  if (in & (uint32_t)StatusErrorFlags::INITIALIZED)
	ss << " INITIALIZED";
  if (in & (uint32_t)StatusErrorFlags::INITIALIZATION_ERROR)
	ss << " INITIALIZATION_ERROR";
  if (in & (uint32_t)StatusErrorFlags::TIMEOUT)
	ss << " TIMEOUT";
  if (in & (uint32_t)StatusErrorFlags::I2T_EXCEEDED)
	ss << " I2T_EXCEEDED";
  return ss.str();
}

std::string TMCL::ControllerModeToString(ControllerMode mode) {
  switch (mode)
  {
  case TMCL::MOTOR_STOP:
	return "MOTOR_STOP";
  case TMCL::POSITION_CONTROL:
	return "POSITION_CONTROL";
  case TMCL::VELOCITY_CONTROL:
	return "VELOCITY_CONTROL";
  case TMCL::NO_MORE_ACTION:
	return "NO_MORE_ACTION";
  case TMCL::SET_POSITION_TO_REFERENCE:
	return "SET_POSITION_TO_REFERENCE";
  case TMCL::CURRENT_MODE:
	return "CURRENT_MODE";
  case TMCL::INITIALIZE:
	return "INITIALIZE";
  default:
	break;
  }
  return "NOT_EXPECTED_INPUT";
}

std::string youbot::intrinsic::TMCL::CommutationMode2string(CommutationMode mode) {
	switch (mode)
	{
	case youbot::intrinsic::TMCL::CommutationMode::BLOCK_BASED_ON_HALL:
		return "BLOCK_BASED_ON_HALL";
	case youbot::intrinsic::TMCL::CommutationMode::SENSORLESS_BLOCK:
		return "SENSORLESS_BLOCK";
	case youbot::intrinsic::TMCL::CommutationMode::SINE_BASED_ON_HALL:
		return "SINE_BASED_ON_HALL";
	case youbot::intrinsic::TMCL::CommutationMode::SINE_BASED_ON_ENCODER:
		return "SINE_BASED_ON_ENCODER";
	case youbot::intrinsic::TMCL::CommutationMode::CONTROLLED_BLOCK:
		return "CONTROLLED_BLOCK";
	case youbot::intrinsic::TMCL::CommutationMode::CONTROLLED_SINE:
		return "CONTROLLED_SINE";
	default:
		return "not assumed commutation mode";
	}
}

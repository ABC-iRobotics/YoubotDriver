#include "TMCLDefinitions.hpp"
#include <sstream>

using namespace TMCL;

std::string RecvStatusToString(ReplyStatus in) {
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

std::string StatusErrorFlagsToString(StatusErrorFlags in) {
  std::stringstream ss;
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::OVER_CURRENT)
	ss << " OVER_CURRENT";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::UNDER_VOLTAGE)
	ss << " UNDER_VOLTAGE";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::OVER_VOLTAGE)
	ss << " OVER_VOLTAGE";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::OVER_TEMPERATURE)
	ss << " OVER_TEMPERATURE";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::MOTOR_HALTED)
	ss << " MOTOR_HALTED";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::HALL_SENSOR_ERROR)
	ss << " HALL_SENSOR_ERROR";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::PWM_MODE_ACTIVE)
	ss << " PWM_MODE_ACTIVE";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::VELOCITY_MODE_ACTIVE)
	ss << " VELOCITY_MODE_ACTIVE";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::POSITION_MODE_ACTIVE)
	ss << " POSITION_MODE_ACTIVE";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::TORQUE_MODE_ACTIVE)
	ss << " TORQUE_MODE_ACTIVE";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::POSITION_REACHED)
	ss << " POSITION_REACHED";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::INITIALIZED)
	ss << " INITIALIZED";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::TIMEOUT)
	ss << " TIMEOUT";
  if ((uint32_t)in & (uint32_t)StatusErrorFlags::I2T_EXCEEDED)
	ss << " I2T_EXCEEDED";
  return ss.str();
}
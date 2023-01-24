#include "YoubotJointAbstract.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include <sstream>
#include <stdexcept>

using namespace youbot;

YoubotJointAbstract::YoubotJointAbstract(int slaveIndex, const std::map<std::string,double>& config)
    : config(config), slaveIndex(slaveIndex) {}

void YoubotJointAbstract::InitializeJoint(bool forceConfiguration) {
  CollectBasicParameters();
  ConfigControlParameters(forceConfiguration);
  InitCommutation();
}

const YoubotJointAbstract::Parameters& YoubotJointAbstract::GetParameters() const {
  if (!parameters.intialized)
    throw::std::runtime_error("");
  return parameters;
}

const const std::map<std::string, double>& YoubotJointAbstract::GetConfig() const {
  return config;
}

int YoubotJointAbstract::GetSlaveIndex() const {
  return slaveIndex;
}

void YoubotJointAbstract::CollectBasicParameters() {
  GetFirmwareVersionViaMailbox(parameters.controllerNum, parameters.firmwareversion);
  // GetTickPerRounds
  parameters.ticksperround = GetEncoderResolutionViaMailbox();
  // Get cooldowntime_sec
  parameters.cooldowntime_sec = GetThermalWindingTimeSecViaMailbox();
  // Get other values from the config
  parameters.gearRatio = config.at("GearRatio");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " GearRatio: "
    + std::to_string(parameters.gearRatio));
  parameters.qMinDeg = config.at("qMinDeg");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " qMinDeg: "
    + std::to_string(parameters.qMinDeg));
  parameters.qMaxDeg = config.at("qMaxDeg");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " qMaxDeg: "
    + std::to_string(parameters.qMaxDeg));
  parameters.torqueconstantNmPerA = config.at("TorqueConstant_NmPerAmpere");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " TorqueConstant_NmPerAmpere: "
    + std::to_string(parameters.torqueconstantNmPerA));
  parameters.calibrationDirection = config.at("CalibrationDirection");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " CalibrationDirection: "
    + std::to_string(parameters.calibrationDirection));
  bool qDirectionSameAsEnc = config.at("qDirectionSameAsEnc");
  if (!qDirectionSameAsEnc) {
    parameters.torqueconstantNmPerA *= -1;
    parameters.gearRatio *= -1;
  }
  log(Log::info, " qDirectionSameAsEnc: " + std::to_string(int(qDirectionSameAsEnc)));
  parameters.qCalibrationRad = config.at("qCalibrationDeg") / 180. * M_PI;
  log(Log::info, " qCalibrationRad: " + std::to_string(parameters.qCalibrationRad));
  parameters.intialized = true;
}

bool YoubotJointAbstract::JointStatus::OverCurrent() const {
  return value & (uint32_t)StatusErrorFlags::OVER_CURRENT;
}

bool YoubotJointAbstract::JointStatus::UnderVoltage() const {
  return value & (uint32_t)StatusErrorFlags::UNDER_VOLTAGE;
};

bool YoubotJointAbstract::JointStatus::OverVoltage() const {
  return value & (uint32_t)StatusErrorFlags::OVER_VOLTAGE;
};

bool YoubotJointAbstract::JointStatus::OverTemperature() const {
  return value & (uint32_t)StatusErrorFlags::OVER_TEMPERATURE;
};

bool YoubotJointAbstract::JointStatus::MotorHalted() const {
  return value & (uint32_t)StatusErrorFlags::MOTOR_HALTED;
};

bool YoubotJointAbstract::JointStatus::HallSensorError() const {
  return value & (uint32_t)StatusErrorFlags::HALL_SENSOR_ERROR;
};

bool YoubotJointAbstract::JointStatus::EncoderError() const {
  return value & (uint32_t)StatusErrorFlags::ENCODER_ERROR;
};

bool YoubotJointAbstract::JointStatus::InitializationError() const {
  return value & (uint32_t)StatusErrorFlags::INITIALIZATION_ERROR;
};

bool YoubotJointAbstract::JointStatus::PWMMode() const {
  return value & (uint32_t)StatusErrorFlags::PWM_MODE_ACTIVE;
};

bool YoubotJointAbstract::JointStatus::VelocityMode() const {
  return value & (uint32_t)StatusErrorFlags::VELOCITY_MODE_ACTIVE;
};

bool YoubotJointAbstract::JointStatus::PositionMode() const {
  return value & (uint32_t)StatusErrorFlags::POSITION_MODE_ACTIVE;
};

bool YoubotJointAbstract::JointStatus::TorqueMode() const {
  return value & (uint32_t)StatusErrorFlags::TORQUE_MODE_ACTIVE;
};

bool YoubotJointAbstract::JointStatus::EmergencyStop() const {
  return value & (uint32_t)StatusErrorFlags::EMERGENCY_STOP;
};

bool YoubotJointAbstract::JointStatus::FreeRunning() const {
  return value & (uint32_t)StatusErrorFlags::FREERUNNING;
};

bool YoubotJointAbstract::JointStatus::PositionReached() const {
  return value & (uint32_t)StatusErrorFlags::POSITION_REACHED;
};

bool YoubotJointAbstract::JointStatus::Initialized() const {
  return value & (uint32_t)StatusErrorFlags::INITIALIZED;
};

bool YoubotJointAbstract::JointStatus::Timeout() const {
  return value & (uint32_t)StatusErrorFlags::TIMEOUT;
};

bool YoubotJointAbstract::JointStatus::I2TExceeded() const {
  return value & (uint32_t)StatusErrorFlags::I2T_EXCEEDED;
};

std::string YoubotJointAbstract::JointStatus::toString() const {
  std::stringstream ss;
  if (OverCurrent())
    ss <<  " OVER_CURRENT";
  if (UnderVoltage())
    ss << " UNDER_VOLTAGE";
  if (OverVoltage())
    ss << " OVER_VOLTAGE";
  if (OverTemperature())
    ss << " OVER_TEMPERATURE";
  if (MotorHalted())
    ss << " MOTOR_HALTED";
  if (HallSensorError())
    ss << " HALL_SENSOR_ERROR";
  if (EncoderError())
    ss << " ENCODER_ERROR";
  if (InitializationError())
    ss << " INITIALIZATION_ERROR";
  if (PWMMode())
    ss << " PWM_MODE_ACTIVE";
  if (VelocityMode())
    ss << " VELOCITY_MODE_ACTIVE";
  if (PositionMode())
    ss << " POSITION_MODE_ACTIVE";
  if (TorqueMode())
    ss << " TORQUE_MODE_ACTIVE";
  if (EmergencyStop())
    ss << " EMERGENCY_STOP";
  if (FreeRunning())
    ss << " FREERUNNING";
  if (PositionReached())
    ss << " POSITION_REACHED";
  if (Initialized())
    ss << " INITIALIZED";
  if (Timeout())
    ss << " TIMEOUT";
  if (I2TExceeded())
    ss << " I2T_EXCEEDED";
  return ss.str();
}

void YoubotJointAbstract::ProcessReturn::Print() const {
  log(Log::info, "Pos: " + std::to_string(qRad) + "[rad] (" + std::to_string(qRad/M_PI*180.) + "[deg],"
    + std::to_string((int)encoderPosition) + "[tick]) Vel: " +
    std::to_string(dqRadPerSec) + "[rad/s] (" + std::to_string(dqRadPerSec / M_PI * 180.) + "[deg/s],"
    + std::to_string((int)motorVelocityRPM) + "RPM) torque: " +
    std::to_string(tau) + "[NM] (" + std::to_string(currentmA) + "[mA])");
}

YoubotJointAbstract::ProcessReturn::ProcessReturn() : status(0), encoderPosition(-1),
currentmA(-1), motorVelocityRPM(-1), motorPWM(-1) {};

void YoubotJointAbstract::ReqJointSpeedRadPerSec(double value) {
  ReqMotorSpeedRPM(qRadPerSec2RPM(value));
}

void youbot::YoubotJointAbstract::ReqJointTorqueNm(double value) {
  ReqMotorCurrentmA(Nm2mA(value));
}

void YoubotJointAbstract::ReqJointPositionRad(double rad) {
  int32_t ticks = qRad2Ticks(rad);
  ReqMotorPositionTick(ticks);
}

double YoubotJointAbstract::GetJointPositionRad() {
  return GetProcessReturnData().qRad;
}

double youbot::YoubotJointAbstract::GetJointSpeedRadPerSec() {
  return GetProcessReturnData().dqRadPerSec;
}

double youbot::YoubotJointAbstract::GetJointTorqueNm() {
  return GetProcessReturnData().tau;
}

int32_t youbot::YoubotJointAbstract::GetMotorPosTick() {
  return GetProcessReturnData().encoderPosition;
}

int32_t youbot::YoubotJointAbstract::GetMotorSpeedRPM() {
  return GetProcessReturnData().motorVelocityRPM;
}

int32_t youbot::YoubotJointAbstract::GetMotorCurrentmA() {
  return GetProcessReturnData().currentmA;
}

void youbot::YoubotJointAbstract::CheckI2tAndTimeoutError(JointStatus status) {
  if (status.I2TExceeded()) {
    log(Log::fatal, "I2t exceeded in slave " + std::to_string(slaveIndex) + " (" + status.toString() + ")");
    SLEEP_MILLISEC(10);
    throw std::runtime_error("I2t exceeded");
  }
  if (status.Timeout()) {
    log(Log::fatal, "Timeout in slave " + std::to_string(slaveIndex) + " (" + status.toString() + ")");
    SLEEP_MILLISEC(10);
    throw std::runtime_error("Timeout");
  }
}

void YoubotJointAbstract::InitCommutation() {
  auto status = GetJointStatusViaMailbox();
  if (status.Initialized()) {
    log(Log::info, "Initialization of Joint " + std::to_string(slaveIndex) + " already initialized, status: " + status.toString());
    return;
  }
  // Initialization
  SLEEP_MILLISEC(500); // Wait to finish the robot the current moves - controller likes it
  log(Log::info, "Initialization of Joint " + std::to_string(slaveIndex) + " status before calibration: " + status.toString());
  // Reset I2t flag
  if (status.I2TExceeded())
    ResetI2TExceededViaMailbox();
  // Reset timeout flag (waiting for I2t clearence will cause timeout as well)
  if(status.Timeout() || status.I2TExceeded())
    ResetTimeoutViaMailbox();
  // Get the new status
  if (status.Timeout() || status.I2TExceeded()) {
    status = GetJointStatusViaMailbox();
    log(Log::info, "Joint " + std::to_string(slaveIndex) + " status after timeout, I2t cleared: " + status.toString());
  }
  StartInitializationViaMailbox();
  auto start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point end;
  do {
    auto status = GetJointStatusViaMailbox();
    if (status.Timeout() || status.I2TExceeded() || status.InitializationError()) {
      StopViaMailbox();
      log(Log::fatal , "Error (timeout/I2t/init error) during initialization of Joint " + std::to_string(slaveIndex));
      throw std::runtime_error("Error (timeout/I2t/init error) during initialization");
    }
    if (status.Initialized()) {
      end = std::chrono::steady_clock::now();
      log(Log::info, "Joint " + std::to_string(slaveIndex) + " is initialized, elapsed time: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) + "[ms]");
      return;
    }
    end = std::chrono::steady_clock::now();
  } while (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() < 2000);
  StopViaMailbox();
  log(Log::fatal, "Joint " + std::to_string(slaveIndex) + " error during initialization (possible causes: it is in the problematic end position, on windows: other processes take too much processor resource)");
  SLEEP_MILLISEC(10);
  throw std::runtime_error("One joint is not initialized and cannot be done it... ");
}

double YoubotJointAbstract::Ticks2qRad(int32_t ticks) const {
  return double(ticks) * 2. * M_PI * parameters.gearRatio / double(parameters.ticksperround) + parameters.qCalibrationRad;
}

int32_t YoubotJointAbstract::qRad2Ticks(double qDeg) const {
  return int32_t((qDeg - parameters.qCalibrationRad) / (2. * M_PI * parameters.gearRatio / double(parameters.ticksperround)));
}

double youbot::YoubotJointAbstract::RPM2qRadPerSec(int32_t RPM) const {
  return double(RPM) / 60. * parameters.gearRatio * 2. * M_PI;
}

int32_t youbot::YoubotJointAbstract::qRadPerSec2RPM(double radpersec) const {
  return radpersec * 60. / parameters.gearRatio / (2. * M_PI);
}

double youbot::YoubotJointAbstract::mA2Nm(int32_t mA) const {
  return double(mA) / 1000. * parameters.torqueconstantNmPerA;
}

int32_t youbot::YoubotJointAbstract::Nm2mA(double Nm) const {
  return int32_t(Nm / parameters.torqueconstantNmPerA * 1000.);
}

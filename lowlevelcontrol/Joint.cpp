#include "Joint.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include <sstream>
#include <stdexcept>

using namespace youbot;

Joint::Joint(int slaveIndex, const std::map<std::string,double>& config,
  EtherCATMaster::Ptr center) : config(config), slaveIndex(slaveIndex), center(center) {}

void Joint::InitializeJoint(bool forceConfiguration) {
  CollectBasicParameters();
  ConfigControlParameters(forceConfiguration);
  InitCommutation();
}

const Joint::Parameters& Joint::GetParameters() const {
  if (!parameters.intialized)
    throw::std::runtime_error("");
  return parameters;
}

const std::map<std::string, double>& Joint::GetConfig() const {
  return config;
}

int Joint::GetSlaveIndex() const {
  return slaveIndex;
}

void Joint::CollectBasicParameters() {
  GetFirmwareVersionViaMailbox(parameters.controllerNum, parameters.firmwareversion);
  // GetTickPerRounds
  parameters.ticksperround = GetEncoderResolutionViaMailbox();
  // GetCommutationMode
  parameters.commutationmode = GetCommutationModeViaMailbox();
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

JointState Joint::GetLatestState() const {
  return JointState(GetQLatestRad(), GetTicksLatest(),
    GetDQLatestRad(), GetRPMLatest(), GetTauLatestNm(), statusLatest);
}

void Joint::LogLatestState() const {
  auto motorticks = ticksLatest.load().value;
  auto mA = mALatest.load().value;
  auto RPM = RPMLatest.load().value;
  auto qRad = Ticks2qRad(motorticks);
  log(Log::info, "Pos: " + std::to_string(qRad) + "[rad] (" + std::to_string(qRad/M_PI*180.) + "[deg],"
    + std::to_string((int)motorticks) + "[tick]) Vel: " +
    std::to_string(RPM2qRadPerSec(RPM)) + "[rad/s] (" + std::to_string((int)RPM) + "RPM) torque: " +
    std::to_string(mA2Nm(mA)) + "[NM] (" + std::to_string(mA) + "[mA])");
  log(Log::info, "Status: " + statusLatest.load().value.toString());
}

void Joint::ReqJointSpeedRadPerSec(double value) {
  ReqMotorSpeedRPM(qRadPerSec2RPM(value));
}

void youbot::Joint::ReqJointTorqueNm(double value) {
  ReqMotorCurrentmA(Nm2mA(value));
}

Data<double> youbot::Joint::GetQLatestRad() const {
  auto temp = ticksLatest.load();
  return Data<double>(Ticks2qRad(temp.value), temp.origin);
}

Data<double> youbot::Joint::GetDQLatestRad() const {
  auto temp = RPMLatest.load();
  return Data<double>(RPM2qRadPerSec(temp.value), temp.origin);
}

Data<double> youbot::Joint::GetTauLatestNm() const {
  auto temp = mALatest.load();
  return Data<double>(mA2Nm(temp.value), temp.origin);
}

Data<int32_t> youbot::Joint::GetTicksLatest() const {
  return ticksLatest.load();
}

Data<int32_t> youbot::Joint::GetRPMLatest() const {
  return RPMLatest.load();
}

Data<int32_t> youbot::Joint::GetMALatest() const {
  return mALatest.load();
}

Data<youbot::JointStatus> youbot::Joint::GetStatusLatest() const {
  return statusLatest.load();
}

void youbot::Joint::ReqMotorTorqueNm(double value) {
  ReqJointTorqueNm(value / parameters.gearRatio );
}

void Joint::ReqJointPositionRad(double rad) {
  int32_t motorticks = qRad2Ticks(rad);
  ReqMotorPositionTick(motorticks);
}

void youbot::Joint::CheckI2tAndTimeoutError(JointStatus status) {
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

void Joint::InitCommutation() {
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

double Joint::Ticks2qRad(int32_t motorticks) const {
  return double(motorticks) * 2. * M_PI * parameters.gearRatio / double(parameters.ticksperround) + parameters.qCalibrationRad;
}

int32_t Joint::qRad2Ticks(double qDeg) const {
  return int32_t((qDeg - parameters.qCalibrationRad) / (2. * M_PI * parameters.gearRatio / double(parameters.ticksperround)));
}

double youbot::Joint::RPM2qRadPerSec(int32_t RPM) const {
  return double(RPM) / 60. * parameters.gearRatio * 2. * M_PI;
}

int32_t youbot::Joint::qRadPerSec2RPM(double radpersec) const {
  return radpersec * 60. / parameters.gearRatio / (2. * M_PI);
}

double youbot::Joint::mA2Nm(int32_t mA) const {
  return double(mA) / 1000. * parameters.torqueconstantNmPerA;
}

int32_t youbot::Joint::Nm2mA(double Nm) const {
  return int32_t(Nm / parameters.torqueconstantNmPerA * 1000.);
}

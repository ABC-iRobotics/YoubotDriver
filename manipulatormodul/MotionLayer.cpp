#include "Manager.hpp"
#include "adapters.hpp"
#include "JointVirtual.hpp"
#include <stdexcept>
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot;

void youbot::MotionLayer::Status::LogStatus() const {
  log(Log::info, "Manipulator status: " + ManipulatorTask::Type2String(motion));
  for (int i = 0; i < 5; i++) {
    log(Log::info, "Joint " + std::to_string(i) + ": q=" + std::to_string(joint[i].q.value) + "[rad] dq/dt="
      + std::to_string(joint[i].dq.value) + "[rad/s] tau="
      + std::to_string(joint[i].tau.value) + "[Nm]");
    log(Log::info, "Joint " + std::to_string(i) + " status: " + joint[i].status.value.toString());
  }
}

void youbot::MotionLayer::_SoftLimit(
  ManipulatorCommand& cmd, const JointsState& status) const {
  static double limitZoneDeg = 4;
  static double corrjointspeed = 0.03;
  static double limitZoneRad = limitZoneDeg / 180 * M_PI;
  for (int i = 0; i < 5; i++) {
    auto q = status.joint[i].q.value;
    auto p = man->GetJoint(i)->GetParameters();
    auto qmin_lim = p.qMinDeg / 180 * M_PI + limitZoneRad;
    auto qmax_lim = p.qMaxDeg / 180 * M_PI - limitZoneRad;
    auto& cmd_ = cmd.commands[i];
    if (q < qmin_lim) {
      switch (cmd_.GetType()) {
      case cmd_.JOINT_POSITION:
        if (cmd_.Get<double>() < qmin_lim)
          cmd_.Set(qmin_lim);
        break;
      case cmd_.JOINT_VELOCITY:
        if (cmd_.Get<double>() < corrjointspeed)
          cmd_.Set(corrjointspeed);
        break;
      case cmd_.JOINT_TORQUE:
        if (cmd_.Get<double>() < 0)
          cmd_.Set(0);
        break;
      default:
        break;
      }
    }
    if (q > qmax_lim) {
      switch (cmd_.GetType()) {
      case cmd_.JOINT_POSITION:
        if (cmd_.Get<double>() > qmax_lim)
          cmd_.Set(qmax_lim);
        break;
      case cmd_.JOINT_VELOCITY:
        if (cmd_.Get<double>() > -corrjointspeed)
          cmd_.Set(-corrjointspeed);
        break;
      case cmd_.JOINT_TORQUE:
        if (cmd_.Get<double>() > 0)
          cmd_.Set(0);
        break;
      default:
        break;
      }
    }
  }
}

youbot::MotionLayer::Status youbot::MotionLayer::GetStatus() {
  Status status;
  if (man)
    for (int i = 0; i < 5; i++) {
      auto j = man->GetJoint(i);
      if (j != nullptr)
        status.joint[i] = j->GetLatestState();
    }
  status.motion = motionStatus.load();
  status.manipulatorStatus = manipulatorStatus.load();
  return status;
}

Eigen::VectorXd youbot::MotionLayer::GetTrueStatus() const {
  Eigen::VectorXd out = Eigen::VectorXd::Zero(5);
  if (center != nullptr && man!=nullptr) {
    if (center->GetType() == EtherCATMaster::VIRTUAL) {
      for (int i=0;i<5;i++)
        if (man->GetJoint(i) != nullptr)
          out[i] = std::dynamic_pointer_cast<intrinsic::JointVirtual>(
            man->GetJoint(i))->GetJointPositionTRUE();
    }
    else {
      for (int i = 0; i < 5; i++)
        if (man->GetJoint(i) != nullptr)
          out[i] = man->GetJoint(i)->GetQLatestRad().value;
    }
  }
  return out;
}

void youbot::MotionLayer::StopManipulatorTask() {
  stoptask = true;
}

// Tasks
void youbot::MotionLayer::DoManipulatorTask(ManipulatorTask::Ptr task, double time_limit) {
  stoptask = false;
  motionStatus.store(task->GetType());
  taskrunning = true;
  auto start = std::chrono::steady_clock::now();
  int elapsed_ms;
  man->CheckAndResetErrorFlagsViaMailbox();
  task->Initialize(man->GetStateLatest());
  do {
    auto stateLatest = man->GetStateLatest();
    auto man_c = task->GetCommand(stateLatest);
    if (manipulatorStatus.load().IsCalibrated())
      _SoftLimit(man_c, stateLatest); // apply soft limit - todo only if commutation initialized and calibrated
    for (int i = 0; i < 5; i++) {
      auto& cmd_ = man_c.commands[i];
      auto& j = man->GetJoint(i);
      switch (cmd_.GetType()) {
      case BLDCCommand::JOINT_POSITION:
        if (!manipulatorStatus.load().IsCalibrated())
          throw std::runtime_error("Position command used on not calibrated arm");
        j->ReqJointPositionRad(cmd_.Get<double>());
        break;
      case BLDCCommand::MOTOR_TICK:
        if (!manipulatorStatus.load().IsCalibrated())
          throw std::runtime_error("Position command used on not calibrated arm");
        j->ReqMotorPositionTick(cmd_.Get<int>());
        break;
      case BLDCCommand::JOINT_VELOCITY:
        if (!manipulatorStatus.load().IsCommutationInitialized())
          throw std::runtime_error("Velocity command used on not commutated arm");
        j->ReqJointSpeedRadPerSec(cmd_.Get<double>());
        break;
      case BLDCCommand::JOINT_TORQUE:
        if (!manipulatorStatus.load().IsCommutationInitialized())
          throw std::runtime_error("Torque command used on not commutated arm");
        j->ReqJointTorqueNm(cmd_.Get<double>());
        break;
      case BLDCCommand::MOTOR_RPM:
        if (!manipulatorStatus.load().IsCommutationInitialized())
          throw std::runtime_error("Velocity command used on not commutated arm");
        j->ReqMotorSpeedRPM(cmd_.Get<int>());
        break;
      case BLDCCommand::MOTOR_CURRENT_MA:
        if (!manipulatorStatus.load().IsCommutationInitialized())
          throw std::runtime_error("Current command used on not commutated arm");
        j->ReqMotorCurrentmA(cmd_.Get<int>());
        break;
      case BLDCCommand::MOTOR_VOLTAGE:
        if (!manipulatorStatus.load().IsCommutationInitialized())
          throw std::runtime_error("Voltage command used on not commutated arm");
        j->ReqVoltagePWM(cmd_.Get<int>());
        break;
      case BLDCCommand::INITIALIZE_COMMUTATION:
        j->ReqInitializationViaProcess();
        break;
      case BLDCCommand::MOTOR_STOP:
        j->ReqStop();
        break;
      case BLDCCommand::ENCODER_SET_REFERENCE:
        j->ReqEncoderReference(cmd_.Get<int>());
        break;
      default:
        break;
      }
    }
    center->ExchangeProcessMsg();
    SLEEP_MILLISEC(10);// compute adaptively the remained time..
    elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start).count();
    if (task->Finished()) {
      if (task->GetType() == ManipulatorTask::INITIALIZATION) {
        auto status = manipulatorStatus.load();
        status.Set(ManipulatorStatus::COMMUTATION_INITIALIZED, true);
        manipulatorStatus.store(status);
      }
      if (task->GetType() == ManipulatorTask::CALIBRATION) {
        auto status = manipulatorStatus.load();
        status.Set(ManipulatorStatus::CALIBRATED, true);
        manipulatorStatus.store(status);
      }
      break;
    }
  } while (elapsed_ms < time_limit * 1000. && !stoptask);
  taskrunning = false;
  motionStatus.store(ManipulatorTask::STOPPED);
}

bool youbot::MotionLayer::IsManipulatorTaskRunning() const {
  return taskrunning;
}

MotionLayer::MotionLayer(
  const std::string& configfilepath, bool virtual_)
 : configfilepath(configfilepath), virtual_(virtual_) {};

void MotionLayer::Initialize() {
  {
    auto status = manipulatorStatus.load();
    status.Set(ManipulatorStatus::START_UP, true);
    manipulatorStatus.store(status);
  }
  // Get Configfile
  Config config(configfilepath);
  config.Init();
  // Initialize logger
  Log::Setup(config.logConfig);
  // Initialize etherCAT bus
  if (!center) {
    if (virtual_)
      center = EtherCATMaster::CreateVirtual();
    else {
      // Find appropriate ethernet adapter and open connection
      char name[1000];
      if (findYouBotEtherCatAdapter(name))
        log(Log::info, "Adapter found:" + std::string(name));
      else {
        log(Log::fatal, "Adapter with turned on youBot arm NOT found!");
        throw std::runtime_error("Adapter with turned on youBot arm NOT found!");
      }
      center = EtherCATMaster::CreatePhysical(name);
    }
  }
  // Initialize manipulator
  if (!man)
    man = std::make_unique<Manipulator>(config, center);
  man->ConfigJointControlParameters();
  man->CheckAndResetErrorFlagsViaMailbox();
  {
    bool commutation_initialized = true;
    auto status = man->GetStateLatest();
    for (int i = 0; i < 5; i++)
      commutation_initialized &= status.joint[i].status.value.Initialized();
    if (commutation_initialized) {
      auto status = manipulatorStatus.load();
      status.Set(ManipulatorStatus::COMMUTATION_INITIALIZED, true);
      manipulatorStatus.store(status);
    }
  }
  // Commutation
  man->InitializeManipulator();
  {
    auto status = manipulatorStatus.load();
    status.Set(ManipulatorStatus::COMMUTATION_INITIALIZED, true);
    manipulatorStatus.store(status);
  }
  // Calibration
  man->Calibrate();
  {
    auto status = manipulatorStatus.load();
    status.Set(ManipulatorStatus::CALIBRATED, true);
    manipulatorStatus.store(status);
  }
  {
    auto status = manipulatorStatus.load();
    status.Set(ManipulatorStatus::START_UP, false);
    status.Set(ManipulatorStatus::CONFIGURATED, true);
    manipulatorStatus.store(status);
  }
  motionStatus.store(ManipulatorTask::STOPPED);
}

bool youbot::MotionLayer::ManipulatorStatus::IsConfigInProgress() const {
  return value & START_UP;
}

bool youbot::MotionLayer::ManipulatorStatus::IsConfigurated() const {
  return value & CONFIGURATED;
}

bool youbot::MotionLayer::ManipulatorStatus::IsCommutationInitialized() const {
  return value & COMMUTATION_INITIALIZED;
}

bool youbot::MotionLayer::ManipulatorStatus::IsCalibrated() const {
  return value & CALIBRATED;
}

bool youbot::MotionLayer::ManipulatorStatus::Is(Flag flag) const {
  return value & flag;
}

void youbot::MotionLayer::ManipulatorStatus::Set(Flag flag, bool in) {
  if (in)
    value |= flag;
  else
    value &= ~flag;
}

std::string youbot::MotionLayer::ManipulatorStatus::ToString() const {
  std::stringstream ss;
  if (Is(START_UP))
    ss << " START_UP";
  if (Is(CONFIGURATED))
    ss << " CONFIGURATED";
  if (Is(COMMUTATION_INITIALIZED))
    ss << " COMMUTATION_INITIALIZED";
  if (Is(CALIBRATED))
    ss << " CALIBRATED";
  return ss.str();
}

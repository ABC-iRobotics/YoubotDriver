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
    _SoftLimit(man_c, stateLatest); // apply soft limit - todo only if commutation initialized and calibrated
    for (int i = 0; i < 5; i++) {
      auto& cmd_ = man_c.commands[i];
      auto& j = man->GetJoint(i);
      switch (cmd_.GetType()) {
      case BLDCCommand::JOINT_POSITION:
        j->ReqJointPositionRad(cmd_.Get<double>());
        break;
      case BLDCCommand::JOINT_VELOCITY:
        j->ReqJointSpeedRadPerSec(cmd_.Get<double>());
        break;
      case BLDCCommand::JOINT_TORQUE:
        j->ReqJointTorqueNm(cmd_.Get<double>());
        break;
      case BLDCCommand::MOTOR_TICK:
        j->ReqMotorPositionTick(cmd_.Get<int>());
        break;
      case BLDCCommand::MOTOR_RPM:
        j->ReqMotorSpeedRPM(cmd_.Get<int>());
        break;
      case BLDCCommand::MOTOR_CURRENT_MA:
        j->ReqMotorCurrentmA(cmd_.Get<int>());
        break;
      case BLDCCommand::MOTOR_VOLTAGE:
        j->ReqVoltagePWM(cmd_.Get<int>());
        break;
      case BLDCCommand::INITIALIZE:
        j->ReqInitializationViaProcess();
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
  } while (elapsed_ms < time_limit * 1000. && !stoptask && !task->Finished());
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
  motionStatus.store(ManipulatorTask::INITIALIZATION);
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
  man->InitializeManipulator();
  man->Calibrate();
  motionStatus.store(ManipulatorTask::STOPPED);
}

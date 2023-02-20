#include "YoubotManipulatorMotionLayer.hpp"
#include "adapters.hpp"
#include "JointVirtual.hpp"
#include <stdexcept>
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot;

void youbot::YoubotManipulatorMotionLayer::Status::LogStatus() const {
  log(Log::info, "Manipulator status: " + ManipulatorTask::Type2String(motion));
  for (int i = 0; i < 5; i++) {
    log(Log::info, "Joint " + std::to_string(i) + ": q=" + std::to_string(joint[i].q.value) + "[rad] dq/dt="
      + std::to_string(joint[i].dq.value) + "[rad/s] tau="
      + std::to_string(joint[i].tau.value) + "[Nm]");
    log(Log::info, "Joint " + std::to_string(i) + " status: " + joint[i].status.value.toString());
  }
}

void youbot::YoubotManipulatorMotionLayer::_SoftLimit(
  ManipulatorCommand& cmd, const JointsState& status) const {
  static double limitZoneDeg = 4;
  static double corrjointspeed = 0.03;
  static double limitZoneRad = limitZoneDeg / 180 * M_PI;
  for (int i = 0; i < 5; i++) {
    auto q = status.joint[i].q.value;
    auto p = man->GetJoint(i)->GetParameters();
    auto qmin_lim = p.qMinDeg / 180 * M_PI + limitZoneRad;
    auto qmax_lim = p.qMaxDeg / 180 * M_PI - limitZoneRad;
    if (q < qmin_lim) {
      switch (cmd.type) {
      case cmd.JOINT_POSITION:
        if (cmd.value(i) < qmin_lim)
          cmd.value(i) = qmin_lim;
        break;
      case cmd.JOINT_VELOCITY:
        if (cmd.value(i) < corrjointspeed)
          cmd.value(i) = corrjointspeed;
        break;
      case cmd.JOINT_TORQUE:
        if (cmd.value(i) < 0)
          cmd.value(i) = 0;
        break;
      default:
        break;
      }
    }
    if (q > qmax_lim) {
      switch (cmd.type) {
      case cmd.JOINT_POSITION:
        if (cmd.value(i) > qmax_lim)
          cmd.value(i) = qmax_lim;
        break;
      case cmd.JOINT_VELOCITY:
        if (cmd.value(i) > -corrjointspeed)
          cmd.value(i) = -corrjointspeed;
        break;
      case cmd.JOINT_TORQUE:
        if (cmd.value(i) > 0)
          cmd.value(i) = 0;
        break;
      default:
        break;
      }
    }
  }
}

youbot::YoubotManipulatorMotionLayer::Status youbot::YoubotManipulatorMotionLayer::GetStatus() {
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

Eigen::VectorXd youbot::YoubotManipulatorMotionLayer::GetTrueStatus() const {
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

void youbot::YoubotManipulatorMotionLayer::StopTask() {
  stoptask = true;
}

// Tasks

void youbot::YoubotManipulatorMotionLayer::DoTask(ManipulatorTask::Ptr task, double time_limit) {
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
    _SoftLimit(man_c, stateLatest); // apply soft limit
    switch (man_c.type) {
    case ManipulatorCommand::JOINT_POSITION:
      man->ReqJointPositionRad(man_c.value[0], man_c.value[1],
        man_c.value[2], man_c.value[3], man_c.value[4]);
      break;
    case ManipulatorCommand::JOINT_VELOCITY:
      man->ReqJointSpeedRadPerSec(man_c.value[0], man_c.value[1],
        man_c.value[2], man_c.value[3], man_c.value[4]);
      break;
    case ManipulatorCommand::JOINT_TORQUE:
      man->ReqJointTorqueNm(man_c.value[0], man_c.value[1],
        man_c.value[2], man_c.value[3], man_c.value[4]);
      break;
      /*
      case ManipulatorCommand::ENCODER_SET_REFERENCE:
      man->Req(man_c.value[0], man_c.value[1],
      man_c.value[2], man_c.value[3], man_c.value[4]);
      break;*/
    }
    center->ExchangeProcessMsg();
    SLEEP_MILLISEC(10);// compute adaptively the remained time..
    elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start).count();
  } while (elapsed_ms < time_limit * 1000. && !stoptask && !task->Finished());
  taskrunning = false;
  motionStatus.store(ManipulatorTask::STOPPED);
}

bool youbot::YoubotManipulatorMotionLayer::IsRunning() const {
  return taskrunning;
}

YoubotManipulatorMotionLayer::YoubotManipulatorMotionLayer(
  const std::string& configfilepath, bool virtual_)
 : configfilepath(configfilepath), virtual_(virtual_) {};

void YoubotManipulatorMotionLayer::Initialize() {
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

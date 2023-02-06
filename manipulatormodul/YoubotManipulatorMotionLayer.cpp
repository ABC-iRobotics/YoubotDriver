#include "YoubotManipulatorMotionLayer.hpp"
#include "adapters.hpp"
#include "YoubotJointVirtual.hpp"
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
          out[i] = std::dynamic_pointer_cast<YoubotJointVirtual>(
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
  man->CheckAndResetErrorFlags();
  task->Initialize(man->GetStateLatest());
  do {
    auto man_c = task->GetCommand(man->GetStateLatest());
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
  YoubotConfig config(configfilepath);
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
    man = std::make_unique<YoubotManipulator>(config, center);
  man->InitializeManipulator();
  man->Calibrate();
  motionStatus.store(ManipulatorTask::STOPPED);
}

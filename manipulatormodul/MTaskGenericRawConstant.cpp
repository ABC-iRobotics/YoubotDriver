#include "MTaskZeroCurrent.hpp"
#include "Time.hpp"
#include "MTaskGenericRawConstant.hpp"

using namespace youbot;

youbot::MTaskGenericRawConstant::MTaskGenericRawConstant(const std::vector<Cmd>& cmd, double P_constant) : cmd(cmd), P_constant(P_constant) {}

ManipulatorCommand youbot::MTaskGenericRawConstant::GetCommand(
  const JointsState& new_state) {
  BLDCCommand cmd_out[5];
  double pos;
  for (int i = 0; i < 5; i++) {
    bool pos_mode = true;
    switch (cmd[i].type)
    {
    case STOP:
      cmd_out[i] = BLDCCommand(BLDCCommand::MOTOR_STOP, 0);
      break;
    case POSITION_MOTOR_TICK:
      cmd_out[i] = BLDCCommand(BLDCCommand::MOTOR_RPM,
        P_constant * (cmd[i].value - new_state.joint[i].motorticks.value) / 4000. * 60.);
      break;
    case POSITION_MOTOR_RAD:
      cmd_out[i] = BLDCCommand(BLDCCommand::MOTOR_RPM,
        P_constant * (cmd[i].value / M_PI / 2. - new_state.joint[i].motorticks.value / 4000.) * 60.);
      break;
    case VELOCITY_MOTOR_RPM:
      cmd_out[i] = BLDCCommand(BLDCCommand::MOTOR_RPM, cmd[i].value);
      break;
    case TORQUE_MOTOR_MA:
      cmd_out[i] = BLDCCommand(BLDCCommand::MOTOR_CURRENT_MA, cmd[i].value);
      break;
    case TORQUE_MOTOR_NM:
      cmd_out[i] = BLDCCommand(BLDCCommand::MOTOR_TORQUE_NM, cmd[i].value); // TODO... 
      break;
    case POSITION_JOINT_RAD:
      cmd_out[i] = BLDCCommand(BLDCCommand::JOINT_VELOCITY,
        P_constant * (cmd[i].value - new_state.joint[i].q.value));
      break;
    case VELOCITY_JOINT_RADPERSEC:
      cmd_out[i] = BLDCCommand(BLDCCommand::JOINT_VELOCITY, cmd[i].value);
      break;
    case TORQUE_JOINT_NM:
      cmd_out[i] = BLDCCommand(BLDCCommand::JOINT_TORQUE, cmd[i].value);
      break;
    default:
      throw std::runtime_error("not defined type");
      break;
    }
  }
  return { cmd_out[0], cmd_out[1], cmd_out[2], cmd_out[3], cmd_out[4] };
}

MTask::TaskType youbot::MTaskGenericRawConstant::GetType() const {
  return GENERIC_RAW_CONSTANT;
}

bool youbot::MTaskGenericRawConstant::_taskFinished() const {
  return false;
}

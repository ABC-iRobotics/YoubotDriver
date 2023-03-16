#include "MTaskCommutation.hpp"
#include "Time.hpp"

using namespace youbot;

ManipulatorCommand youbot::MTaskCommutation::GetCommand(const JointsState& new_state) {
    // Check if the commutation should have finished - if not then run to error
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started_at).count() > 2000)
        throw std::runtime_error("Unsuccessful commutation too much time elapsed");
    // Check the joint states and call initialize/stop messages
    finished = true;
    BLDCCommand cmd[5];
    for (int i = 0; i < 5; i++) {
        bool inited = new_state.joint[i].status.value.Initialized();
        finished &= inited;
        if (inited)
            cmd[i] = BLDCCommand(BLDCCommand::MOTOR_STOP, 0);
        else
            cmd[i] = BLDCCommand(BLDCCommand::INITIALIZE_COMMUTATION, 0);
    }
    // Send out the resulting commands
    return { cmd[0], cmd[1], cmd[2], cmd[3], cmd[4] };
}

MTask::TaskType youbot::MTaskCommutation::GetType() const {
  return COMMUTATION;
}

bool youbot::MTaskCommutation::_taskFinished() const {
  return finished;
}

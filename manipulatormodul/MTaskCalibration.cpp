#include "MTaskCalibration.hpp"
#include "Time.hpp"
#include "Logger.hpp"

using namespace youbot;

void youbot::MTaskCalibration::Initialize(const Config* config) {
  // Get directions..., get k motor_joint values...
  for (int i = 0; i < 5; i++) {
	backward[i] = bool(config->jointConfigs[i].at("CalibrationDirection"))
	  ^ bool(config->jointConfigs[i].at("qDirectionSameAsEnc"));
	for (int i = 0; i < 5; i++)
	  cmd.commands[i] = { BLDCCommand::JOINT_VELOCITY,backward[i] ? -calJointRadPerSec : calJointRadPerSec };
	log(Log::info, "Calibration of joint " + std::to_string(i) + "started");
  }
  initialized = true;
}

ManipulatorCommand youbot::MTaskCalibration::GetCommand(const JointsState& new_state) {
  // Check if it is initialized
  if (!initialized)
	throw std::runtime_error("Not initialized");
  // First check if we are in the waiting period
  if (std::chrono::duration_cast<std::chrono::milliseconds>(
	std::chrono::steady_clock::now() - started_at).count() < 200)
	return cmd; // return the initialized command
  // Till it get to the all holding state
  if (reached_since < 4) {
	// if dt>200ms, check if the joint has already stopped
	std::string str = "Calibration vel: ";
	for (int i = 0; i < 5; i++)
	  if (still_moving[i]) {
		int vel = new_state.joint[i].motorRPM.value;
		str = str + std::to_string(vel) + "RPM ";
		if (abs(vel)< 5) // if it is ~stopped, increase its counter
		  cycles_in_zero_speed[i]++;
		else
		  cycles_in_zero_speed[i] = 0; // if it is moving, null the counter
		// if it has stopped for 5 cycles, hold via small current
		if (cycles_in_zero_speed[i] >= 5) {
		  log(Log::info, "Joint " + std::to_string(i) + " at endposition");
		  cmd.commands[i] = { BLDCCommand::MOTOR_CURRENT_MA, backward[i] ?
			-holding_current_mA : holding_current_mA };
		  still_moving[i] = false; // change state
		}
	  }
	  else // if it has reached the holding state do nothing
		str = str + std::to_string(new_state.joint[i].q.value) + "rad ";
	log(Log::info, str);
	// if all joints just holding increase a counter - to let the hold current command start working
	if (still_moving[0] || still_moving[1] || still_moving[2]
	  || still_moving[3] || still_moving[4])
	  reached_since = 0;
	else
	  reached_since++;
	return cmd;
  }
  // If all joints are holding the endposition with 30mA
  // Start reference setting
  // (it does need a few milliseconds, we wait here still the returned positions become ~0)
  if (reached_since == 4) {
	for (int i = 0; i < 5; i++)
	  cmd.commands[i] = { BLDCCommand::ENCODER_SET_REFERENCE,0 };
	ref_setting_started = std::chrono::steady_clock::now();
	reached_since = 10;
	return cmd;
  }
  // Check if the joint references are set
  if (reached_since == 10) {
	bool allSet = true;
	std::string str;
	for (int i = 0; i < 5; i++) {
	  int motorticks = new_state.joint[i].motorticks.value;
	  if (abs(motorticks) > 3)
		allSet = false;
	  str = str + std::to_string(motorticks) + "motorticks ";
	}
	log(Log::info, str);
	int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
	  std::chrono::steady_clock::now() - ref_setting_started).count();
	if (elapsed_ms > 2000) {
	  // Save/send error messages
	  log(Log::fatal, "During calibration unsuccessful reference setting (>2000[ms])");
	  SLEEP_MILLISEC(2) // leave time to log...
		throw std::runtime_error("During calibration unsuccessful reference setting (>2000[ms])");
	}
	if (!allSet) {
	  SLEEP_MILLISEC(2) // leave time to log...
		return cmd;
	}
	log(Log::info, "Encoders nulled, elapsed time: " + std::to_string(elapsed_ms) + "[ms]");
  }
  // Set to idle:
  for (int i = 0; i < 5; i++)
	cmd.commands[i] = { BLDCCommand::MOTOR_VOLTAGE,0 };
  finished = true;
  return cmd;
}

MTask::TaskType youbot::MTaskCalibration::GetType() const {
    return CALIBRATION;
}

bool youbot::MTaskCalibration::_taskFinished() const {
  return finished;
}

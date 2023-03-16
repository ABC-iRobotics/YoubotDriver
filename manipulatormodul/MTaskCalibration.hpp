#ifndef CALIBRATE_MANIPULATOR_TASK_HPP
#define CALIBRATE_MANIPULATOR_TASK_HPP

#include "MTask.hpp"
#include "Config.hpp"

namespace youbot {
 /// <summary>
 /// Task that send out commutation initialization command and finishes
 ///  as all of the joints are initialized
 ///
 /// </summary>
  class MTaskCalibration : public MTask {
  public:
	void Initialize(const Config* config);  ///< initializes the task - necessary!

    ManipulatorCommand GetCommand(const JointsState& new_state) override;  ///< returns the current command

    TaskType GetType() const override; ///< returns the task type

  protected:
    bool _taskFinished() const override; ///< returns if the task has finished

	bool backward[5]; ///< initialized by Initialize()

	// Used during the process
	int reached_since = 0; // reached the limit position for x cycles 
	bool still_moving[5] = { true,true,true,true,true };
	int cycles_in_zero_speed[5] = { 0,0,0,0,0 }; //counters for ~0 speed, if achieves 5->still moving to zero
	std::chrono::steady_clock::time_point ref_setting_started;
	bool finished = false;
	bool initialized = false;
	ManipulatorCommand cmd;

	const double calJointRadPerSec = 0.2; ///< constant
	const int holding_current_mA = 30; ///< constant
  };
}
#endif
#ifndef CALIBRATE_MANIPULATOR_TASK_HPP
#define CALIBRATE_MANIPULATOR_TASK_HPP

#include "MTask.hpp"
#include "Eigen/dense"
#include "Logger.hpp"
#include "Config.hpp"

namespace youbot {
  /// <summary>
 /// Task that send out commutation initialization command and finishes as all of the joints are initialized
 /// 
 /// TODO!!
 /// </summary>
  class MTaskCalibration : public MTask {
	int reached_since = 0; // reached the limit position for x cycles 
	bool still_moving[5] = { true,true,true,true,true };
	int cycles_in_zero_speed[5] = { 0,0,0,0,0 }; //counters for ~0 speed, if achieves 5->still moving to zero



    const double calJointRadPerSec = 0.35;
    bool finished = false;

	bool directions[5] = { true,true,true,true,true };

	double _getSign(int i) const {
	  return 1;//or -1;
	}

	ManipulatorCommand cmd;

  public:
	MTaskCalibration(const Config* config) {
	  // Get directions..., get k motor_joint values...

	  for (int i = 0; i < 5; i++)
		cmd.commands[i] = { BLDCCommand::JOINT_VELOCITY,calJointRadPerSec * _getSign(i) };
	}

	//initialize


    ManipulatorCommand GetCommand(const JointsState& new_state) override {

      if (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started_at).count() < 200)
        return cmd;

	  if (reached_since) {
		// if dt>200ms, check if the joint has already stopped
		std::string str = "Calibration vel: ";
		for (int i = 0; i < 5; i++)
		  if (still_moving[i]) {
			int vel = new_state.joint[i].dq.value;
			str = str + std::to_string(vel) + "RPM ";
			if (vel<5 && vel>-5) // if it is ~stopped, increase its counter
			  cycles_in_zero_speed[i]++;
			else
			  cycles_in_zero_speed[i] = 0; // if it is moving, null the counter
			// if it has stopped for 5 cycles, hold via small current
			if (cycles_in_zero_speed[i] >= 5) {
			  log(Log::info, "Joint " + std::to_string(i) + " at endposition");
			  int holding_current = 30; //[mA]
			  cmd.commands[i] = { BLDCCommand::MOTOR_CURRENT_MA, _getSign(i) * holding_current };
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
	  }


      return cmd;
    }

    TaskType GetType() const override {
      return CALIBRATION;
    }

  protected:
    bool _taskFinished() const override {
      return false;
    }
  };
}
#endif
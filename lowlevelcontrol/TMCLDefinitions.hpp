#ifndef TMCL_DEFINITIONS_HPP
#define TMCL_DEFINITIONS_HPP

#include <string>


namespace youbot {
  namespace intrinsic {

	namespace TMCL {

	  enum class Module : uint8_t {
		DRIVE = 0,
		GRIPPER = 1
	  };

	  enum class Cmd : uint8_t {
		ROR = 1,  //Rotate right
		ROL = 2,  //Rotate left
		MST = 3,  //Motor stop
		MVP = 4,  //Move to position
		SAP = 5,  //Set axis parameter
		GAP = 6,  //Get axis parameter
		STAP = 7, //Store axis parameter into EEPROM
		RSAP = 8, //Restore axis parameter from EEPROM
		SGP = 9,  //Set global parameter
		GGP = 10, //Get global parameter
		STGP = 11, //Store global parameter into EEPROM
		RSGP = 12, //Restore global parameter from EEPROM
		RFS = 13,
		SIO = 14,
		GIO = 15,
		SCO = 30,
		GCO = 31,
		CCO = 32,
		CLE = 36, //ClearErrorFlags
		FIRMWARE_VERSION = 136
	  };

	  enum class AxisParam : uint8_t {
		TARGET_POSITION = 0,
		ACTUAL_POSITION = 1, // ok
		TARGET_SPEED = 2,
		ACTUAL_SPEED = 3,
		MAX_RAMP_VELOCITY = 4,
		MAX_CURRENT = 6,
		MAX_VEL_TO_REACH_TARGET = 7,
		TRESHOLD_SPEED_VEL_PID = 8,
		MOTOR_HALTED_VELOCITY = 9,
		MAX_DISTANCE_TO_REACH_TARGET = 10,
		ACCELERATION = 11,
		TRESHOLD_SPEED_POS_PID = 12,
		VEL_THRESHOLD_HALLFX = 14,
		INITIALIZE = 15,
		THERMAL_WINDING_TIME = 25, //cool down time too
		I2T_LIMIT_VALUE = 26,
		CURRENT_I2T_VALUE = 27,
		CLEAR_I2T_FLAG = 29,
		REINITIALIZE_ENCODER = 31,
		POSITION_PID_P1 = 130,
		POSITION_PID_I1 = 131,
		POSITION_PID_D1 = 132,
		POSITION_PID_I_CLIPPING1 = 135,
		VELOCITY_PID_P1 = 140,
		VELOCITY_PID_I1 = 141,
		VELOCITY_PID_D1 = 142,
		VELOCITY_PID_I_CLIPPING1 = 143,
		ACTIVATERAMP_IN_PID = 146,
		ACTUAL_MOTOR_CURRENT = 150,
		ACTUAL_SUPPLY_VOLTAGE = 151, // in 0.01V
		ACTUAL_TEMPERATURE = 152,
		TARGET_CURRENT = 155,
		ERROR_STATUS_FLAG = 156,
		CLEAR_MOTOR_CONTROLLER_TIMEOUT_FLAG = 158, // only in the drive..
		COMMUTATION_MODE = 159,
		CURRENT_PID_P1 = 168,
		CURRENT_PID_I1 = 169,
		CURRENT_PID_D1 = 170,
		CURRENT_PID_I_CLIPPING1 = 171,
		CURRENT_PID_P2 = 172,
		CURRENT_PID_I2 = 173,
		CURRENT_PID_D2 = 174,
		CURRENT_PID_I_CLIPPING2 = 175,
		POSITION_PID_P2 = 230,
		POSITION_PID_I2 = 231,
		POSITION_PID_D2 = 232,
		POSITION_PID_I_CLIPPING2 = 233,
		VELOCITY_PID_P2 = 234,
		VELOCITY_PID_I2 = 235,
		VELOCITY_PID_D2 = 236,
		VELOCITY_PID_I_CLIPPING2 = 237,
		ENCODER_STEPS_PER_ROTATION = 250,
		ENCODER_DIRECTION = 251
	  };

	  enum class ReplyStatus : uint8_t {
		NO_ERROR_ = 100,
		INVALID_COMMAND = 2,
		WRONG_TYPE = 3,
		INVALID_VALUE = 4,
		CONFIGURATION_EEPROM_LOCKED = 5,
		COMMAND_NOT_AVAILABLE = 6,
		REPLY_WRITE_PROTECTED = 8
	  };

	  std::string RecvStatusToString(ReplyStatus in);

	  enum class StatusErrorFlags : uint32_t {
		OVER_CURRENT = 0x1,
		UNDER_VOLTAGE = 0x2,
		OVER_VOLTAGE = 0x4,
		OVER_TEMPERATURE = 0x8,
		MOTOR_HALTED = 0x10,
		HALL_SENSOR_ERROR = 0x20,
		ENCODER_ERROR = 0x40,
		INITIALIZATION_ERROR = 0x80,
		PWM_MODE_ACTIVE = 0x100,
		VELOCITY_MODE_ACTIVE = 0x200,
		POSITION_MODE_ACTIVE = 0x400,
		TORQUE_MODE_ACTIVE = 0x800,
		EMERGENCY_STOP = 0x1000,
		FREERUNNING = 0x2000,
		POSITION_REACHED = 0x4000,
		INITIALIZED = 0x8000,
		TIMEOUT = 0x10000,
		I2T_EXCEEDED = 0x20000
	  };

	  std::string StatusErrorFlagsToString(uint32_t in);

	  enum class MotorBank : uint8_t {
		USER_VARIABLES = 2 //0..15: password protected, 16-. freely usable... 
	  };

	  enum UserVariables : uint8_t {
		NEED_CALIBRATION = 17,
		NEED_CONFIGURATION = 18
	  };

	  enum ControllerMode : uint8_t {
		MOTOR_STOP = 0,
		POSITION_CONTROL = 1,
		VELOCITY_CONTROL = 2,
		NO_MORE_ACTION = 3,
		SET_POSITION_TO_REFERENCE = 4,
		PWM_MODE = 5,
		CURRENT_MODE = 6,
		INITIALIZE = 7
	  };

	  std::string ControllerModeToString(ControllerMode mode);

	  enum class CommutationMode : uint8_t {
		BLOCK_BASED_ON_HALL = 0,
		SENSORLESS_BLOCK = 1,
		SINE_BASED_ON_HALL = 2,
		SINE_BASED_ON_ENCODER = 3,
		CONTROLLED_BLOCK = 4,
		CONTROLLED_SINE = 5
	  };
	  std::string CommutationMode2string(CommutationMode mode);
	  /*
	  enum GripperErrorFlags {
		STALL_GUARD_STATUS = 0x1,
		GRIPPER_OVER_TEMPERATURE = 0x2,
		PRE_WARNING_OVER_TEMPERATURE = 0x4,
		SHORT_TO_GROUND_A = 0x8,
		SHORT_TO_GROUND_B = 0x10,
		OPEN_LOAD_A = 0x20,
		OPEN_LOAD_B = 0x40,
		STAND_STILL = 0x80
	  };
	  */
	}

  }
}

#endif

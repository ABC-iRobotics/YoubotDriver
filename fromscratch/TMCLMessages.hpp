#ifndef TMCL_MESSAGES
#define TMCL_MESSAGES

#include "TMCLMessageTemplates.hpp"

// Does work:

class FirmWareRequest : public TMCLRequest {
public:
  FirmWareRequest(unsigned int slaveNumber);

  void GetOutput(long& controllernum, long& firmwareversion) const;
};

typedef TMCL_GetAxisParam<TMCLRequest::ACTUAL_POSITION> GetPosition;

typedef TMCL_SetAxisParam<TMCLRequest::ACTUAL_POSITION> SetEncoder;

typedef TMCL_GetAxisParam<TMCLRequest::ERROR_STATUS_FLAG> GetErrorStatusFlag;

class GetMotorControllerStatus : public TMCLRequest {
public:
  GetMotorControllerStatus(unsigned int slaveNumber);

  std::string StatusErrorFlagsAsString();

  using TMCLRequest::GetRecStatusFlag;
};






// Under development

typedef TMCL_GetAxisParam<TMCLRequest::INITIALIZE> GetIsUnderInitialization;

typedef TMCL_SetAxisParam<TMCLRequest::INITIALIZE> SetIsUnderInitialization;

class RestroreFromEEPROM : public TMCLRequest {
public:
  RestroreFromEEPROM(unsigned int slaveNumber) : TMCLRequest(slaveNumber) {
    _toModuleAddress = DRIVE;
    _toCommandNumber = RSAP;
    _toTypeNumber = TMCLRequest::POSITION_PID_P1;
    _toMotorNumber = 0;
    SetValue(0);
  }

  using TMCLRequest::GetRecStatusFlag;
};

//

class ClearErrorFlags : public TMCLRequest {
public:
  ClearErrorFlags(unsigned int slaveNumber) : TMCLRequest(slaveNumber) {
    _toModuleAddress = DRIVE;
    _toCommandNumber = CLE;
    _toTypeNumber = 0;
    _toMotorNumber = 0;
    SetValue(0);
  }
};

class ClearMotorControllerTimeoutFlag : public TMCLRequest {
public:
  ClearMotorControllerTimeoutFlag(unsigned int slaveNumber) : TMCLRequest(slaveNumber) {
    _toModuleAddress = DRIVE;
    _toCommandNumber = SAP;
    _toTypeNumber = TMCLRequest::CLEAR_MOTOR_CONTROLLER_TIMEOUT_FLAG;
    _toMotorNumber = 0;
    SetValue(1);
  }

  using TMCLRequest::GetRecStatusFlag;
};







typedef TMCL_OnlyCommand<TMCLRequest::MST> MotorStop;

typedef TMCL_CmdWithValue<TMCLRequest::MVP> MoveToPosition; // Absolute position, if type=1, rel, if type=2, then coord - according to set SCO





typedef TMCL_GetAxisParam<TMCLRequest::POSITION_PID_P2> GetP2ParameterPositionControl;



typedef TMCL_GetAxisParam<TMCLRequest::POSITION_PID_P1> GetP1ParameterPositionControl;

typedef TMCL_GetAxisParam<TMCLRequest::ENCODER_STEPS_PER_ROTATION> GetEncoderStepsPerRotation;

typedef TMCL_GetAxisParam<TMCLRequest::ENCODER_DIRECTION> GetEncoderDirection;

typedef TMCL_GetAxisParam<TMCLRequest::ACTUAL_MOTOR_CURRENT> GetCurrent;

typedef TMCL_GetAxisParam<TMCLRequest::MAX_CURRENT> GetMaxCurrent;


typedef TMCL_SetAxisParam<TMCLRequest::POSITION_PID_P1> SetPParameterFirstParametersPositionControl;

typedef TMCL_SetAxisParam<TMCLRequest::ENCODER_STEPS_PER_ROTATION> SetEncoderStepsPerRotation;

typedef TMCL_SetAxisParam<TMCLRequest::ENCODER_DIRECTION> SetEncoderDirection;


typedef TMCL_GetAxisParam<TMCLRequest::ENCODER_DIRECTION> GetEncoderDirection; // handled manually in the old
typedef TMCL_SetAxisParam<TMCLRequest::ENCODER_DIRECTION> SetEncoderDirection; // handled manually in the old

#endif

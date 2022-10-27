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





// Under development











typedef TMCL_OnlyCommand<TMCLRequest::MST> MotorStop;

typedef TMCL_CmdWithValue<TMCLRequest::MVP> MoveToPosition; // Absolute position, if type=1, rel, if type=2, then coord - according to set SCO








typedef TMCL_GetAxisParam<TMCLRequest::POSITION_PID_P1> GetPParameterFirstParametersPositionControl;


typedef TMCL_GetAxisParam<TMCLRequest::ENCODER_STEPS_PER_ROTATION> GetEncoderStepsPerRotation;

typedef TMCL_GetAxisParam<TMCLRequest::ENCODER_DIRECTION> GetEncoderDirection;

typedef TMCL_GetAxisParam<TMCLRequest::ACTUAL_MOTOR_CURRENT> GetCurrent;

typedef TMCL_GetAxisParam<TMCLRequest::MAX_CURRENT> GetMaxCurrent;




typedef TMCL_SetAxisParam<TMCLRequest::POSITION_PID_P1> SetPParameterFirstParametersPositionControl;

typedef TMCL_SetAxisParam<TMCLRequest::ENCODER_STEPS_PER_ROTATION> SetEncoderStepsPerRotation;

typedef TMCL_SetAxisParam<TMCLRequest::ENCODER_DIRECTION> SetEncoderDirection;

#endif

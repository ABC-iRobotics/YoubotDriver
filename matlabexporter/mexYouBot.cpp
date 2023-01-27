/* system header */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* MEX header */
#include <mex.h> 
#include "matrix.h"

#include "class_handle.hpp"

/* youBot headers */
#include "adapters.hpp"
#include "YoubotManipulator.hpp"
#include "Time.hpp"
#include "Logger.hpp"

using namespace youbot;

struct Manager {
  YoubotConfig config;

  YoubotManipulator* arm = NULL;

  Manager(const std::string& configFileName) : config(configFileName), arm(NULL) {}

  ~Manager() {
    if (arm)
      delete arm;
  }
};

/* MEX entry function */
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {
  if (nrhs == 0) {
    mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
      "Input arguments must be more than 0!");
  }
  double* mode = mxGetPr(prhs[0]);

  // Initialization (0,configfilename)
  if (*mode == 0) {
    if (nrhs != 2) {
      mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
        "Input arguments must be 2 in inintialization (mode==0) mode!");
    }
    char input_buf[5000];
    mxGetString(prhs[1], input_buf, 5000);
    auto manager = new Manager(input_buf);
    // Initialize logger
    Log::Setup(manager->config.logConfig);

    // Check parameters
    if (nlhs != 1) {
      mexErrMsgTxt("New: One output expected.");
      return;
    }
    // Return a handle to a new C++ instance
    plhs[0] = convertPtr2Mat<Manager>(manager);
    return;
  }

  // Setup youbotArm (2,ptr)
  if (*mode == 2) {
    if (nrhs != 2) {
      mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
        "Input arguments must be 2 in find arm (mode==0.5) mode!");
    }
    Manager* manager = convertMat2Ptr<Manager>(prhs[1]);

    // Find appropriate ethernet adapter and open connection
    if (!EtherCATMaster::GetSingleton()->isOpened()) {
      char name[1000];
      if (findYouBotEtherCatAdapter(name))
        log(Log::info, "Adapter found:" + std::string(name));
      else {
        log(Log::fatal, "Adapter with turned on youBot arm NOT found!");
        mexErrMsgTxt("Adapter with turned on youBot arm NOT found!");
      }
      if (!EtherCATMaster::GetSingleton()->OpenConnection(name))
        mexErrMsgTxt("EtherCATMaster::GetSingleton()->OpenConnection was not successful");
    }

    if (!manager->arm)
      manager->arm = new YoubotManipulator(manager->config, EtherCATMaster::GetSingleton());
    manager->arm->InitializeManipulator();
    manager->arm->Calibrate();
    // Check parameters
    if (nlhs != 0)
      mexErrMsgTxt("Find arm: No output expected.");
    return;
  }

  // Delete (1,ptr)
  if (*mode == 1) { 
    if (nrhs != 2) {
      mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
        "Input arguments must be 2 in delete (mode==2) mode!");
    }
    EtherCATMaster::GetSingleton()->StopProcessThread();
    destroyObject<Manager>(prhs[1]);
    EtherCATMaster::GetSingleton()->CloseConnection();
    Log::DropLogger();
    // Check parameters
    if (nlhs != 0)
      mexErrMsgTxt("Delete: Zero output expected.");
  }

  // Start process thread (5,ptr,Ts)
  if (*mode == 5) {
    if (nrhs != 3) {
      mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
        "Input arguments must be 3 in start process msgs (mode==5) mode!");
    }
    Manager* man = convertMat2Ptr<Manager>(prhs[1]);
    if (!man->arm)
      mexErrMsgTxt("Arm not initialized.");
    SLEEP_SEC(1);
    man->arm->CheckAndResetErrorFlags();
    double* Ts = mxGetPr(prhs[2]);
    EtherCATMaster::GetSingleton()->StartProcessThread(*Ts >= 0 ? (int)*Ts : 0);
    // Check parameters
    if (nlhs != 0)
      mexErrMsgTxt("Start process thread: Zero output expected.");
    return;
  }

  // Set Manipulator joints (6,ptr,q)
  if (*mode == 6) {
    if (nrhs != 3) {
      mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
        "Input arguments must be 3 in set joints (mode==6) mode!");
    }
    Manager* man = convertMat2Ptr<Manager>(prhs[1]);
    if (!man->arm)
      mexErrMsgTxt("Arm not initialized.");
    
    if (mxGetN(prhs[2])*mxGetM(prhs[2]) != 5) {
      mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
        "Second input argument must be a vector of length 5 in set joints mode!");
    }

    double* q = mxGetPr(prhs[2]);
    man->arm->ReqJointPositionRad(q[0] / 180. * M_PI,q[1] / 180. * M_PI,q[2] / 180. * M_PI,q[3] / 180. * M_PI,q[4] / 180. * M_PI);

    // Check parameters
    if (nlhs != 0)
      mexErrMsgTxt("Start process thread: Zero output expected.");
    return;
  }

  // Stop process thread (7)
  if (*mode == 7) {
    if (nrhs != 2) {
      mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
        "Input arguments must be 2 in stop process msgs (mode==7) mode!");
    }
    EtherCATMaster::GetSingleton()->StopProcessThread();
    // Check parameters
    if (nlhs != 0)
      mexErrMsgTxt("Start process thread: Zero output expected.");
    return;
  }

  // Get Manipulator joint variables (8,ptr)
  if (*mode == 8) {
    if (nrhs != 2) {
      mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
        "Input arguments must be 2 in get joints (mode==8) mode!");
    }
    Manager* man = convertMat2Ptr<Manager>(prhs[1]);
    if (!man->arm)
      mexErrMsgTxt("Arm not initialized.");
    // Check parameters
    if (nlhs == 1) {
      plhs[0] = mxCreateDoubleMatrix(5, 1, mxREAL);
      double* q = mxGetPr(plhs[0]);
      man->arm->GetQLatest(q[0], q[1], q[2], q[3], q[4]);
      for (int i = 0; i < 5; i++)
        q[i] *= 180. / M_PI;
      return;
    }
    if (nlhs == 0)
      return;
    mexErrMsgTxt("Get jointvariables: One output expected.");
    return;
  }
}

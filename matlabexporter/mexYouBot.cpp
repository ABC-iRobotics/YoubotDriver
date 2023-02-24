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
#include "Manager.hpp"
#include "Logger.hpp"
#include "Time.hpp"
#include "MTaskRawConstantJointSpeed.hpp"
#include "MTaskCommutation.hpp"
#include "MTaskCalibration.hpp"

using namespace youbot;

/* MEX entry function */
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {
  try {
    if (nrhs == 0) {
      mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
        "Input arguments must be more than 0!");
    }
    double* mode = mxGetPr(prhs[0]);

    // Initialization (0,configfilename, isvirtual)
    if (*mode == 0) {
      if (nrhs != 3)
        mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
          "Input arguments must be 3 in inintialization (mode==0) mode!");
      if (nlhs != 1)
        mexErrMsgTxt("Delete: Zero output expected.");
      char input_buf[5000];
      mxGetString(prhs[1], input_buf, 5000);
      bool isvirtual = *mxGetPr(prhs[2]) > 0;
      auto modul = new Manager(input_buf, isvirtual);

      // Return a handle to a new C++ instance
      plhs[0] = convertPtr2Mat<Manager>(modul);
      return;
    }

    // Delete (1,ptr)
    if (*mode == 1) {
      if (nrhs != 2)
        mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
          "Input arguments must be 2 in delete (mode==2) mode!");
      if (nlhs != 0)
        mexErrMsgTxt("Delete: Zero output expected.");
      destroyObject<Manager>(prhs[1]);
      Log::DropLogger();
      return;
    }

    // Start modul thread (5,ptr)
    if (*mode == 5) {
      if (nrhs != 2)
        mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
          "Input arguments must be 2 in start process msgs (mode==5) mode!");
      if (nlhs != 0)
        mexErrMsgTxt("Start process thread: Zero output expected.");
      auto modul = convertMat2Ptr<Manager>(prhs[1]);
      modul->StartThreadAndInitialize();
      return;
    }

    // Stop modul thread (7,ptr)
    if (*mode == 7) {
      if (nrhs != 2)
        mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
          "Input arguments must be 2 in stop process msgs (mode==7) mode!");
      if (nlhs != 0)
        mexErrMsgTxt("Stop process thread: Zero output expected.");
      auto modul = convertMat2Ptr<Manager>(prhs[1]);
      modul->StopThread();
      return;
    }

    // New Manipulator task (6,ptr,params,type, tlimit)
    if (*mode == 6) {
      if (nrhs != 5)
        mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
          "Input arguments must be 4 in new task (mode==6) mode!");
      if (nlhs != 0)
        mexErrMsgTxt("New task: Zero output expected.");
      auto modul = convertMat2Ptr<Manager>(prhs[1]);
      int N = mxGetN(prhs[2]) * mxGetM(prhs[2]);
      double* p = mxGetPr(prhs[2]);
      Eigen::VectorXd param(N);
      for (int n = 0; n < N; n++)
        param[n] = p[n];
      MTask::Ptr task;
      int type = (int)*mxGetPr(prhs[3]);
      double tlimit = *mxGetPr(prhs[4]);
      switch (type) {
      case 0:
        task = std::make_shared<IdleManipulatorTask>();
        break;
      case 1:
        task = std::make_shared<MTaskRawConstantJointSpeed>(param / 180. * M_PI, tlimit);
        break;
      case 2:
        task = std::make_shared<MTaskZeroCurrent>();
        break;
      case 3:
        task = std::make_shared<MTaskCommutation>();
        break;
      case 4:
        task = std::make_shared<MTaskCalibration>();
        break;
      }
      modul->NewManipulatorTask(task, tlimit);
      return;
    }

    // Get full status of Manipulator (9,ptr)
    if (*mode == 9) {
      if (nrhs != 2)
        mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
          "Input arguments must be 2 in get full status (mode==9) mode!");
      auto modul = convertMat2Ptr<Manager>(prhs[1]);
      auto out = modul->GetStatus();
      // Check parameters
      if (nlhs >= 1) {
        plhs[0] = mxCreateDoubleMatrix(5, 1, mxREAL);
        double* q = mxGetPr(plhs[0]);
        for (int i = 0; i < 5; i++)
          q[i] = out.joint[i].q.value * 180. / M_PI;
      }
      if (nlhs >= 2) {
        plhs[1] = mxCreateDoubleMatrix(5, 1, mxREAL);
        double* dq = mxGetPr(plhs[1]);
        for (int i = 0; i < 5; i++)
          dq[i] = out.joint[i].dq.value * 180. / M_PI;
      }
      if (nlhs >= 3) {
        plhs[2] = mxCreateDoubleMatrix(5, 1, mxREAL);
        double* tau = mxGetPr(plhs[2]);
        for (int i = 0; i < 5; i++)
          tau[i] = out.joint[i].tau.value;
      }
      if (nlhs == 4) {
        plhs[3] = mxCreateDoubleMatrix(1, 1, mxREAL);
        double* mode = mxGetPr(plhs[3]);
        switch (out.motion) {
        case MTask::COMMUTATION:
          *mode = 0;
          break;
        case MTask::STOPPED:
          *mode = 1;
          break;
        case MTask::RAW_CONSTANT_JOINTSPEED:
          *mode = 2;
          break;
        default:
          *mode = 10;
        }
      }
      if (nlhs > 4)
        mexErrMsgTxt("Get jointvariables: Four or less outputs expected.");
      return;
    }

    // Get true Manipulator joint variables (8,ptr)
    if (*mode == 8) {
      if (nrhs != 2)
        mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
          "Input arguments must be 2 in get true joints (mode==8) mode!");
      auto modul = convertMat2Ptr<Manager>(prhs[1]);
      auto out = modul->GetTrueStatus();
      // Check parameters
      if (nlhs == 1) {
        plhs[0] = mxCreateDoubleMatrix(5, 1, mxREAL);
        double* q = mxGetPr(plhs[0]);
        for (int i = 0; i < 5; i++)
          q[i] = out[i] * 180. / M_PI;
        return;
      }
      if (nlhs == 0)
        return;
      mexErrMsgTxt("Get jointvariables: One output expected.");
      return;
    }

    mexErrMsgIdAndTxt("MATLAB:youBot:inputmismatch",
      "mode not implemented");
  }
  catch (const std::runtime_error& error) {
    SLEEP_MILLISEC(200); // let the logger do its job...
    mexErrMsgIdAndTxt("MATLAB:youBot:runtime_error",
      error.what());
  }
}

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP


//#include <string>
//#include "Time.hpp"
#include <chrono>

namespace youbot {

/***************************************************/
/*! \class PidController
    \brief A basic pid class.

    This class implements a generic structure that
    can be used to create a wide range of pid
    controllers. It can function independently or
    be subclassed to provide more specific controls
    based on a particular control loop.

    In particular, this class implements the standard
    pid equation:

    \f$command  = -p_{term} - i_{term} - d_{term} \f$

    where: <br>
    <UL TYPE="none">
    <LI>  \f$ p_{term}  = p_{gain} * p_{error} \f$
    <LI>  \f$ i_{term}  = i_{gain} * i_{error} \f$
    <LI>  \f$ d_{term}  = d_{gain} * d_{error} \f$
    <LI>  \f$ i_{error} = i_{error} + p_{error} * dt \f$
    <LI>  \f$ d_{error} = (p_{error} - p_{error last}) / dt \f$
    </UL>

    given:<br>
    <UL TYPE="none">
    <LI>  \f$ p_{error}  = p_{state} - p_{target} \f$.
    </UL>


    @section Usage

    To use the Pid class, you should first call some version of init()
    (in non-realtime) and then call updatePid() at every update step.
    For example:

\verbatim
control_toolbox::Pid pid;
pid.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
double position_desi_ = 0.5;
...
ros::Time last_time = ros::Time::now();
while (true) {
  ros::Time time = ros::Time::now();
  double effort = pid.updatePid(currentPosition() - position_desi_, time - last_time);
  last_time = time;
}
\endverbatim

*/
/***************************************************/


class PidController
{
public:

  /*!
   * \brief Constructor, zeros out Pid values when created and
   * initialize Pid-gains and integral term limits:[iMax:iMin]-[I1:I2].
   *
   * \param P  The proportional gain.
   * \param I  The integral gain.
   * \param D  The derivative gain.
   * \param I1 The integral upper limit.
   * \param I2 The integral lower limit.
   */
  PidController(double P = 0.0, double I = 0.0, double D = 0.0, double I1 = 0.0, double I2 = -0.0);

  /*!
   * \brief Destructor of Pid class.
   */
  ~PidController();

  /*!
   * \brief Update the Pid loop with nonuniform time step size.
   *
   * \param p_error  Error since last call (p_state-p_target)
   * \param dt Change in time since last call
   */
  template<class Rep, class Period>
  double updatePid(double p_error, std::chrono::duration<Rep, Period> dt);

  /*!
   * \brief Initialize PID-gains and integral term limits:[iMax:iMin]-[I1:I2]
   *
   * \param P  The proportional gain.
   * \param I  The integral gain.
   * \param D  The derivative gain.
   * \param I1 The integral upper limit.
   * \param I2 The integral lower limit.
   */
  void initPid(double P, double I, double D, double I1, double I2);
  
  /*!
   * \brief Reset the state of this PID controller
   */
  void reset();

  /*!
   * \brief Set current command for this PID controller
   */
  void setCurrentCmd(double cmd);

  /*!
   * \brief Return current command for this PID controller
   */
  double getCurrentCmd();

  /*!
   * \brief Return PID error terms for the controller.
   * \param pe  The proportional error.
   * \param ie  The integral error.
   * \param de  The derivative error.
   */
  void getCurrentPIDErrors(double& pe, double& ie, double& de);

  /*!
   * \brief Set PID gains for the controller.
   * \param P  The proportional gain.
   * \param I  The integral gain.
   * \param D  The derivative gain.
   * \param i_max
   * \param i_min
   */
  void setGains(double P, double I, double D, double i_max, double i_min);

  /*!
   * \brief Get PID gains for the controller.
   * \param p  The proportional gain.
   * \param i  The integral gain.
   * \param d  The derivative gain.
   * \param i_max The max integral windup.
   * \param i_mim The min integral windup.
   */
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  /*!
   * \brief Update the Pid loop with nonuniform time step size. This update call 
   * allows the user to pass in a precomputed derivative error. 
   *
   * \param error  Error since last call (p_state-p_target)
   * \param error_dot d(Error)/dt since last call
   * \param dt Change in time since last call
   */
  template<class Rep, class Period>
  double updatePid(double error, double error_dot, std::chrono::duration<Rep, Period> dt);

  PidController& operator =(const PidController& p);

private:
  double p_error_last_; /**< _Save position state for derivative state calculation. */
  double p_error_; /**< Position error. */
  double d_error_; /**< Derivative error. */
  double i_error_; /**< Integator error. */
  double p_gain_;  /**< Proportional gain. */
  double i_gain_;  /**< Integral gain. */
  double d_gain_;  /**< Derivative gain. */
  double i_max_;   /**< Maximum allowable integral term. */
  double i_min_;   /**< Minimum allowable integral term. */
  double cmd_;     /**< Command to send. */
  double last_i_error;
};


/*!
* \brief Update the Pid loop with nonuniform time step size. This update call
* allows the user to pass in a precomputed derivative error.
*
* \param error  Error since last call (p_state-p_target)
* \param error_dot d(Error)/dt since last call
* \param dt Change in time since last call
*/

template<class Rep, class Period>
inline double PidController::updatePid(double p_error, std::chrono::duration<Rep, Period> dt) {
  double p_term, d_term, i_term;
  p_error_ = error; //this is pError = pState-pTarget
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(dt);
  double deltatime = (double)us.count() / 1000.0;  //in milli seconds

  if (deltatime == 0.0 || isnan(error) || isinf(error))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = p_gain_ * p_error_;

  // Calculate the integral error

  i_error_ = last_i_error + deltatime * p_error_;
  last_i_error = deltatime * p_error_;

  //Calculate integral contribution to command
  i_term = i_gain_ * i_error_;

  // Limit i_term so that the limit is meaningful in the output
  if (i_term > i_max_)
  {
    i_term = i_max_;
    i_error_ = i_term / i_gain_;
  }
  else if (i_term < i_min_)
  {
    i_term = i_min_;
    i_error_ = i_term / i_gain_;
  }

  // Calculate the derivative error
  if (deltatime != 0)
  {
    d_error_ = (p_error_ - p_error_last_) / deltatime;
    p_error_last_ = p_error_;
  }
  // Calculate derivative contribution to command
  d_term = d_gain_ * d_error_;
  cmd_ = -p_term - i_term - d_term;

  // printf(" p_error_ %lf  i_error_ %lf  p_term %lf i_term %lf  dt %lf out %lf\n", p_error_, i_error_, p_term, i_term, deltatime, cmd_);

  return cmd_;
}

template<class Rep, class Period>
inline double PidController::updatePid(double error, double error_dot, std::chrono::duration<Rep, Period> dt) {
  double p_term, d_term, i_term;
  p_error_ = error; //this is pError = pState-pTarget
  d_error_ = error_dot;
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(dt);
  double deltatime = (double)us.count() / 1000.0;  //in milli seconds

  if (deltatime == 0.0 || isnan(error) || isinf(error) || isnan(error_dot) || isinf(error_dot))
    return 0.0;


  // Calculate proportional contribution to command
  p_term = p_gain_ * p_error_;

  // Calculate the integral error
  i_error_ = last_i_error + deltatime * p_error_;
  last_i_error = deltatime * p_error_;

  // i_error_ = i_error_ + deltatime * p_error_;
  //   printf("i_error_ %lf dt.fractional_seconds() %lf\n", i_error_, deltatime);

  //Calculate integral contribution to command
  i_term = i_gain_ * i_error_;

  // Limit i_term so that the limit is meaningful in the output
  if (i_term > i_max_)
  {
    i_term = i_max_;
    i_error_ = i_term / i_gain_;
  }
  else if (i_term < i_min_)
  {
    i_term = i_min_;
    i_error_ = i_term / i_gain_;
  }

  // Calculate derivative contribution to command
  d_term = d_gain_ * d_error_;
  cmd_ = -p_term - i_term - d_term;

  return cmd_;
}

}

#endif

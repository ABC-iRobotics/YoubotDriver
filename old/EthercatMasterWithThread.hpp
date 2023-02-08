#ifndef YOUBOT_ETHERCATMASTERWITHTHREAD_H
#define YOUBOT_ETHERCATMASTERWITHTHREAD_H

/****************************************************************
 *
 * Copyright (c) 2011
 * All rights reserved.
 *
 * Hochschule Bonn-Rhein-Sieg
 * University of Applied Sciences
 * Computer Science Department
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author:
 * Jan Paulus, Nico Hochgeschwender, Michael Reckhaus, Azamat Shakhimardanov
 * Supervised by:
 * Gerhard K. Kraetzschmar
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This sofware is published under a dual-license: GNU Lesser General Public 
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Hochschule Bonn-Rhein-Sieg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ****************************************************************/
#include <vector>
#include <sstream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "DataObjectLockFree.hpp"
#include "Logger.hpp"
#include "Units.hpp"
#include "Time.hpp"
#include "Exceptions.hpp"
#include "TMCLProtocolDefinitions.hpp"
#include "YouBotSlaveMsg.hpp"
#include "YouBotSlaveMailboxMsg.hpp"
#include "EthercatMaster.hpp"
#include "JointTrajectoryController.hpp"
#include "JointLimitMonitor.hpp"

extern "C"{
#include <ethercattype.h>
#include <ethercatmain.h>
}

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// The Ethercat Master is managing the whole ethercat communication 
/// It have to be a singleton in the system
///////////////////////////////////////////////////////////////////////////////
class EthercatMasterWithThread : public EthercatMasterInterface {
friend class EthercatMaster;
friend class YouBotJoint;
friend class YouBotGripper;
friend class YouBotGripperBar;
  private:
    EthercatMasterWithThread(const std::string& configFile, const std::string& configFilePath);

    ~EthercatMasterWithThread();

  public:
    bool isThreadActive();

    void AutomaticSendOn(const bool enableAutomaticSend);

    void AutomaticReceiveOn(const bool enableAutomaticReceive);

    ///sends ethercat messages to the motor controllers
    /// returns a true if everything it OK and returns false if something fail
    bool sendProcessData();

    /// receives ethercat messages from the motor controllers
    /// returns a true if everything it OK and returns false if something fail
    bool receiveProcessData();

    void registerJointTrajectoryController(JointTrajectoryController* object, const unsigned int JointNumber);

    void deleteJointTrajectoryControllerRegistration(const unsigned int JointNumber);

    unsigned int getNumberOfThreadCyclesPerSecond();

    void registerJointLimitMonitor(JointLimitMonitor* object, const unsigned int JointNumber);

    void registerDataTrace(void* object, const unsigned int JointNumber);

    void deleteDataTraceRegistration(const unsigned int JointNumber);

  private:
    ///stores a ethercat message to the buffer
    ///@param msgBuffer ethercat message
    ///@param jointNumber joint number of the sender joint
    void setMsgBuffer(const YouBotSlaveMsg& msgBuffer, const unsigned int jointNumber);

    ///get a ethercat message form the buffer
    ///@param msgBuffer ethercat message
    ///@param jointNumber joint number of the receiver joint
    void getMsgBuffer(const unsigned int jointNumber, YouBotSlaveMsg& returnMsg);

    ///stores a mailbox message in a buffer which will be sent to the motor controllers
    ///@param msgBuffer ethercat mailbox message
    ///@param jointNumber joint number of the sender joint
    void setMailboxMsgBuffer(const YouBotSlaveMailboxMsg& msgBuffer, const unsigned int jointNumber);

    ///gets a mailbox message form the buffer which came form the motor controllers
    ///@param msgBuffer ethercat mailbox message
    ///@param jointNumber joint number of the receiver joint
    bool getMailboxMsgBuffer(YouBotSlaveMailboxMsg& mailboxMsg, const unsigned int jointNumber);

    ///sends and receives ethercat messages and mailbox messages to and from the motor controllers
    ///this method is executed in a separate thread
    void updateSensorActorValues();

    ec_mbxbuft mailboxBuffer;

    std::vector<YouBotSlaveMsgThreadSafe> slaveMessages;

    //in microseconds
    unsigned int timeTillNextEthercatUpdate;

    boost::thread_group threads;

    volatile bool stopThread;

    std::vector<YouBotSlaveMsg> automaticSendOffBufferVector;

    std::vector<YouBotSlaveMsg> automaticReceiveOffBufferVector;

    std::vector<YouBotSlaveMailboxMsgThreadSafe> mailboxMessages;

    std::vector<bool> newInputMailboxMsgFlag;

    std::vector<bool> outstandingMailboxMsgFlag;

    std::vector<bool> pendingMailboxMsgsReply;

    bool automaticSendOn;

    bool automaticReceiveOn;

    long int communicationErrors;

    long int maxCommunicationErrors;

    std::vector<JointTrajectoryController*> trajectoryControllers;

    boost::mutex trajectoryControllerVectorMutex;

    std::vector<JointLimitMonitor*> jointLimitMonitors;

    boost::mutex jointLimitMonitorVectorMutex;

    std::vector<void*> dataTraces;

    boost::mutex dataTracesMutex;
};

} // namespace youbot
#endif

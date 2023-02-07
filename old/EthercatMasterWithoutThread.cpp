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
extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "ethercatprint.h"
}
#include "EthercatMasterWithoutThread.hpp"

namespace youbot {

EthercatMasterWithoutThread::EthercatMasterWithoutThread(const std::string& configFile, const std::string& configFilePath) :
  EthercatMasterInterface(configFile, configFilePath) {

  initialize();
  
  YouBotSlaveMsg emptySlaveMsg;
  for (auto it : identified_slaves) {
    processDataBuffer.push_back(emptySlaveMsg);
    YouBotSlaveMailboxMsg emptyMailboxSlaveMsg(it);
    firstMailboxBufferVector.push_back(emptyMailboxSlaveMsg);
  }
}

EthercatMasterWithoutThread::~EthercatMasterWithoutThread() {
  // Bouml preserved body begin 000D1BF1
    closeEthercat();
    if (configfile != NULL)
      delete configfile;
  // Bouml preserved body end 000D1BF1
}

bool EthercatMasterWithoutThread::isThreadActive() {
  // Bouml preserved body begin 000E6B71
  return false;
  // Bouml preserved body end 000E6B71
}

void EthercatMasterWithoutThread::AutomaticSendOn(const bool enableAutomaticSend) {
  // Bouml preserved body begin 000D1DF1
    LOG(trace) << "automatic send is not possible if the EtherCAT master has no thread";

    return;
  // Bouml preserved body end 000D1DF1
}

void EthercatMasterWithoutThread::AutomaticReceiveOn(const bool enableAutomaticReceive) {
  // Bouml preserved body begin 000D1E71
    LOG(trace) << "automatic receive is not possible if the EtherCAT master has no thread";
    return;
  // Bouml preserved body end 000D1E71
}

///sends ethercat messages to the motor controllers
/// returns a true if everything it OK and returns false if something fail
bool EthercatMasterWithoutThread::sendProcessData() {
  // Bouml preserved body begin 000D2471

    for (unsigned int i = 0; i < processDataBuffer.size(); i++) {
      //fill output buffer (send data)
      *(ethercatOutputBufferVector[i]) = (processDataBuffer[i]).stctOutput;
    }

    //send data to ethercat
    if (ec_send_processdata() == 0) {
      return false;
    }
    
    return true;

  // Bouml preserved body end 000D2471
}

/// receives ethercat messages from the motor controllers
/// returns a true if everything it OK and returns false if something fail
bool EthercatMasterWithoutThread::receiveProcessData() {
  // Bouml preserved body begin 000D5D71

    //receive data from ethercat
    if (ec_receive_processdata(this->ethercatTimeout) == 0) {
      return false;
    }

    for (unsigned int i = 0; i < processDataBuffer.size(); i++) {
      //fill input buffer (receive data)
      (processDataBuffer[i]).stctInput = *(ethercatInputBufferVector[i]);
    }

    return true;

  // Bouml preserved body end 000D5D71
}

void EthercatMasterWithoutThread::registerJointLimitMonitor(JointLimitMonitor* object, const unsigned int JointNumber) {
  // Bouml preserved body begin 000FB0F1

  // Bouml preserved body end 000FB0F1
}

///stores a ethercat message to the buffer
///@param msgBuffer ethercat message
///@param jointNumber joint number of the sender joint
void EthercatMasterWithoutThread::setMsgBuffer(const YouBotSlaveMsg& msgBuffer, const unsigned int jointNumber) {
  // Bouml preserved body begin 000D20F1

    processDataBuffer[jointNumber - 1].stctOutput = msgBuffer.stctOutput;

  // Bouml preserved body end 000D20F1
}

///get a ethercat message form the buffer
///@param msgBuffer ethercat message
///@param jointNumber joint number of the receiver joint
void EthercatMasterWithoutThread::getMsgBuffer(const unsigned int jointNumber, YouBotSlaveMsg& returnMsg) {
  // Bouml preserved body begin 000D2171
  
    returnMsg = processDataBuffer[jointNumber - 1];
    
  // Bouml preserved body end 000D2171
}

///stores a mailbox message in a buffer which will be sent to the motor controllers
///@param msgBuffer ethercat mailbox message
///@param jointNumber joint number of the sender joint
void EthercatMasterWithoutThread::setMailboxMsgBuffer(const YouBotSlaveMailboxMsg& msgBuffer, const unsigned int jointNumber) {
  // Bouml preserved body begin 000D21F1
  
    firstMailboxBufferVector[jointNumber - 1].stctOutput = msgBuffer.stctOutput;
    sendMailboxMessage(firstMailboxBufferVector[jointNumber - 1]);
    return;
  // Bouml preserved body end 000D21F1
}

///gets a mailbox message form the buffer which came form the motor controllers
///@param msgBuffer ethercat mailbox message
///@param jointNumber joint number of the receiver joint
bool EthercatMasterWithoutThread::getMailboxMsgBuffer(YouBotSlaveMailboxMsg& mailboxMsg, const unsigned int jointNumber) {
  // Bouml preserved body begin 000D2271
    bool returnvalue = receiveMailboxMessage(firstMailboxBufferVector[jointNumber - 1]);
    mailboxMsg.stctInput = firstMailboxBufferVector[jointNumber - 1].stctInput;
    return returnvalue;
  // Bouml preserved body end 000D2271
}
} // namespace youbot

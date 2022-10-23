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
#include "EthercatMasterWithThread.hpp"
#include "DataTrace.hpp"

namespace youbot {

EthercatMasterWithThread::EthercatMasterWithThread(const std::string& configFile, const std::string& configFilePath) :
  EthercatMasterInterface(configFile, configFilePath) {
  // Bouml preserved body begin 00041171
    timeTillNextEthercatUpdate = 1000; //usec
    communicationErrors = 0;
    maxCommunicationErrors = 100;
    stopThread = false;
    this->automaticSendOn = true;
    this->automaticReceiveOn = true;
    configfile->readInto(timeTillNextEthercatUpdate, "EtherCAT", "EtherCATUpdateRate_[usec]");
    configfile->readInto(maxCommunicationErrors, "EtherCAT", "MaximumNumberOfEtherCATErrors");

    initialize();
    
    YouBotSlaveMsgThreadSafe emptySlaveMsgThreadSafe;
    for (auto it: identified_slaves) {
        YouBotSlaveMailboxMsgThreadSafe emptyMailboxSlaveMsg(it);
        mailboxMessages.push_back(emptyMailboxSlaveMsg);
        pendingMailboxMsgsReply.push_back(false);
        trajectoryControllers.push_back(NULL);
        jointLimitMonitors.push_back(NULL);
        slaveMessages.push_back(emptySlaveMsgThreadSafe);
        outstandingMailboxMsgFlag.push_back(false);
        newInputMailboxMsgFlag.push_back(false);
        dataTraces.push_back(NULL);
    }

    automaticReceiveOffBufferVector.reserve(nrOfSlaves);
    stopThread = false;
    threads.create_thread(boost::bind(&EthercatMasterWithThread::updateSensorActorValues, this));

    SLEEP_MILLISEC(10); //needed to start up thread and EtherCAT communication

    this->ethercatConnectionEstablished = true;
}

EthercatMasterWithThread::~EthercatMasterWithThread() {
  // Bouml preserved body begin 000411F1
    stopThread = true;
    threads.join_all();
    closeEthercat();
    if (configfile != NULL)
      delete configfile;
  // Bouml preserved body end 000411F1
}

bool EthercatMasterWithThread::isThreadActive() {
  // Bouml preserved body begin 000E6AF1
    return true;
  // Bouml preserved body end 000E6AF1
}

void EthercatMasterWithThread::AutomaticSendOn(const bool enableAutomaticSend) {
  // Bouml preserved body begin 000775F1
    this->automaticSendOn = enableAutomaticSend;

    if (this->automaticSendOn == true) {
      unsigned int slaveNo = 0;

      for (unsigned int i = 0; i < automaticSendOffBufferVector.size(); i++) {
        slaveNo = automaticSendOffBufferVector[i].jointNumber - 1;
        slaveMessages[slaveNo].stctInput.Set(automaticSendOffBufferVector[i].stctInput);
        slaveMessages[slaveNo].stctOutput.Set(automaticSendOffBufferVector[i].stctOutput);
        slaveMessages[slaveNo].jointNumber.Set(automaticSendOffBufferVector[i].jointNumber);
      }

      automaticSendOffBufferVector.clear();
    } else {
      return;
    }
    return;
  // Bouml preserved body end 000775F1
}

void EthercatMasterWithThread::AutomaticReceiveOn(const bool enableAutomaticReceive) {
  // Bouml preserved body begin 0008FB71
    this->automaticReceiveOn = enableAutomaticReceive;


    if (this->automaticReceiveOn == false) {
      

      for (unsigned int i = 0; i < slaveMessages.size(); i++) {
        slaveMessages[i].stctInput.Get(automaticReceiveOffBufferVector[i].stctInput);
        slaveMessages[i].stctOutput.Get(automaticReceiveOffBufferVector[i].stctOutput);
        slaveMessages[i].jointNumber.Get(automaticReceiveOffBufferVector[i].jointNumber);
      }
    }

    return;
  // Bouml preserved body end 0008FB71
}

///sends ethercat messages to the motor controllers
/// returns a true if everything it OK and returns false if something fail
bool EthercatMasterWithThread::sendProcessData() {
  // Bouml preserved body begin 000E68F1
    throw std::runtime_error("When using the EthercatMaster with thread there is not need to send process data manual.");
    return false;
  // Bouml preserved body end 000E68F1
}

/// receives ethercat messages from the motor controllers
/// returns a true if everything it OK and returns false if something fail
bool EthercatMasterWithThread::receiveProcessData() {
  // Bouml preserved body begin 000E6971
    throw std::runtime_error("When using the EthercatMaster with thread there is not need to receive process data manual");
    return false;
  // Bouml preserved body end 000E6971
}

void EthercatMasterWithThread::registerJointTrajectoryController(JointTrajectoryController* object, const unsigned int JointNumber) {
  // Bouml preserved body begin 000EBCF1
    {
      boost::mutex::scoped_lock trajectoryControllerMutex(trajectoryControllerVectorMutex);
      if (this->trajectoryControllers[JointNumber - 1] != NULL)
        throw std::runtime_error("A joint trajectory controller is already register for this joint!");
      if ((JointNumber - 1) >= this->trajectoryControllers.size())
        throw std::out_of_range("Invalid joint number");

      this->trajectoryControllers[JointNumber - 1] = object;
    }
    LOG(debug) << "register joint trajectory controller for joint: " << JointNumber;
  // Bouml preserved body end 000EBCF1
}

void EthercatMasterWithThread::deleteJointTrajectoryControllerRegistration(const unsigned int JointNumber) {
  // Bouml preserved body begin 000F06F1
    {
      boost::mutex::scoped_lock trajectoryControllerMutex(trajectoryControllerVectorMutex);
      if ((JointNumber - 1) >= this->trajectoryControllers.size())
        throw std::out_of_range("Invalid joint number");

      this->trajectoryControllers[JointNumber - 1] = NULL;
    }
    LOG(debug) << "delete joint trajectory controller registration for joint: " << JointNumber;
  // Bouml preserved body end 000F06F1
}

unsigned int EthercatMasterWithThread::getNumberOfThreadCyclesPerSecond() {
  // Bouml preserved body begin 000F41F1

    return static_cast<unsigned int> (1.0 / ((double) timeTillNextEthercatUpdate / 1000 / 1000));
  // Bouml preserved body end 000F41F1
}

void EthercatMasterWithThread::registerJointLimitMonitor(JointLimitMonitor* object, const unsigned int JointNumber) {
  // Bouml preserved body begin 000FB071
    {
      boost::mutex::scoped_lock limitMonitorMutex(jointLimitMonitorVectorMutex);
      if (this->jointLimitMonitors[JointNumber - 1] != NULL)
        LOG(warning) << "A joint limit monitor is already register for this joint!";
      if ((JointNumber - 1) >= this->jointLimitMonitors.size())
        throw std::out_of_range("Invalid joint number");

      this->jointLimitMonitors[JointNumber - 1] = object;
    }
    LOG(debug) << "register a joint limit monitor for joint: " << JointNumber;
  // Bouml preserved body end 000FB071
}

void EthercatMasterWithThread::registerDataTrace(void* object, const unsigned int JointNumber) {
  // Bouml preserved body begin 00105871
    {
      boost::mutex::scoped_lock datatraceM(dataTracesMutex);
      if (this->dataTraces[JointNumber - 1] != NULL)
        throw std::runtime_error("A data trace is already register for this joint!");
      if ((JointNumber - 1) >= this->dataTraces.size())
        throw std::out_of_range("Invalid joint number");

      this->dataTraces[JointNumber - 1] = (DataTrace*)object;
    }
    LOG(debug) << "register a data trace for joint: " << JointNumber;
  // Bouml preserved body end 00105871
}

void EthercatMasterWithThread::deleteDataTraceRegistration(const unsigned int JointNumber) {
  // Bouml preserved body begin 001058F1
    {
      boost::mutex::scoped_lock datatraceM(dataTracesMutex);
      if ((JointNumber - 1) >= this->dataTraces.size())
        throw std::out_of_range("Invalid joint number");

      this->dataTraces[JointNumber - 1] = NULL;

    }
    LOG(debug) << "removed data trace for joint: " << JointNumber;
  // Bouml preserved body end 001058F1
}

///stores a ethercat message to the buffer
///@param msgBuffer ethercat message
///@param jointNumber joint number of the sender joint
void EthercatMasterWithThread::setMsgBuffer(const YouBotSlaveMsg& msgBuffer, const unsigned int jointNumber) {
  // Bouml preserved body begin 000414F1

    if (this->automaticSendOn == true) {
      slaveMessages[jointNumber - 1].stctOutput.Set(msgBuffer.stctOutput);
    } else {
      YouBotSlaveMsg localMsg;
      localMsg.stctInput = msgBuffer.stctInput;
      localMsg.stctOutput = msgBuffer.stctOutput;
      localMsg.jointNumber = jointNumber;
      automaticSendOffBufferVector.push_back(localMsg);
    }

  // Bouml preserved body end 000414F1
}

///get a ethercat message form the buffer
///@param msgBuffer ethercat message
///@param jointNumber joint number of the receiver joint
void EthercatMasterWithThread::getMsgBuffer(const unsigned int jointNumber, YouBotSlaveMsg& returnMsg) {
  // Bouml preserved body begin 00041571

    if (this->automaticReceiveOn == true) {
      slaveMessages[jointNumber - 1].stctInput.Get(returnMsg.stctInput);
      slaveMessages[jointNumber - 1].stctOutput.Get(returnMsg.stctOutput);
      slaveMessages[jointNumber - 1].jointNumber.Get(returnMsg.jointNumber);
    } else {
      returnMsg = this->automaticReceiveOffBufferVector[jointNumber - 1];
    }

  // Bouml preserved body end 00041571
}

///stores a mailbox message in a buffer which will be sent to the motor controllers
///@param msgBuffer ethercat mailbox message
///@param jointNumber joint number of the sender joint
void EthercatMasterWithThread::setMailboxMsgBuffer(const YouBotSlaveMailboxMsg& msgBuffer, const unsigned int jointNumber) {
  // Bouml preserved body begin 00049D71
    this->mailboxMessages[jointNumber - 1].stctOutput.Set(msgBuffer.stctOutput);
    outstandingMailboxMsgFlag[jointNumber - 1] = true;
    return;
  // Bouml preserved body end 00049D71
}

///gets a mailbox message form the buffer which came form the motor controllers
///@param msgBuffer ethercat mailbox message
///@param jointNumber joint number of the receiver joint
bool EthercatMasterWithThread::getMailboxMsgBuffer(YouBotSlaveMailboxMsg& mailboxMsg, const unsigned int jointNumber) {
  // Bouml preserved body begin 00049DF1
    if (newInputMailboxMsgFlag[jointNumber - 1] == true) {
      this->mailboxMessages[jointNumber - 1].stctInput.Get(mailboxMsg.stctInput);
      newInputMailboxMsgFlag[jointNumber - 1] = false;
      return true;
    }
    return false;
  // Bouml preserved body end 00049DF1
}

///sends and receives ethercat messages and mailbox messages to and from the motor controllers
///this method is executed in a separate thread
void EthercatMasterWithThread::updateSensorActorValues() {
  // Bouml preserved body begin 0003F771

    long timeToWait = 0;
    boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration pastTime;
    //  int counter = 0;
    boost::posix_time::time_duration realperiode;
    boost::posix_time::time_duration timeSum = startTime - startTime;
    SlaveMessageOutput trajectoryContollerOutput;
    YouBotSlaveMailboxMsg tempMsg;


    while (!stopThread) {

      pastTime = boost::posix_time::microsec_clock::local_time() - startTime;
      timeToWait = timeTillNextEthercatUpdate - pastTime.total_microseconds() - 100;

      if (timeToWait < 0 || timeToWait > (int) timeTillNextEthercatUpdate) {
        //    printf("Missed communication period of %d  microseconds it have been %d microseconds \n",timeTillNextEthercatUpdate, (int)pastTime.total_microseconds()+ 100);
      } else {
        boost::this_thread::sleep(boost::posix_time::microseconds(timeToWait));
      }

      // realperiode = boost::posix_time::microsec_clock::local_time() - startTime;
      startTime = boost::posix_time::microsec_clock::local_time();

      /*
            counter++;
            timeSum  = timeSum + realperiode;

            if(counter == 1000){

              double dtotaltime = (double)timeSum.total_microseconds()/counter;
              printf("TotalTime %7.0lf us\n", dtotaltime);
              counter = 0;
              timeSum = startTime - startTime;
            }
       */




      //send and receive data from ethercat
      if (ec_send_processdata() == 0) {
        LOG(warning) << "Sending process data failed";
      }

      if (ec_receive_processdata(this->ethercatTimeout) == 0) {
        if (communicationErrors == 0) {
          LOG(warning) << "Receiving data failed";
        }
        communicationErrors++;
      } else {
        communicationErrors = 0;
      }

      if (communicationErrors > maxCommunicationErrors) {
        LOG(error) << "Lost EtherCAT connection";
        this->closeEthercat();
        stopThread = true;
        break;
      }

      if (ec_iserror())
        LOG(warning) << "there is an error in the soem driver";


      for (unsigned int i = 0; i < nrOfSlaves; i++) {

        //send data
        if(automaticSendOn == true)
          slaveMessages[i].stctOutput.Get(*(ethercatOutputBufferVector[i]));

        //receive data
        if(automaticReceiveOn == true)
          slaveMessages[i].stctInput.Set(*(ethercatInputBufferVector[i]));


        // Limit checker
        if (jointLimitMonitors[i] != NULL) {
          this->jointLimitMonitors[i]->checkLimitsProcessData(*(ethercatInputBufferVector[i]), *(ethercatOutputBufferVector[i]));
          //copy back changed velocity for limit checker
          slaveMessages[i].stctOutput.Set(*(ethercatOutputBufferVector[i]));
        }
        // this->parseYouBotErrorFlags(secondBufferVector[i]);

        //send mailbox messages from first buffer
        if (outstandingMailboxMsgFlag[i]) { 
          this->mailboxMessages[i].stctOutput.Get(tempMsg.stctOutput);
          this->mailboxMessages[i].slaveNumber.Get(tempMsg.slaveNumber);
          sendMailboxMessage(tempMsg);
          outstandingMailboxMsgFlag[i] = false;
          pendingMailboxMsgsReply[i] = true;
        }

        //receive mailbox messages to first buffer
        if (pendingMailboxMsgsReply[i]) {
          this->mailboxMessages[i].slaveNumber.Get(tempMsg.slaveNumber);
          if (receiveMailboxMessage(tempMsg)) {
            this->mailboxMessages[i].stctInput.Set(tempMsg.stctInput);
            newInputMailboxMsgFlag[i] = true;
            pendingMailboxMsgsReply[i] = false;
          }
        }
      }
      
      // Trajectory Controller
      {
        boost::mutex::scoped_lock trajectoryControllerMutex(trajectoryControllerVectorMutex);
        for (unsigned int i = 0; i < nrOfSlaves; i++) {
          if (this->trajectoryControllers[i] != NULL) {
            if (this->trajectoryControllers[i]->updateTrajectoryController(*(ethercatInputBufferVector[i]), trajectoryContollerOutput)) {
              //   printf("send vel slave: %d", i);
              (*(ethercatOutputBufferVector[i])).controllerMode = trajectoryContollerOutput.controllerMode;
              (*(ethercatOutputBufferVector[i])).value = trajectoryContollerOutput.value;
              //copy back
              slaveMessages[i].stctOutput.Set(*(ethercatOutputBufferVector[i]));
            }
          }
        }
      }
      // update Data Traces
      for (unsigned int i = 0; i < nrOfSlaves; i++) {
        {
          boost::mutex::scoped_lock datatraceM(dataTracesMutex);
          if (dataTraces[i] != NULL) {
            ((DataTrace*)dataTraces[i])->updateTrace();
          }
        }
      }
    }
  // Bouml preserved body end 0003F771
}

} // namespace youbot

#include "EthercatMasterInterface.hpp"
#include "TMCLProtocolDefinitions.hpp"
#include "Logger.hpp"
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

///provides all ethercat slave informations from the SOEM driver
///@param ethercatSlaveInfos ethercat slave informations


/// checks if an error has occurred in the soem driver
/// returns a true if an error has occurred


///return the quantity of ethercat slave which have an input/output buffer

youbot::EthercatMasterInterface::EthercatMasterInterface(const std::string& configFile, const std::string& configFilePath) : configFileName(configFile),
configFilepath(configFilePath) {
  this->ethercatConnectionEstablished = false;
  ethernetDevice = "eth0";
  mailboxTimeout = 4000; //micro sec
  ethercatTimeout = 500; //micro sec
  configfile = NULL;

  //initialize to zero
  for (unsigned int i = 0; i < 4096; i++) {
    IOmap_[i] = 0;
  }
  //read ethercat parameters form config file
  configfile = new ConfigFile(this->configFileName, this->configFilepath);

  // configfile.setSection("EtherCAT");
  configfile->readInto(ethernetDevice, "EtherCAT", "EthernetDevice");
  configfile->readInto(ethercatTimeout, "EtherCAT", "EtherCATTimeout_[usec]");
  configfile->readInto(mailboxTimeout, "EtherCAT", "MailboxTimeout_[usec]");
}

unsigned int youbot::EthercatMasterInterface::getNumberOfSlaves() const {
  return nrOfSlaves;
}

bool youbot::EthercatMasterInterface::isErrorInSoemDriver() {
  return ec_iserror();
}

void youbot::EthercatMasterInterface::getEthercatDiagnosticInformation(std::vector<ec_slavet>& ethercatSlaveInfos) {
  // Bouml preserved body begin 000D1EF1
  ethercatSlaveInfos = ethercatSlaveInfo;
  for (unsigned int i = 0; i < ethercatSlaveInfos.size(); i++) {
    ethercatSlaveInfos[i].inputs = NULL;
    ethercatSlaveInfos[i].outputs = NULL;
  }
  // Bouml preserved body end 000D1EF1
}

///sends the mailbox Messages which have been stored in the buffer
///@param mailboxMsg ethercat mailbox message


///closes the ethercat connection

bool youbot::EthercatMasterInterface::closeEthercat() {
  // Bouml preserved body begin 000D2071

  ethercatConnectionEstablished = false;
  // Request safe operational state for all slaves
  ec_slave[0].state = EC_STATE_SAFE_OP;

  /* request SAFE_OP state for all slaves */
  ec_writestate(0);

  //stop SOEM, close socket
  ec_close();

  return true;
  // Bouml preserved body end 000D2071
}

bool youbot::EthercatMasterInterface::sendMailboxMessage(const YouBotSlaveMailboxMsg& mailboxMsg) {
  // Bouml preserved body begin 00052F71
  //  LOG(trace) << "send mailbox message (buffer two) slave " << mailboxMsg.getSlaveNo();
  mailboxBufferSend[0] = mailboxMsg.stctOutput.moduleAddress;
  mailboxBufferSend[1] = mailboxMsg.stctOutput.commandNumber;
  mailboxBufferSend[2] = mailboxMsg.stctOutput.typeNumber;
  mailboxBufferSend[3] = mailboxMsg.stctOutput.motorNumber;
  mailboxBufferSend[4] = mailboxMsg.stctOutput.value >> 24;
  mailboxBufferSend[5] = mailboxMsg.stctOutput.value >> 16;
  mailboxBufferSend[6] = mailboxMsg.stctOutput.value >> 8;
  mailboxBufferSend[7] = mailboxMsg.stctOutput.value & 0xff;
  if (ec_mbxsend(mailboxMsg.slaveNumber, &mailboxBufferSend, mailboxTimeout)) {
    return true;
  }
  else {
    return false;
  }
  // Bouml preserved body end 00052F71
}

///receives mailbox messages and stores them in the buffer
///@param mailboxMsg ethercat mailbox message

bool youbot::EthercatMasterInterface::receiveMailboxMessage(YouBotSlaveMailboxMsg& mailboxMsg) {
  // Bouml preserved body begin 00052FF1
  if (ec_mbxreceive(mailboxMsg.slaveNumber, &mailboxBufferReceive, mailboxTimeout)) {
    //    LOG(trace) << "received mailbox message (buffer two) slave " << mailboxMsg.getSlaveNo();
    mailboxMsg.stctInput.replyAddress = (int)mailboxBufferReceive[0];
    mailboxMsg.stctInput.moduleAddress = (int)mailboxBufferReceive[1];
    mailboxMsg.stctInput.status = (int)mailboxBufferReceive[2];
    mailboxMsg.stctInput.commandNumber = (int)mailboxBufferReceive[3];
    mailboxMsg.stctInput.value = (mailboxBufferReceive[4] << 24 | mailboxBufferReceive[5] << 16 | mailboxBufferReceive[6] << 8 | mailboxBufferReceive[7]);
    return true;
  }
  return false;
  // Bouml preserved body end 00052FF1
}

void youbot::EthercatMasterInterface::parseYouBotErrorFlags(const YouBotSlaveMsg& messageBuffer) {
  // Bouml preserved body begin 000A9E71
  std::stringstream errorMessageStream;
  errorMessageStream << " ";
  std::string errorMessage;
  errorMessage = errorMessageStream.str();


  if (messageBuffer.stctInput.errorFlags & OVER_CURRENT) {
    LOG(error) << errorMessage << "got over current";
    //    throw JointErrorException(errorMessage + "got over current");
  }

  if (messageBuffer.stctInput.errorFlags & UNDER_VOLTAGE) {
    LOG(error) << errorMessage << "got under voltage";
    //    throw JointErrorException(errorMessage + "got under voltage");
  }

  if (messageBuffer.stctInput.errorFlags & OVER_VOLTAGE) {
    LOG(error) << errorMessage << "got over voltage";
    //   throw JointErrorException(errorMessage + "got over voltage");
  }

  if (messageBuffer.stctInput.errorFlags & OVER_TEMPERATURE) {
    LOG(error) << errorMessage << "got over temperature";
    //   throw JointErrorException(errorMessage + "got over temperature");
  }

  if (messageBuffer.stctInput.errorFlags & MOTOR_HALTED) {
    //   LOG(info) << errorMessage << "is halted";
    //   throw JointErrorException(errorMessage + "is halted");
  }

  if (messageBuffer.stctInput.errorFlags & HALL_SENSOR_ERROR) {
    LOG(error) << errorMessage << "got hall sensor problem";
    //   throw JointErrorException(errorMessage + "got hall sensor problem");
  }

  //    if (messageBuffer.stctInput.errorFlags & ENCODER_ERROR) {
  //      LOG(error) << errorMessage << "got encoder problem";
  //      //   throw JointErrorException(errorMessage + "got encoder problem");
  //    }
  //
  //     if (messageBuffer.stctInput.errorFlags & INITIALIZATION_ERROR) {
  //      LOG(error) << errorMessage << "got inizialization problem";
  //      //   throw JointErrorException(errorMessage + "got motor winding problem");
  //    }

  //    if (messageBuffer.stctInput.errorFlags & PWM_MODE_ACTIVE) {
  //  LOG(error) << errorMessage << "has PWM mode active";
  //   throw JointErrorException(errorMessage + "the cycle time is violated");
  //    }

  if (messageBuffer.stctInput.errorFlags & VELOCITY_MODE) {
    //   LOG(info) << errorMessage << "has velocity mode active";
    //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
  }

  if (messageBuffer.stctInput.errorFlags & POSITION_MODE) {
    //   LOG(info) << errorMessage << "has position mode active";
    //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
  }

  if (messageBuffer.stctInput.errorFlags & TORQUE_MODE) {
    //   LOG(info) << errorMessage << "has torque mode active";
    //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
  }

  //    if (messageBuffer.stctInput.errorFlags & EMERGENCY_STOP) {
  //      LOG(info) << errorMessage << "has emergency stop active";
  //      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
  //    }
  //
  //    if (messageBuffer.stctInput.errorFlags & FREERUNNING) {
  //   //   LOG(info) << errorMessage << "has freerunning active";
  //      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
  //    }

  if (messageBuffer.stctInput.errorFlags & POSITION_REACHED) {
    //    LOG(info) << errorMessage << "has position reached";
    //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
  }

  if (messageBuffer.stctInput.errorFlags & INITIALIZED) {
    //  LOG(info) << errorMessage << "is initialized";
    //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
  }

  if (messageBuffer.stctInput.errorFlags & TIMEOUT) {
    LOG(error) << errorMessage << "has a timeout";
    //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
  }

  if (messageBuffer.stctInput.errorFlags & I2T_EXCEEDED) {
    LOG(error) << errorMessage << "exceeded I2t";
    //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
  }

  // Bouml preserved body end 000A9E71
}

void youbot::EthercatMasterInterface::initialize() {

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ethernetDevice.c_str())) {
    LOG(info) << "Initializing EtherCAT on " << ethernetDevice << " with communication thread";
    /* find and auto-config slaves */
    if (ec_config(TRUE, &IOmap_) > 0) {
      LOG(trace) << ec_slavecount << " EtherCAT slaves found and configured.";

      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
      if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        LOG(warning) << "Not all EtherCAT slaves reached safe operational state.";
        ec_readstate();
        //If not all slaves operational find out which one
        for (int i = 1; i <= ec_slavecount; i++) {
          if (ec_slave[i].state != EC_STATE_SAFE_OP) {
            LOG(info) << "Slave " << i <<
              " State=" << ec_slave[i].state <<
              " StatusCode=" << ec_slave[i].ALstatuscode <<
              " : " << ec_ALstatuscode2string(ec_slave[i].ALstatuscode);
          }
        }
      }
      //Read the state of all slaves
      //ec_readstate();

      LOG(trace) << "Request operational state for all EtherCAT slaves";

      ec_slave[0].state = EC_STATE_OPERATIONAL;
      // request OP state for all slaves
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */
      ec_writestate(0);
      // wait for all slaves to reach OP state

      ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
      if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        LOG(trace) << "Operational state reached for all EtherCAT slaves.";
      }
      else {
        throw std::runtime_error("Not all EtherCAT slaves reached operational state.");
      }
    }
    else {
      throw std::runtime_error("No EtherCAT slaves found!");
    }
  }
  else {
    throw std::runtime_error("No socket connection on " + ethernetDevice + "\nExcecute as root");
  }

  std::string baseJointControllerName = "TMCM-174";
  std::string baseJointControllerNameAlternative = "TMCM-1632";
  std::string manipulatorJointControllerName = "TMCM-174";
  std::string ManipulatorJointControllerNameAlternative = "TMCM-1610";

  nrOfSlaves = 0;

  configfile->readInto(baseJointControllerName, "BaseJointControllerName");
  configfile->readInto(baseJointControllerNameAlternative, "BaseJointControllerNameAlternative");
  configfile->readInto(manipulatorJointControllerName, "ManipulatorJointControllerName");
  configfile->readInto(ManipulatorJointControllerNameAlternative, "ManipulatorJointControllerNameAlternative");
  std::string actualSlaveName;

  //reserve memory for all slave with a input/output buffer
  for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
    LOG(trace) << "Slave: " << cnt << " Name: " << ec_slave[cnt].name << " Output size: " << ec_slave[cnt].Obits
      << "bits Input size: " << ec_slave[cnt].Ibits << "bits State: " << ec_slave[cnt].state
      << " delay: " << ec_slave[cnt].pdelay; //<< " has dclock: " << (bool)ec_slave[cnt].hasdc;

    ethercatSlaveInfo.push_back(ec_slave[cnt]);

    actualSlaveName = ec_slave[cnt].name;
    if ((actualSlaveName == baseJointControllerName || actualSlaveName == baseJointControllerNameAlternative ||
      actualSlaveName == manipulatorJointControllerName || actualSlaveName == ManipulatorJointControllerNameAlternative
      ) && ec_slave[cnt].Obits > 0 && ec_slave[cnt].Ibits > 0) {
      identified_slaves.push_back(cnt);
      nrOfSlaves++;
      ethercatOutputBufferVector.push_back((SlaveMessageOutput*)(ec_slave[cnt].outputs));
      ethercatInputBufferVector.push_back((SlaveMessageInput*)(ec_slave[cnt].inputs));
    }
  }
  if (nrOfSlaves > 0) {
    LOG(info) << nrOfSlaves << " EtherCAT slaves found";
  }
  else {
    throw std::runtime_error("No EtherCAT slave could be found");
    return;
  }
}

bool youbot::EthercatMasterInterface::isEtherCATConnectionEstablished() {
  return this->ethercatConnectionEstablished;
}

#include "EthercatMasterInterface.hpp"


///provides all ethercat slave informations from the SOEM driver
///@param ethercatSlaveInfos ethercat slave informations


/// checks if an error has occurred in the soem driver
/// returns a true if an error has occurred


///return the quantity of ethercat slave which have an input/output buffer

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

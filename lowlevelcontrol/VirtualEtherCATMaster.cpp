#include "VirtualEtherCATMaster.hpp"
#include <exception>
#include <stdexcept>
#include "Time.hpp"
#include <string>

#include "Logger.hpp"

using namespace youbot;
using namespace youbot::intrinsic;

VirtualEtherCATMaster::Type VirtualEtherCATMaster::GetType() const {
  return Type::VIRTUAL;
}

VirtualEtherCATMaster::~VirtualEtherCATMaster() {
}

int VirtualEtherCATMaster::getSlaveNum() const {
  throw std::runtime_error("No imlpemented");
  return -1;
}

std::string VirtualEtherCATMaster::getSlaveName(int cnt) const {
  throw std::runtime_error("No imlpemented");
  return "";
}

VirtualEtherCATMaster::MailboxStatus VirtualEtherCATMaster::SendMailboxMessage(MailboxMessage::MailboxMessagePtr ptr) {
  throw std::runtime_error("No imlpemented");
  return MailboxStatus(0);
}

void VirtualEtherCATMaster::GetProcessMsg(ProcessBuffer& buff, uint8_t slaveNumber) const {
  throw std::runtime_error("No imlpemented");
}

void VirtualEtherCATMaster::SetProcessFromSlaveSize(uint8_t size, uint8_t slaveNumber) {
  throw std::runtime_error("No imlpemented");
}

int VirtualEtherCATMaster::SetProcessMsg(const ProcessBuffer& buffer, uint8_t slaveNumber) {
  throw std::runtime_error("No imlpemented");
  return -1;
}

void VirtualEtherCATMaster::ExchangeProcessMsg() {
  _callAfterExchangeCallbacks();
}
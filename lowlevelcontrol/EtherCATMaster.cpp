#include "EtherCATMaster.hpp"
#include "SimpleOpenEtherCATMaster.hpp"
#include "VirtualEtherCATMaster.hpp"
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot;
using namespace youbot::intrinsic;

EtherCATMaster::Ptr EtherCATMaster::CreatePhysical(const std::string& adapterName) {
  auto ptr = std::make_shared<SimpleOpenEtherCATMaster>(adapterName);
  ptr->Init();
  return ptr;
}

EtherCATMaster::Ptr EtherCATMaster::CreateVirtual() {
  return std::make_shared<VirtualEtherCATMaster>();
}

void youbot::EtherCATMaster::RegisterAfterExchangeCallback(std::function<void(void)> in) {
  afterExchangeCallbacks.push_back(in);
}

void youbot::EtherCATMaster::_callAfterExchangeCallbacks() {
  for (auto it : afterExchangeCallbacks)
	it();
}

EtherCATMaster::~EtherCATMaster() {
  //StopProcessThread();
}
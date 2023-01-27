#include "EtherCATMaster.hpp"
#include "SimpleOpenEtherCATMaster.hpp"
#include "VirtualEtherCATMaster.hpp"
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot;
using namespace youbot::intrinsic;

EtherCATMaster::Ptr EtherCATMaster::CreatePhysical() {
  return std::make_shared<SimpleOpenEtherCATMaster>();
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

/*
void EtherCATMaster::_processThreadFunc(int sleepMS) {
  isRunning = true;
  while (!toStopThread) {
	ExchangeProcessMsg();
	log(Log::info, "Thread running");
	SLEEP_MILLISEC(sleepMS);
  }
  isRunning = false;
}
*/
EtherCATMaster::~EtherCATMaster() {
  //StopProcessThread();
}
/*
void EtherCATMaster::StartProcessThread(int sleepMS) {
  toStopThread = false;
  thread = std::thread([this, sleepMS] { _processThreadFunc(sleepMS); });
  thread.detach();
}

void EtherCATMaster::StopProcessThread() {
  toStopThread = true;
  while (isRunning)
	SLEEP_MILLISEC(1);
  if (thread.joinable())
	thread.join();
}
*/
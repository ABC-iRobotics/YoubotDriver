#include "EtherCATMaster.hpp"
#include "SimpleOpenEtherCATMaster.hpp"
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot::intrinsic;

EtherCATMaster* EtherCATMaster::GetSingleton() {
  static SimpleOpenEtherCATMaster center;
  return &center;
}

void EtherCATMaster::_processThreadFunc(int sleepMS) {
  isRunning = true;
  while (!toStopThread) {
	ExchangeProcessMsg();
	log(Log::info, "Thread running");
	SLEEP_MILLISEC(sleepMS);
  }
  isRunning = false;
}

EtherCATMaster::~EtherCATMaster() {
  StopProcessThread();
}

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
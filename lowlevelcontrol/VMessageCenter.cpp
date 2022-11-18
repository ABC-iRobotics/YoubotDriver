#include "VMessageCenter.hpp"
#include "SOEMMessageCenter.hpp"
#include "Logger.hpp"
#include "Time.hpp"

VMessageCenter* VMessageCenter::GetSingleton() {
  static SOEMMessageCenter center;
  return &center;
}

void VMessageCenter::_processThreadFunc(int sleepMS) {
  isRunning = true;
  while (!toStopThread) {
	ExchangeProcessMsg();
	log(Log::info, "Thread running");
	SLEEP_MILLISEC(sleepMS);
  }
  isRunning = false;
}

VMessageCenter::~VMessageCenter() {
  StopProcessThread();
}

void VMessageCenter::StartProcessThread(int sleepMS) {
  toStopThread = false;
  thread = std::thread([this, sleepMS] { _processThreadFunc(sleepMS); });
  thread.detach();
}

void VMessageCenter::StopProcessThread() {
  toStopThread = true;
  while (isRunning)
	SLEEP_MILLISEC(1);
  if (thread.joinable())
	thread.join();
}
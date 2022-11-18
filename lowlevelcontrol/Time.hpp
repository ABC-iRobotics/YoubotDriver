#ifndef YOUBOT_TIME_HPP
#define YOUBOT_TIME_HPP

#include <thread>
#include <chrono>

#define SLEEP_MILLISEC(millisec) std::this_thread::sleep_for(std::chrono::milliseconds(millisec));
#define SLEEP_MICROSEC(microsec) std::this_thread::sleep_for(std::chrono::microseconds(microsec));
#define SLEEP_SEC(sec) std::this_thread::sleep_for(std::chrono::seconds(sec));

#endif //YOUBOT_TIME_HPP

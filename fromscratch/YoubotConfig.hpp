#ifndef YOUBOT_CONFIG_HPP
#define YOUBOT_CONFIG_HPP

#include <string>
#include <map>

typedef std::map<std::string, double> NameValueMap;

struct YoubotConfig {
  int gripperIndex, shoulderIndex, waistIndex, elbowIndex, wristPitchIndex, wristYawIndex;

  NameValueMap shoulderConfig, waistConfig, elbowConfig, wristPitchConfig, wristYawConfig;

  YoubotConfig(const std::string& filename);
};

#endif

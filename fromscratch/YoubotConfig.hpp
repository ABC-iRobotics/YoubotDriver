#ifndef YOUBOT_CONFIG_HPP
#define YOUBOT_CONFIG_HPP

#include <string>
#include <map>

typedef std::map<std::string, double> NameValueMap;

struct YoubotConfig {
  int gripperIndex, jointIndices[5];
  
  int& shoulderIndex = jointIndices[0];
  int& waistIndex = jointIndices[1];
  int& elbowIndex = jointIndices[2];
  int& wristPitchIndex = jointIndices[3];
  int& wristYawIndex = jointIndices[4];

  NameValueMap jointConfigs[5];
  
  NameValueMap& shoulderConfig = jointConfigs[0];
  NameValueMap& waistConfig = jointConfigs[1];
  NameValueMap& elbowConfig = jointConfigs[2];
  NameValueMap& wristPitchConfig = jointConfigs[3];
  NameValueMap& wristYawConfig = jointConfigs[4];

  YoubotConfig(const std::string& filename);
};

#endif

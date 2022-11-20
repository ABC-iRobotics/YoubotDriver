#ifndef YOUBOT_CONFIG_HPP
#define YOUBOT_CONFIG_HPP

#include <string>
#include <map>

namespace youbot {

  typedef std::map<std::string, double> NameValueMap;

  struct YoubotConfig {
	int gripperIndex, jointIndices[5];

	int& waistIndex = jointIndices[0];
	int& shoulderIndex = jointIndices[1];
	int& elbowIndex = jointIndices[2];
	int& wristPitchIndex = jointIndices[3];
	int& wristYawIndex = jointIndices[4];

	NameValueMap jointConfigs[5];

	NameValueMap& waistConfig = jointConfigs[0];
	NameValueMap& shoulderConfig = jointConfigs[1];
	NameValueMap& elbowConfig = jointConfigs[2];
	NameValueMap& wristPitchConfig = jointConfigs[3];
	NameValueMap& wristYawConfig = jointConfigs[4];

	std::map<std::string, std::string> logConfig;

	YoubotConfig(const std::string& filename);
  };
}
#endif

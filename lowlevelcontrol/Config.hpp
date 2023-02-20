#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <map>

namespace youbot {

  typedef std::map<std::string, double> NameValueMap;

  /// <summary>
  /// Content of a config file loaded into NameValueMap-s
  /// </summary>
  struct Config {
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

	/// <summary>
	/// Constructor: only saves the filename
	/// </summary>
	/// <param name="filename"> path and filename of the config file </param>
	Config(const std::string& filename);

	/// <summary>
	/// Loads the contant of the config file into the NameValueMap-s
	/// without checking, sorting, etc. the values within
	/// </summary>
	void Init();

	const std::string filename;
  };
}
#endif

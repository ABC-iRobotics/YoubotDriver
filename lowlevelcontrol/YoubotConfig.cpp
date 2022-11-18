#include "YoubotConfig.hpp"
#include <fstream>
#include "json.hpp"
#include "Logger.hpp"

YoubotConfig::YoubotConfig(const std::string& filename) {
  std::ifstream configFile;
  configFile.open(filename, std::ios_base::in);

  if (!configFile.is_open()) {
    log(__PRETTY_FUNCTION__,__LINE__,__FILE__, Log::fatal, "FATAL ERROR: cannot open json file '" + filename);
    throw std::runtime_error("FATAL ERROR: cannot open json file '" + filename + "' (in YoubotConfig::YoubotConfig)");
  }

  nlohmann::json config;
  try {
    configFile >> config;
  }
  catch (...) {
    log(__PRETTY_FUNCTION__, __LINE__, __FILE__, Log::fatal, "FATAL ERROR: parsing config file '" + filename);
    std::throw_with_nested(std::runtime_error("FATAL ERROR: parsing config file '" + filename + "' (in YoubotConfig::YoubotConfig)"));
  }
  if (config.find("JointIndices") != config.end()) {
    auto indices = config.at("JointIndices");
    if (indices.find("Gripper") == indices.end())
      throw std::runtime_error("JointIndices/Gripper is not found.");
    gripperIndex = indices.at("Gripper");
    if (indices.find("WaistJoint") == indices.end())
      throw std::runtime_error("JointIndices/WaistJoint is not found.");
    waistIndex = indices.at("WaistJoint");
    if (indices.find("ShoulderJoint") == indices.end())
      throw std::runtime_error("JointIndices/ShoulderJoint is not found.");
    shoulderIndex = indices.at("ShoulderJoint");
    if (indices.find("ElbowJoint") == indices.end())
      throw std::runtime_error("JointIndices/ElbowJoint is not found.");
    elbowIndex = indices.at("ElbowJoint");
    if (indices.find("WristPitchJoint") == indices.end())
      throw std::runtime_error("JointIndices/WristPitchJoint is not found.");
    wristPitchIndex = indices.at("WristPitchJoint");
    if (indices.find("WristYawJoint") == indices.end())
      throw std::runtime_error("JointIndices/WristYawJoint is not found.");
    wristYawIndex = indices.at("WristYawJoint");
  }
  else
    throw std::runtime_error("JointIndices is not found.");

  // Waist params into NamevalueMap
  if (config.find("WaistJoint") != config.end())
    for (auto& it : config.at("WaistJoint").items())
      waistConfig.insert({ it.key(),it.value() });
  else
    throw std::runtime_error("WaistJoint is not found.");

  // Shoulder params into NamevalueMap
  if (config.find("ShoulderJoint") != config.end())
    for (auto& it : config.at("ShoulderJoint").items())
      shoulderConfig.insert({ it.key(),it.value() });
  else
    throw std::runtime_error("ShoulderJoint is not found.");

  // Elbow params into NamevalueMap
  if (config.find("ElbowJoint") != config.end())
    for (auto& it : config.at("ElbowJoint").items())
      elbowConfig.insert({ it.key(),it.value() });
  else
    throw std::runtime_error("ElbowJoint is not found.");

  // WristPitch params into NamevalueMap
  if (config.find("WristPitchJoint") != config.end())
    for (auto& it : config.at("WristPitchJoint").items())
      wristPitchConfig.insert({ it.key(),it.value() });
  else
    throw std::runtime_error("WristPitchJoint is not found.");

  // WristYaw params into NamevalueMap
  if (config.find("WristYawJoint") != config.end())
    for (auto& it : config.at("WristYawJoint").items())
      wristYawConfig.insert({ it.key(),it.value() });
  else
    throw std::runtime_error("WristYawJoint is not found.");

  if (config.find("Logging") != config.end())
    for (auto& it : config.at("Logging").items())
      logConfig.insert({ it.key(),it.value() });
}

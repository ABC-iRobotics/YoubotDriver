#include "YoubotJoint.hpp"

YoubotJoint::YoubotJoint(int slaveIndex, const NameValueMap& config, VMessageCenter* center)
    : slaveIndex(slaveIndex), center(center), config(config) {}

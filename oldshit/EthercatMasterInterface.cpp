#include "EthercatMasterInterface.hpp"


///provides all ethercat slave informations from the SOEM driver
///@param ethercatSlaveInfos ethercat slave informations


/// checks if an error has occurred in the soem driver
/// returns a true if an error has occurred


///return the quantity of ethercat slave which have an input/output buffer

unsigned int youbot::EthercatMasterInterface::getNumberOfSlaves() const {
  return nrOfSlaves;
}

bool youbot::EthercatMasterInterface::isErrorInSoemDriver() {
  return ec_iserror();
}

void youbot::EthercatMasterInterface::getEthercatDiagnosticInformation(std::vector<ec_slavet>& ethercatSlaveInfos) {
  // Bouml preserved body begin 000D1EF1
  ethercatSlaveInfos = ethercatSlaveInfo;
  for (unsigned int i = 0; i < ethercatSlaveInfos.size(); i++) {
    ethercatSlaveInfos[i].inputs = NULL;
    ethercatSlaveInfos[i].outputs = NULL;
  }
  // Bouml preserved body end 000D1EF1
}

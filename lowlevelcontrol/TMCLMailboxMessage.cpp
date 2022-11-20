#include "TMCLMailboxMessage.hpp"

using namespace youbot;
using namespace youbot::intrinsic;

GetFirmware::GetFirmware(int slaveIndex) : MailboxMessage(slaveIndex, 8, 8) {
  uint8_t* buff = getToSlaveBuff();
  *buff = (uint8_t)TMCL::Module::DRIVE;
  *(buff + 1) = (uint8_t)TMCL::Cmd::FIRMWARE_VERSION;
  *(buff + 2) = 0;
  *(buff + 3) = 0;
}

void GetFirmware::GetOutput(int& controllernum, int& firmwarenum) const {
  char* ptr, * ptr2;
  controllernum = strtol((const char*)getFromSlaveBuff(), &ptr, 10);
  char buff[4];
  memcpy(buff, ptr + 1, 3);
  buff[3] = 'V';
  firmwarenum = strtol(buff, &ptr2, 10);
}

std::shared_ptr<GetFirmware> GetFirmware::InitSharedPtr(int slaveIndex) {
  return std::make_shared<GetFirmware>(slaveIndex);
}

double ConvertTemperature(uint32_t adc) {
  return 3434 * 298.16 / (3434 + log((9011.2 / adc + 2.2) / 10) * 298.16) - 273.16;
}

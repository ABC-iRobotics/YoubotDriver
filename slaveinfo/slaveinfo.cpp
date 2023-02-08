#include "adapters.hpp"
#include "ethercat.h"
#include "Logger.hpp"

using namespace youbot;

int main(int argc, char *argv[])
{
  Log::Setup({});
  ec_adaptert* adapter0 = ec_find_adapters();
  ec_adaptert* adapter = adapter0;
  char buff[1000];
  while (adapter != NULL) {
    sprintf(buff, "Description : %s, Device to use for wpcap: %s", adapter->desc, adapter->name);
    log(Log::info, std::string(buff));
    slaveinfo(adapter->name);
    adapter = adapter->next;
  }
  ec_free_adapters(adapter0);
  return (0);
}

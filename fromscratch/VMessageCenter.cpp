#include "VMessageCenter.hpp"
#include "SOEMMessageCenter.hpp"

VMessageCenter* VMessageCenter::GetSingleton() {
  static SOEMMessageCenter center;
  return &center;
}

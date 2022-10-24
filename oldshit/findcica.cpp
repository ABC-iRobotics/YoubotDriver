/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : slaveinfo [ifname] [-sdo] [-map]
 * Ifname is NIC interface, f.e. eth0.
 * Optional -sdo to display CoE object dictionary.
 * Optional -map to display slave PDO mapping
 *
 * This shows the configured slave data.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>

#include "adapters.hpp"
#include "DataObjectLockFree.hpp"

char ifbuf[1024];

int main2(int argc, char *argv[])
{
  char name[5000];

  if (findYouBotEtherCatAdapter(name)) {
	printf("\n\n\nAdapter found: %s\n", name);
  }

  return (0);
}

#include "YouBotManipulator.hpp"

int main(int argc, char* argv[])
{ 
  try {
	youbot::YouBotManipulator arm("config",
	  //"D:/Tresors/WORK/PROJECT - Ruzics Barna KUKA/myYouBotDriver/config");
	  "C:/Users/kutij/Desktop/myYouBotDriver/src/config");

  }
  catch (std::exception& e) {
	std::cout << e.what() << std::endl;
  }
  catch (...) {
	std::cout << "unhandled exception" << std::endl;
  }

  return (0);
}

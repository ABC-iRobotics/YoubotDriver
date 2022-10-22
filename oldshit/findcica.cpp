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

  DataObjectLockFree<int> i;



  return (0);
}

#include "YouBotManipulator.hpp"

int main(int argc, char* argv[])
{
  boost::posix_time::time_duration dur = boost::posix_time::seconds(100);// boost::posix_time::duration_from_string("4.000001");

  auto n = dur.total_microseconds();

  std::chrono::microseconds dur2(n);


  std::chrono::milliseconds dur3(10);

  auto us = std::chrono::duration_cast<std::chrono::microseconds>(dur3);

  printf("%jd %jd\n",n,us.count());

  try {
	youbot::YouBotManipulator arm("youbot-manipulator",
	  "D:/Tresors/WORK/PROJECT - Ruzics Barna KUKA/myYouBotDriver/config");


  }
  catch (std::exception& e) {
	std::cout << e.what() << std::endl;
  }
  catch (...) {
	std::cout << "unhandled exception" << std::endl;
  }

  return (0);
}

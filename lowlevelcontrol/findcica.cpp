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

char ifbuf[1024];

int main(int argc, char *argv[])
{
  char name[5000];

  if (findYouBotEtherCatAdapter(name)) {
	printf("\n\n\nAdapter found: %s\n", name);
  }

  return (0);
}

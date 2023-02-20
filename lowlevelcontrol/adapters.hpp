#ifndef ADAPTERS_H
#define ADAPTERS_H

namespace youbot {
  /// <summary>
  /// Function to find an EtherNET adress, where checkIfYouBotEtherCatAdapter returns with true.
  /// It uses the SOEM driver.
  /// </summary>
  /// <param name="name"> buffer, the resulted adress will be saved into</param>
  /// <returns> returns if it is successful </returns>
  bool findYouBotEtherCatAdapter(char* name);

  /// <summary>
  /// Function to check if there are 6 etherCAT slaves on the EtherNET adress "ifname" and they are "TMCM-1610","TMCM-174" or "TMCM-1632"
  /// It uses the SOEM driver.
  /// </summary>
  /// <param name="ifname"> the EtherNET adress </param>
  /// <param name="printSDO"> log setting </param>
  /// <param name="printMAP"> log setting </param>
  /// <returns> returns if there is a youbot manipulator arm on the adress </returns>
  bool checkIfYouBotEtherCatAdapter(char* ifname, bool printSDO = false, bool printMAP = false);

  /// <summary>
  /// Function that launches the original slaveinfo of the SOEM driver on etherNET adress "ifname".
  /// It uses the SOEM driver.
  /// </summary>
  /// <param name="ifname"></param>
  void slaveinfo(char* ifname);
}

#endif

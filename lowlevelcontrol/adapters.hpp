#ifndef ADAPTERS_H
#define ADAPTERS_H

// Tries to find an adapter of of youBot arm
//
//
bool findYouBotEtherCatAdapter(char* name);;

bool checkIfYouBotEtherCatAdapter(char* ifname, bool printSDO=false, bool printMAP=false);

void slaveinfo(bool printSDO, bool printMAP, char* ifname);


#endif

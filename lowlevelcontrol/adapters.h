#ifndef ADAPTERS_H
#define ADAPTERS_H

// Tries to find an adapter of of youBot arm
//
//
bool findYouBotEtherCatAdapter(char* name);;

bool checkIfYouBotEtherCatAdapter(bool printSDO, bool printMAP, char* ifname);

void slaveinfo(bool printSDO, bool printMAP, char* ifname);


#endif

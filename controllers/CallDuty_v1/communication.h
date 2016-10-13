#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "headerStruct.h"

int listening(WbDeviceTag receiver, int floorColor, int botNumber, int *listFriends, int *stateUML, int *suggestedState);
int speaking(struct a *sOri, WbDeviceTag emitter, int flagCom, int botNumber, int toWhom, int codeTask, int time, int cache);

#endif
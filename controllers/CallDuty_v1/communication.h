#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "readWriteFiles.h"
#include "headerStruct.h"

int listening(WbDeviceTag receiver, int floorColor, int botNumber, int *listFriends, int *stateUML, int *suggestedState, struct flags4Files *flagFiles, char *fileRobot, char *dirPath);
int speaking(struct robotDevices *bot, int botNumber, int toWhom, int codeTask, int time, int cache, struct flags4Files *flagFiles, char *fileRobot, char *dirPath);

#endif
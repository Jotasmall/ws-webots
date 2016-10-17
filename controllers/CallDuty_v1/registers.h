#ifndef REGISTERS_H
#define REGISTERS_H

#include <webots/robot.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <windows.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "readWriteFiles.h"
#include "communication.h"
#include "headerStruct.h"

void updateEstimations(int task, int value, int cache, int timeImage, int timeMeasured, int timeListened, struct robotState *botState, struct modelParam *botParam, struct robotEstimations *botEst, struct robotDevices *botDevices, struct flags4Files *botFlags);
void updateBitacora(int codeTask, int estimations, int cache, int timeMeasured, int timeListened, struct robotEstimations *botEst, struct flags4Files *botFlags, struct robotDevices *botDevices, struct robotState *botState);
//void cronometer(int task, int cache, int *suggestedState, int *timeImage, int *timeMeasured, struct robotDevices *botDevices, struct robotState *botState, struct flags4Files *botFlags);

/*
void cronometer(int task, int cacheShape);
void countObjects();
*/
#endif
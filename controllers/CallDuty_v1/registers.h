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

void updateEstimations(int task, int cache, struct robot *bot);
void updateBitacora(int codeTask, int estimations, int cache, struct robot *bot);
void cronometer(int task, int cache, struct robot *bot);
void countObjects(int nbRegions, struct robot *bot);

#endif
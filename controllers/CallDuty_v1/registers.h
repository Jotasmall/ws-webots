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
#include "communication.h"#include "headerStruct.h" 
#include "botStruct.h"
robot bot;

void updateEstimations(int task, int cache);
void updateBitacora(int codeTask, int estimations, int cache);
void cronometer(int task, int cache);
void countObjects(int nbRegions);

#endif
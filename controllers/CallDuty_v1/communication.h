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
#include "registers.h"
#include "headerStruct.h" 
#include "botStruct.h"
#include "readWriteFiles.h"
robot bot;

int listening();
int speaking(int toWhom, int codeTask, int time, int cache);

#endif
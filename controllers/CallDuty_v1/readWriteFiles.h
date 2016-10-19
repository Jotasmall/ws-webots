#ifndef READ_WRITE_FILES_H
#define READ_WRITE_FILES_H

#include <webots/robot.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <windows.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "headerStruct.h" 
#include "botStruct.h"
robot bot;

void createDir(int option, int flagBuild);
void createFiles();
void writeDecision(float boundP, float realP, int mechanism, int flagTravel);
void writeMessage(int speaking, const char *msg);

#endif
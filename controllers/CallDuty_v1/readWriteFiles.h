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

void createDir(int option, int flagBuild, struct flags4Files *botFlags);
void createFiles(struct flags4Files *botFlags);
void writeDecision(float boundP, float realP, int mechanism, int flagTravel, struct flags4Files *botFlags);
void writeMessage(int speaking, const char *msg, struct flags4Files *botFlags);

#endif
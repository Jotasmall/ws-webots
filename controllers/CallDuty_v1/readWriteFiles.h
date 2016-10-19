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

void createDir(int option, int flagBuild, struct robot *bot);
void createFiles(struct robot *bot);
void writeDecision(float boundP, float realP, int mechanism, int flagTravel, struct robot *bot);
void writeMessage(int speaking, const char *msg, struct robot *bot);

#endif
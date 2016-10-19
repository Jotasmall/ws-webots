#ifndef COMPLEX_MOVEMENT_H
#define COMPLEX_MOVEMENT_H

#include <webots/differential_wheels.h>
#include <webots/camera.h>
#include "initBot.h"
#include "movement.h"
#include "dsp.h"
#include "communication.h"
#include "headerStruct.h"
#include "registers.h"

int followingLine(double *speed, WbDeviceTag *displayExtra, struct robot *bot);
int doorEntrance(double *speed, int steps, struct robot *bot);
int setRobotPosition(double *speed, WbDeviceTag *displayExtra, struct robot *bot);
int going2region(int colorDestination, double *speed, WbDeviceTag *displayExtra, struct robot *bot);
int going2it(int index, double *speed, WbDeviceTag *displayExtra, struct robot *bot);
int levyFlight(double *speed, WbDeviceTag *displayExtra, struct robot *bot);


#endif
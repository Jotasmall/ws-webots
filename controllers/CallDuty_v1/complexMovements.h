#ifndef COMPLEX_MOVEMENT_H
#define COMPLEX_MOVEMENT_H

#include <webots/differential_wheels.h>
#include <webots/camera.h>
#include "initBot.h"
#include "movement.h"
#include "dsp.h"
#include "communication.h"#include "headerStruct.h" 
#include "botStruct.h"
#include "registers.h"
robot bot;

int followingLine(double *speed, WbDeviceTag *displayExtra);
int doorEntrance(double *speed, int steps);
int setRobotPosition(double *speed, WbDeviceTag *displayExtra);
int going2region(double *speed, WbDeviceTag *displayExtra);
int going2it(int index, double *speed, WbDeviceTag *displayExtra);
int levyFlight(double *speed, WbDeviceTag *displayExtra);


#endif
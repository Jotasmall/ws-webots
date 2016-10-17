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

int followingLine(double *speed, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, int colorLine, struct robotState *botState, struct robotCamera *botCam, struct robotDevices *botDevices);
int doorEntrance(double *speed, int steps, struct robotDevices *botDevices, struct flags4Files *botFlags, struct robotState *botState);
int setRobotPosition(int colorLine, double *speed, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState);
int going2region(int color, int colorLine, int colorDestination, double *speed, WbDeviceTag *displayExtra, struct robotCamera *botCam, int *shapeSeen, int *pointA, int *pointB, struct robotDevices *botDevices, struct robotState *botState, struct flags4Files *botFlags);
int going2it(int index,int color, double *speed, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState);
int levyFlight(int figura, int color, double *speed, struct robotDevices *botDevices, struct robotCamera *botCam, struct robotState *botState, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB);


#endif
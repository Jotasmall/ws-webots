#ifndef COMPLEX_MOVEMENT_H
#define COMPLEX_MOVEMENT_H

#include <webots/differential_wheels.h>
#include <webots/camera.h>
#include "movement.h"
#include "dsp.h"
#include "headerStruct.h"

int followingLine(double *speed, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, int colorLine, struct robotState *botState, struct robotCamera *botCam, struct robotDevices *botDevices);


#endif
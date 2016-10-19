#ifndef INITBOT_H
#define INITBOT_H

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/led.h>
#include <webots/receiver.h>
#include <webots/emitter.h>

#include <windows.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "headerStruct.h" 
#include "botStruct.h"
robot bot;

void resetDevices();
void resetDisplay(WbDeviceTag *displayExtra);
void calibrateSensors();
void initEstimations(int nRegions);

#endif

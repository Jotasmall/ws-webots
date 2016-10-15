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

/*void initSensors(WbDeviceTag *sensors);
void initCamera(WbDeviceTag *cam, unsigned short *width, unsigned short *height);
void initLeds(WbDeviceTag *leds);*/
void resetDevices(struct robotDevices *bot);
void resetDisplay(WbDeviceTag *displayExtra, unsigned short width, unsigned short height);
void calibrateSensors(struct robotDevices *bot);
//void calibrateSensors(WbDeviceTag *sensors, int *ps_offset);
//void initEstimations(struct robotEstimations *bot, int nRegions);

#endif

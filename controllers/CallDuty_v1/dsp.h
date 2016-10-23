#ifndef IMAGE_H
#define IMAGE_H

#include <webots/camera.h>
#include <webots/robot.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <webots/display.h>
#include "movement.h"
#include "communication.h"
#include "headerStruct.h" 
#include "botStruct.h"
robot bot;

int compareColorPixel(int pixelX, int pixelY, int foreground);
int cont_height_figure(int indexP, int color);
int detectTam();
int whereIam(int avoiding, double *speed);
int whereArrive(double *speed);
int findMiddle(int wrongLine);
int waitingColor();
int whatIsee(int color, float Eccentricity, float Extent, int squarewidth, int middleAxisH, int middleAxisV);
int detectImage(WbDeviceTag *displayExtra);
int doubleCheck(double *speed, WbDeviceTag *displayExtra);
int check4Robot(WbDeviceTag *displayExtra);

#endif
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

int compareColorPixel(int pixelX, int pixelY, int foreground, struct robot *bot);
int cont_height_figure(int indexP, int color, struct robot *bot);
int detectTam(struct robot *bot);
int whereIam(int avoiding, double *speed, struct robot *bot);
int whereArrive(double *speed, struct robot *bot);
int find_middle(int wrongLine, struct robot *bot);
int waiting_color(struct robot *bot);
int whatIsee(int color, float Eccentricity, float Extent, int squarewidth, int middleAxisH, int middleAxisV);
int detectImage(WbDeviceTag *displayExtra, struct robot *bot);
int doubleCheck(double *speed, WbDeviceTag *displayExtra, struct robot *bot);
int check4Robot(WbDeviceTag *displayExtra, struct robot *bot);

#endif
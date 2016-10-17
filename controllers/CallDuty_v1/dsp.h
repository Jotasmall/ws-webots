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

int compareColorPixel(struct robotCamera *botCam, const unsigned char *image, int pixelX, int pixelY, int foreground, struct robotState *bot);
int cont_height_figure(int indexP, int color, struct robotCamera *botCam, struct robotState *botState);
int whereIam(int avoiding, int color, double *speed, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState);
int whereArrive(int color, double *speed, struct robotDevices *botDevices, struct robotCamera *botCam, struct robotState *botState, struct flags4Files *botFlags);
int find_middle(int wrongLine, int colorLine, struct robotCamera *botCam, struct robotState *botState);
int waiting_color(int foreground, int color, struct robotState *botState, struct robotCamera *botCam, struct robotDevices *botDevices);
int whatIsee(int color, float Eccentricity, float Extent, int squarewidth, int middleAxisH, int middleAxisV, int numImage);
int detectImage(WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, int color, int foreground, int shape, int numImage, int *numberComponents, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState);
int doubleCheck(double *speed, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, int color, int foreground, int shape, int numImage, int *numberComponents, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState);
int check4Robot(WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, int color, int foreground, int shape, int numImage, int *numberComponents, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState);

#endif
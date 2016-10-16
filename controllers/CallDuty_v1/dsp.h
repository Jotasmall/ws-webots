#ifndef IMAGE_H
#define IMAGE_H

#include <webots/camera.h>
#include <webots/robot.h>
#include "headerStruct.h"


int compareColorPixel(struct robotCamera *botCam, const unsigned char *image, int pixelX, int pixelY, int foreground, struct robotState *bot);

//int cont_height_figure(int indexP);
/*
int detectTam();
int find_middle(int wrongLine, int colorLine);
int whereIam(int avoiding);
int whereArrive();
int waiting_color(int foreground);
int check4Robot();
int doubleCheck();
int detectImage(int foreground, int shape, int numImage, int *numberComponents);
int whatIsee(float Eccentricity, float Extent, int squarewidth, int middleAxisH, int middleAxisV, int numImage);
*/

#endif
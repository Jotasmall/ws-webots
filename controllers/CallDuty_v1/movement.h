#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "dsp.h"
#include "registers.h"
#include "headerStruct.h"

void forward(int steps, double *speed, struct robot *bot);
void turnSteps(int steps, double *speed, struct robot *bot);
void avoidance(double *speed, struct robot *bot);
int readSensors(int print, struct robot *bot);
int run(int steps, double *speed, struct robot *bot);
int hitWall(int front, double *speed, struct robot *bot);
int enterTam(double *speed, struct robot *bot);
int waiting(int n, struct robot *bot);

#endif
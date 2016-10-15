#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "headerStruct.h"

void forward(int steps, double *speed);
void turnSteps(int steps, double *speed);
void avoidance(double *speed, struct robotDevices *bot);
int readSensors(int print, struct robotDevices *bot);
int run(int flagLoad, int steps, double *speed, struct robotDevices *bot);
int hitWall(int front, double *speed, struct robotDevices *bot);
int enterTam(struct robotDevices *bot, double *speed);
int waiting(int n);

#endif
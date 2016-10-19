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
#include "botStruct.h"
robot bot;

void forward(int steps, double *speed);
void turnSteps(int steps, double *speed);
void avoidance(double *speed);
int readSensors(int print);
int run(int steps, double *speed);
int hitWall(int front, double *speed);
int enterTam(double *speed);
int waiting(int n);

#endif
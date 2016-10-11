#ifndef MOVEMENT_H
#define MOVEMENT_H

void forward(int steps, double *speed);
void turnSteps(int steps, double *speed);
int readSensors(int print, int *ps_value);
int run(int steps, double *speed, int *ps_value);

#endif
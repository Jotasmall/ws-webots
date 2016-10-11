#ifndef MOVEMENT_H
#define MOVEMENT_H

void forward(int steps, double *speed);
void turnSteps(int steps, double *speed);
int readSensors(int print, int *ps_value, int *ps_offset, WbDeviceTag *sensors);
int run(int flagLoad, int steps, double *speed, int *ps_value, int *ps_offset, WbDeviceTag *sensors);
void avoidance(double *speed, int *ps_value, int *ps_offset, WbDeviceTag *sensors);

#endif
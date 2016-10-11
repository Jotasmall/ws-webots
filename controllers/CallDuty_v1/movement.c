#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include "movement.h"

#define TIME_STEP 64
#define NB_DIST_SENS 8
#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 600


void forward(int steps, double *speed){ //ok-
  int k = 1;
  if (steps < 0) {
    k = -1;
    steps = abs(steps);
  }
  speed[LEFT] = k*300; //200
  speed[RIGHT] = k*300;
  k = 0;
  wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);
  while (k < steps) {
    k++;
    wb_robot_step(TIME_STEP); 
  }
  wb_differential_wheels_set_speed(0, 0);
  wb_robot_step(TIME_STEP);
}

void turnSteps(int steps, double *speed){ 
  // In simulations 360 degrees 106 required steps on encoder at 200 timeStep 64
  if (steps < 0) {
    // Turn upon the same position
    speed[LEFT] = -206;
    speed[RIGHT] = 206;
    steps = abs(steps); 
  } else {
    speed[LEFT] = 206;
    speed[RIGHT] = -206;
  }
  wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);
  while (steps > 0) {
    steps--;
    wb_robot_step(TIME_STEP);  
    cronometer(-1, 0);
  }
  wb_differential_wheels_set_speed(0, 0);
  wb_robot_step(TIME_STEP);
}

int readSensors(int print, int *ps_value){ 
  int flag = 0, i, k;
  // Reset values
  for(i=0; i<NB_DIST_SENS; i++){ ps_value[i] = 0;}
  //Sensor values
  for (k=0; k<SAMPLES; k++) { 
    for (i=0; i<NB_DIST_SENS; i++) {
      ps_value[i] += (int)wb_distance_sensor_get_value(Robotps[i])-ps_offset[i];
    }
    wb_robot_step(TIME_STEP); 
  }  
  for (i=0; i<NB_DIST_SENS; i++){
    ps_value[i] /= SAMPLES;
    if (ps_value[i] > THRESHOLD_DIST) { 
      flag = 1;
      if (print) { //Sensor 5 for follow wall
        //printf("\n An obstacle is detected at sensor %d value %d", i, ps_value[i]);
        //printf("\n");
      }
    } 
  }
  return flag;
}

int run(int steps, double *speed, int *ps_value){ //ok-
  int i, j;
  int matrix[8][2] = {{150,-35},{100, -15},{80, -10},{-10,-10},{-10,-10},{-10,80},{-30,100},{-20,150}};
  while(steps > 0) {  
    readSensors(0, ps_value);
    for (i = 0; i < 2; i++) {
      speed[i] = 0;
      for (j = 0; j < NB_DIST_SENS; j++) {
        // 0.002 = 1/HalfRange = 512 
        speed[i] += matrix[j] [i] * (1 - ps_value[j]*0.002);
      }
      if (speed[i] > MAX_SPEED) {
        speed[i] = MAX_SPEED;
      } else if (speed[i] < -MAX_SPEED) {
        speed[i] = -MAX_SPEED;
      }
    }
    if (flagLoad){ //reducing speed when cargo
      speed[LEFT]=speed[LEFT]*SPEEDCARGO;
      speed[RIGHT]=speed[RIGHT]*SPEEDCARGO;
    }
    wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);
    wb_robot_step(TIME_STEP);
    cronometer(-1, 0); 
        
    steps--;
// 	  Every 5 steps check ground color
//    if(steps%5 == 0){ whereIam(1);}
  } 
  wb_differential_wheels_set_speed(0, 0);
  wb_robot_step(TIME_STEP);
  return 1;
}


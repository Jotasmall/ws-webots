/*
 * File:          my_controller.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_door.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define SIDES 2
#define UP -0.75
#define DOWN 0.75

WbDeviceTag door;
WbDeviceTag ds[2];

int openDoor(float upDown);
void wait(int steps);
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  int i;
  // initialize motors  
  door = wb_robot_get_device("motor1");
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  
  ds[0] = wb_robot_get_device("ds_enter");
  wb_distance_sensor_enable(ds[0], TIME_STEP);
  ds[1] = wb_robot_get_device("ds_exit");
  wb_distance_sensor_enable(ds[1], TIME_STEP);

  double ds_value[2] = {0.0, 0.0}; 

  wb_motor_set_position(door, INFINITY);
  wb_motor_set_velocity(door, 0.0);
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  int flagOpened = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    
    /* 
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    for (i = 0; i<SIDES; i++) {
      ds_value[i] = wb_distance_sensor_get_value(ds[i]);
    }
    //printf("\n Door values entrance %g exit %g", ds_value[0], ds_value[1]);
    //printf("\n");
    /* Process sensor data here */
    if ((flagOpened == 0) && ((ds_value[0] > 500) || (ds_value[1] > 500))) {
      openDoor(UP);
      flagOpened = 1;
    } else if (flagOpened) {
      wait(120);
      openDoor(DOWN);
      flagOpened = 0;
    }
    
    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_door_set_speed(100.0,100.0);
     */
  };
  
  /* Enter your cleanup code here */
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  
  return 0;
}


int openDoor(float upDown){
  int i;
//  wb_motor_set_position(door, 10);
  wb_motor_set_velocity(door, upDown);
  for (i = 0; i < 3; i++) {
    //printf("\n Door opening");
    wb_robot_step(TIME_STEP);
  }
  wb_motor_set_velocity(door, 0.0);
  wb_robot_step(TIME_STEP);  
  return 0;
}

void wait(int steps){
  while (steps > 0) {
    steps--;
    wb_robot_step(TIME_STEP); //waiting
  } 
}


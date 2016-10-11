#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/led.h>

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <windows.h>

#define TIME_STEP 64
#define NB_DIST_SENS 8
#define CALIBRATE 50
#define HEXBLACK 0x000000
#define NB_LEDS 10

void initSensors(WbDeviceTag *sensors){
  int i;
  char textRobotps[4] = "ps0";
  for (i=0;i<NB_DIST_SENS; i++) {
    sensors[i] = wb_robot_get_device(textRobotps);
    textRobotps[2]++;
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }
}

void calibrateSensors(WbDeviceTag *sensors, int *ps_offset){
  int i, k;
  for (k = 0; k <CALIBRATE; k++){
    for (i=0; i<NB_DIST_SENS; i++){
      ps_offset[i] += (int) wb_distance_sensor_get_value(sensors[i]);
    }
    wb_robot_step(TIME_STEP);
  } 
  printf("\n Calibration offset ");
  for (i=0; i<NB_DIST_SENS; i++){
    ps_offset[i] /= CALIBRATE-1;
    printf("%d ", ps_offset[i]);
  }  
}

void initLeds(WbDeviceTag *leds){
  char text[5] = "led0"; 
  int i;
  for (i=0; i<NB_LEDS; i++) {
    leds[i] = wb_robot_get_device(text); 
    text[3]++; 
  }
}

void initCamera(WbDeviceTag *cam, unsigned short *width, unsigned short *height){
  *cam = wb_robot_get_device("camera");
  wb_camera_enable((WbDeviceTag) *cam, TIME_STEP);
  *width = wb_camera_get_width((WbDeviceTag) *cam);
  *height = wb_camera_get_height((WbDeviceTag) *cam);  
}	

void resetDisplay(WbDeviceTag *displayExtra, unsigned short width, unsigned short height){ //ok
  *displayExtra = wb_robot_get_device("displayExtra");
  wb_display_set_color((WbDeviceTag)*displayExtra, HEXBLACK);
  wb_display_draw_rectangle((WbDeviceTag)*displayExtra, 0, 0, width, height);
}

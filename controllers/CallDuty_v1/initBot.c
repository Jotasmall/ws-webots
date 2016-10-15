#include "initBot.h"

#define TIME_STEP 64
#define NB_DIST_SENS 8
#define CALIBRATE 50
#define HEXBLACK 0x000000
#define NB_LEDS 10

void resetDevices(struct robotDevices *bot){
  int i;
  // get distance sensors
  char textSensors[4] = "ps0";
  for (i = 0; i < NB_DIST_SENS; i++) {
    bot->sensors[i] = wb_robot_get_device(textSensors);
    textSensors[2]++;
    wb_distance_sensor_enable(bot->sensors[i], TIME_STEP);
  }
  // get camera
  bot->cam = wb_robot_get_device("camera");
  wb_camera_enable(bot->cam, TIME_STEP);
  bot->width = wb_camera_get_width(bot->cam);
  bot->height = wb_camera_get_height(bot->cam);
  // enabling encoders
  wb_differential_wheels_enable_encoders(TIME_STEP);
  wb_differential_wheels_set_encoders(0,0);
  // get leds
  char text[5] = "led0"; 
  for (i=0; i<NB_LEDS; i++) {
    bot->leds[i] = wb_robot_get_device(text); 
    text[3]++; 
  }
  // communication module
  bot->receiver = wb_robot_get_device("receiver");
  bot->emitter = wb_robot_get_device("emitter");
  wb_receiver_enable(bot->receiver,TIME_STEP);
}

void calibrateSensors(struct robotDevices *bot){
  int i, k;
  for (k = 0; k <CALIBRATE; k++){
    for (i=0; i<NB_DIST_SENS; i++){
      bot->ps_offset[i] += (int) wb_distance_sensor_get_value(bot->sensors[i]);
    }
    wb_robot_step(TIME_STEP);
  } 
  //printf("\n Calibration offset ");
  for (i=0; i<NB_DIST_SENS; i++){
    bot->ps_offset[i] /= CALIBRATE-1;
    //printf("%d ", ps_offset[i]);
  }  
}

void resetDisplay(WbDeviceTag *displayExtra, unsigned short width, unsigned short height){ //ok
  *displayExtra = wb_robot_get_device("displayExtra");
  wb_display_set_color((WbDeviceTag)*displayExtra, HEXBLACK);
  wb_display_draw_rectangle((WbDeviceTag)*displayExtra, 0, 0, width, height);
}

void initEstimations(struct robotEstimations *bot, int nRegions){ //ok-
  // rand() % (max_n - min_n + 1) + min_n;
  bot->estPickS = rand() % (500-200+1)+200; 
  bot->estDropN = rand() % (500-200+1)+200;
  bot->estTravelGrey = rand() % (1000-350+1)+350;
  bot->estTravelRed = rand() % (1000-350+1)+350;
  bot->estTravelBlue = rand() % (1000-350+1)+350;
  //bot->timeImage = 0;
  //bot->timeMeasured = 0;
  //bot->timeListened = 0;  
  bot->lastImage = 0;
  memset(bot->nPick, 0, nRegions);  
  memset(bot->nDrop, 0, nRegions);    
  wb_robot_step(32);
}
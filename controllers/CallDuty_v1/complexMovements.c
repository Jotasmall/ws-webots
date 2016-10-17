#include "complexMovements.h"

#define K_TURN 4
#define TIME_STEP 64
#define THRESHOLD_DIST 150
#define LEFT 0
#define RIGHT 1
#define ROBOT_COLOR 10
#define ROBOT 306

int followingLine(double *speed, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, int colorLine, struct robotState *botState, struct robotCamera *botCam, struct robotDevices *botDevices){//ok-
  int delta = 0;
  int entering = 0;
  int flagRobot = 0;
  int nComp;
  readSensors(0, botDevices);
  entering = botDevices->ps_value[5] > 50;
  while(entering) { 
    readSensors(0, botDevices);
    if ((botDevices->ps_value[0] > THRESHOLD_DIST) || (botDevices->ps_value[7] > THRESHOLD_DIST)){ 
      waiting(20);  
      printf("\n %d something is in front of me", botState->botNumber);
      printf("\n");
    } else {
      botCam->image = wb_camera_get_image(botDevices->cam);
      // cronometer(IMAGE, 0); // Disable because it is only one row
      delta = find_middle(0, colorLine, botCam, botState);
      if ((delta > -1) && (delta < 100)) {
        delta = delta - botCam->width/2;
        speed[LEFT] = 220 - K_TURN*abs(delta);
        speed[RIGHT] = 220 - K_TURN*abs(delta);
        
        wb_differential_wheels_set_speed(speed[LEFT]+K_TURN*delta,speed[RIGHT]-K_TURN*delta);
        wb_robot_step(TIME_STEP);
        //c cronometer(-1, 0); 
      } else {
        flagRobot = check4Robot(displayExtra, shapeSeen, pointA, pointB, ROBOT_COLOR, ROBOT_COLOR, ROBOT, 0, &nComp, botCam, botDevices, botState);
        if (flagRobot) {
          waiting(30);//20
        } else {
          //printf("\n %s is lost from the line", robotName);
          //printf("\n");
          return -1; // End of travel
        }  
      }
    }  
  }  
  return 1;  
}

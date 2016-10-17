#include "complexMovements.h"

#define TIME_STEP 64

#define K_TURN 4
#define TURN_CACHE -52
#define TURN_90 27
#define TURN_M90 -27
#define SPEEDCARGO 1

#define THRESHOLD_DIST 150
#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 600
#define BACKWARD_SPEED 200
// Colors
#define RED 0
#define GREY 1
#define BLUE 2
#define CYAN 3
#define MAGENTA 4
#define BLACK 6
#define GREEN 7
#define WHITE 8
// Figures
#define BOX 301
#define TRIANGLE 302
#define CIRCLE 303
#define ALL 304
#define NOTHING 305
#define ROBOT 306
// Special variables
#define IMAGE 201
#define PICKING 0
#define DROPPING 1
#define NEST 5
#define SOURCE 10
#define PROXIMITY_COLOR 28

#define M2ROBOT 1
#define ROBOT_LEAVING 31
#define ROBOT_ARRIVING 32
#define ROBOT_UPDATING 33
#define M2NEST 2
#define LEAVE 11
#define COME 12

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
          //printf("\n %s is lost from the line", botState->botNumber);
          //printf("\n");
          return -1; // End of travel
        }  
      }
    }  
  }  
  return 1;  
}

int doorEntrance(double *speed, int steps, struct robotDevices *botDevices, struct flags4Files *botFlags, struct robotState *botState){
  forward(10, speed);
  turnSteps(TURN_M90, speed);
  readSensors(0, botDevices);
  if ((botDevices->ps_value[0] > THRESHOLD_DIST) || (botDevices->ps_value[7]> THRESHOLD_DIST)) {
    printf("\n %d wrong turn", botState->botNumber);
    printf("\n");
    return 0;
  }
  forward(steps, speed);
  speaking(botDevices, botState->botNumber, M2NEST, ROBOT_LEAVING, 0, 0, botFlags); // To indicate home-nest 
  speaking(botDevices, botState->botNumber, -1, ROBOT_LEAVING, 0, 0, botFlags); // To indicate friends 
  waiting(1);
  turnSteps(-10, speed);
  return 1;
}

int setRobotPosition(int colorLine, double *speed, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState){
  int nComp;
  int flagRobot = check4Robot(displayExtra, shapeSeen, pointA, pointB, ROBOT_COLOR, ROBOT_COLOR, ROBOT, 0, &nComp, botCam, botDevices, botState);
  // She saw a robot or not Cyan color in front
  if (flagRobot) {
    //printf("\n False Cyan landmark, %s", botState->botNumber);
    forward(-20, speed);
    return 0;
  }
  readSensors(0, botDevices);
  // hit by sensor 1, turn almost 20 degrees
  if ((botDevices->ps_value[0] > THRESHOLD_DIST) || (botDevices->ps_value[1] > THRESHOLD_DIST)) { turnSteps(10, speed);} 
  speed[LEFT] = 100;
  speed[RIGHT] = -100;
  int notReady = 1;
  int wrongDoor = 0;
  int counter = 0, aux;
  //printf("\n %s is looking for line of color %d", botState->botNumber, colorLine);
  //printf("\n");
  while(notReady) { 
    wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);      
    readSensors(0, botDevices);
    counter++;
    if (botDevices->ps_value[5]> 300) {
      notReady = find_middle(0, colorLine, botCam, botState) < 0; // returns the index -1 if not
      wrongDoor = find_middle(1, colorLine, botCam, botState) > 0; // return 100 if it found it
      flagRobot = check4Robot(displayExtra, shapeSeen, pointA, pointB, ROBOT_COLOR, ROBOT_COLOR, ROBOT, 0, &nComp, botCam, botDevices, botState);
      aux = counter;
      while (flagRobot) {
        flagRobot = check4Robot(displayExtra, shapeSeen, pointA, pointB, ROBOT_COLOR, ROBOT_COLOR, ROBOT, 0, &nComp, botCam, botDevices, botState);
        printf("\n %d waiting for another robot to leave", botState->botNumber);
        printf("\n");
        waiting(20);
        counter++;
        if (counter > 90) {
          printf("\n %d wait for an entire turn and no free way", botState->botNumber);
          printf("\n");
          return -1;
        } else if (flagRobot == 0) {
          counter = aux;
          printf("\n %d has a clear way", botState->botNumber);
          printf("\n");
        }
      }
      if (wrongDoor) {
        return -2;
      } 
    } else {
      flagRobot = check4Robot(displayExtra, shapeSeen, pointA, pointB, ROBOT_COLOR, ROBOT_COLOR, ROBOT, 0, &nComp, botCam, botDevices, botState);
      if (flagRobot) {
        printf("\n %d find another robot here",botState->botNumber);
        printf("\n");
        return -1;
      }
      if (counter > 60) {
        printf("\n %d gave a entire turn and no line", botState->botNumber);
        printf("\n");
        return -1;
      }
    }
    //c cronometer(-1, 0); 
  } 
  return 0;
}

int going2region(int color, int colorLine, int colorDestination, double *speed, WbDeviceTag *displayExtra, struct robotCamera *botCam, int *shapeSeen, int *pointA, int *pointB, struct robotDevices *botDevices, struct robotState *botState, struct flags4Files *botFlags){ //ok
  int endTask = 0, i;
  resetDisplay(displayExtra, botCam->width, botCam->height);
  //printf("\n %s getting in position destination %d by line of color %d", botState->botNumber, colorDestination, colorLine);
  //printf("\n");
  endTask = setRobotPosition(colorLine, speed, displayExtra, shapeSeen, pointA, pointB, botCam, botDevices, botState);
  if (endTask == -1) { // found no line
    //while(!run(flagLoad, 50)); //60
	for (i = 0; i<10; i++) {
		run(botState->flagLoad, 5, speed, botDevices);
		whereIam(1, color, speed, botCam, botDevices, botState);
	}
    return 0;
  } else if (endTask == -2) { //found another color
    turnSteps(15, speed);
    //while(!run(flagLoad, 60));
	for (i = 0; i<12; i++) {
		run(botState->flagLoad, 5, speed, botDevices);
		whereIam(1, color, speed, botCam, botDevices, botState);
	}
    return 0;
  }
//  endTask = followingLine(colorLine);
  endTask = followingLine(speed, displayExtra, shapeSeen, pointA, pointB, colorLine, botState, botCam, botDevices);

  if (endTask == -1) { //End of travel
    //printf("\n Robot %s going inside", botState->botNumber);
    //printf("\n");
    endTask = doorEntrance(speed, 60, botDevices, botFlags, botState); 
    if (endTask == 0) {
      forward(-20, speed);
      return 0;
    }
    whereArrive(color, speed, botDevices, botCam, botState, botFlags);
    run(botState->flagLoad, 5, speed, botDevices);
	whereIam(1, color, speed, botCam, botDevices, botState);
	run(botState->flagLoad, 5, speed, botDevices);
	whereIam(1, color, speed, botCam, botDevices, botState);
    if (botState->floorColor == colorDestination) {
      //printf("\n Excellent entrance, %s is on desired region", botState->botNumber);
      //printf("\n");
      return 1;
    } else {
      //printf("\n Something went wrong, please %s recheck color destination %d", botState->botNumber, colorDestination);
      //printf("\n");
      return 0;
    } 
  }
  return 1000;  
}

int going2it(int index,int color, double *speed, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState){//ok
  int intensity[botCam->width]; //int *intensity = (int *)malloc(sizeof(int)*width);
  int i = 0, index2 = 0, delta = 0;
  int count = 0;
  int flagRobot = 0;

  botCam->image = wb_camera_get_image(botDevices->cam);
  wb_robot_step(TIME_STEP);
  //c cronometer(IMAGE, 0);
  
  if (index == 100) {
    for (i = 0; i < botCam->width; i++) {
      intensity[i] = cont_height_figure(i, color, botCam, botState);
    }
    for (i = 0; i < botCam->width; i++) {
      if (count < intensity[i]) {
        count = intensity[i];
        index = i;
        index2 = i;
      } else if (intensity[i] < count) {
        break; // to avoid same height in different squares 
      } else if (count == intensity[i]) { //it is a square
        index2 = i;
      }
    }
    if (index2 > index) {
      delta = index + (index2-index)/2 - (botCam->width/2);
      index = (botCam->width/2) + delta;
    } else {

      delta = index - (botCam->width/2);
    }  
  } else if ((index >= 0) && (index < botCam->width)) {
    delta = index-(botCam->width/2);
  } else {
    printf("\n no direction defined for %d", botState->botNumber);
    printf("\n");
    return 0;
  }
  count = cont_height_figure(index, color, botCam, botState);
  //printf("\n According to direction defined %d by %s the height is %d", index, botState->botNumber, count);
  int iter=0;
  if ((index >= 0) && (index < botCam->height)) {
    iter = count-6;
  } else {
    iter = (MAX_SPEED*botCam->height)/(MAX_SPEED+BACKWARD_SPEED);
  } // increase by 1.25 max_speed
  speed[LEFT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/botCam->height;
  speed[RIGHT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/botCam->height;
  // The robot is close enough to the object, i.e., > 75%  
  if (count > PROXIMITY_COLOR) {
    resetDisplay(displayExtra, botCam->width, botCam->height);
    flagRobot = check4Robot(displayExtra, shapeSeen, pointA, pointB,   color, ROBOT_COLOR, ROBOT, 0, &iter, botCam, botDevices, botState);

    if (color == CYAN){   
      if (flagRobot) { 
        forward(-15, speed);
        // printf("\n %d found a robot when going to Landmark", botState->botNumber); //-- JUAN EDIT
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      detectImage(displayExtra, shapeSeen, pointA, pointB, color, CYAN, BOX, 255, &iter, botCam, botDevices, botState);
      //printf("\n Difference between A-B %d", pointA-pointB);
      iter = pointA - pointB;
      if (iter > 2) { hitWall(5, speed, botDevices);}
      else if (iter < -2) { hitWall(-5, speed, botDevices);}
      else { hitWall(0, speed, botDevices);}
   
      //printf("\n %d reached cyan landmark!", botState->botNumber);
      //printf("\n");
      waiting(1);      
      return 1;
    } else {
      //printf("\n Robot %d is near but...", botState->botNumber);
      if (flagRobot) {
        //printf("\n %d found another robot there", botState->botNumber);
        forward(-5, speed);       
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      forward(20, speed);
      hitWall(1, speed, botDevices);
      forward(-7, speed);
      delta = botDevices->ps_value[0] + botDevices->ps_value[1] - botDevices->ps_value[7] - botDevices->ps_value[6];
      if (delta > THRESHOLD_DIST) { turnSteps(3, speed);} // almost 10°
      else if (delta < THRESHOLD_DIST) { turnSteps(-3, speed);}
      /*switch(color){
        case BLUE:
          printf("\n Robot %d is by color BLUE", botState->botNumber);
          printf("\n"); break;
        case RED: 
          printf("\n Robot %d is by color RED", botState->botNumber); 
          printf("\n"); break;
        case MAGENTA: 
          printf("\n Robot %d is by color MAGENTA", botState->botNumber); 
          printf("\n"); break;
        case BLACK: 
          printf("\n Robot %d is by color BLACK", botState->botNumber); 
          printf("\n"); break;   
      }*/
      waiting(1);
      return 1;
    }   
  } else { //before being close enough
    // printf("\n %d saw shape with height %d", botState->botNumber, count);
    if (readSensors(0, botDevices) && ((botDevices->ps_value[0] > THRESHOLD_DIST) || (botDevices->ps_value[1] > THRESHOLD_DIST) 
    || (botDevices->ps_value[7] > THRESHOLD_DIST) || (botDevices->ps_value[6] > THRESHOLD_DIST))) { // 1 for obstacle
      //printf("\n %d found obstacle on the way", botState->botNumber);
      //printf("\n");
      avoidance(speed, botDevices);
    }
    
    flagRobot = check4Robot(displayExtra, shapeSeen, pointA, pointB,  ROBOT_COLOR, ROBOT_COLOR, ROBOT, 0, &iter, botCam, botDevices, botState);
    if (flagRobot) {
      //printf("\n I %d found another robot there", botState->botNumber);
      // rand() % (max_n - min_n + 1) + min_n;
      if (rand()%100 > 50) { waiting(10);}
      else { turnSteps(6, speed);}
      return 0;  
    }
    speed[LEFT] = speed[LEFT]+K_TURN*delta;
    speed[RIGHT] = speed[RIGHT]-K_TURN*delta;
    if (botState->flagLoad){ //reducing speed when cargo
      speed[LEFT]=speed[LEFT]*SPEEDCARGO;
      speed[RIGHT]=speed[RIGHT]*SPEEDCARGO;
    }
    wb_differential_wheels_set_speed(speed[LEFT]+K_TURN*delta,
                                     speed[RIGHT]-K_TURN*delta);
    wb_robot_step(TIME_STEP); 
    //c cronometer(-1, 0); //-1 for movements
  }
  return 0;
} 

int levyFlight(int figura, int color, double *speed, struct robotDevices *botDevices, struct robotCamera *botCam, struct robotState *botState, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB){

  int index = -1;
  int nComp;
  // rand() % (max_n - min_n + 1) + min_n;
  int r = rand()%(54-27+1)+27; 
  // Robot needs about 7 steps at 200 speed to have a new image
  //printf("\n Robot %s turning random %d", robotName, r); //-- JUAN EDIT
  //printf("\n"); //-- JUAN EDIT
  while (r > 0) {
    turnSteps(3, speed); // Blind turn
    r -= 3;
    
    botCam->image = wb_camera_get_image(botDevices->cam);
    wb_robot_step(TIME_STEP);
    // cronometer(IMAGE, 0) //It is only a line
        
    if (cont_height_figure(-10, color, botCam, botState) > 15) {//18
      printf("\n Backward invading useful region on turn");
      printf("\n");
      forward(-30, speed); //70
    }
    if ((color == CYAN) && (cont_height_figure(-11, color, botCam, botState) > 22)) { //25
      printf("\n Backward invading on turn %d",cont_height_figure(-11, color, botCam, botState));
      printf("\n");
      forward(-30, speed); //70
    }
    index = detectImage(displayExtra, shapeSeen, pointA, pointB, color, color, figura, 0, &nComp, botCam, botDevices, botState); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
        return doubleCheck(speed, displayExtra, shapeSeen, pointA, pointB, color, color, figura, 0, &nComp, botCam, botDevices, botState);
      }
      //--printf("\n Shape watched on levy - Levy Aborted %d", index);
      //--printf("\n");
      return index;  
    } 
    whereIam(1, color, speed, botCam, botDevices, botState);
  } 
  r = rand()%(100-40)+41; // walk forward between 100 to 40 steps
  //printf("\n %s Walking forward %d", robotName, r); //-- JUAN EDIT
  //printf("\n"); //-- JUAN EDIT
  wb_differential_wheels_set_encoders(0,0);
  while (r > 0) {
    run(botState->flagLoad, 5, speed, botDevices); // Blind walk
    r -= 5;
    whereIam(1, color, speed, botCam, botDevices, botState);

    botCam->image = wb_camera_get_image(botDevices->cam);
    wb_robot_step(TIME_STEP);
    // cronometer(IMAGE, 0) //It is only a line
        
    if (cont_height_figure(-10, color, botCam, botState    ) > 15) { //18
      //printf("\n Backward invading useful region on walk");
      //printf("\n");
      forward(-20, speed); //30
      turnSteps(TURN_CACHE, speed);
    }   
    if ((color == CYAN) && (cont_height_figure(-11, color, botCam, botState    ) > 22)) {
      //printf("\n Backward invading on walk %d", cont_height_figure(-11));
      //printf("\n");
      forward(-20, speed); //70
      turnSteps(TURN_CACHE/2, speed);
    } 
    index = detectImage(displayExtra, shapeSeen, pointA, pointB,   color, color, figura, 0, &nComp, botCam, botDevices, botState); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
        return doubleCheck(speed, displayExtra, shapeSeen, pointA, pointB, color, color, figura, 0, &nComp, botCam, botDevices, botState); 
      }
      //-- printf("\n Color watched on levy - Levy Aborted %d", index);
      //-- printf("\n");
      return index;  
    } 
  }
  return index;
}



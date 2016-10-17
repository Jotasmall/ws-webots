#include "complexMovements.h"

#define TIME_STEP 64

#define K_TURN 4
#define TURN_CACHE -52
#define TURN_90 27
#define TURN_M90 -27

#define THRESHOLD_DIST 150
#define LEFT 0
#define RIGHT 1

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
  //printf("\n %s getting in position destination %d by line of color %d", robotName, colorDestination, colorLine);
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
    //printf("\n Robot %s going inside", robotName);
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
      //printf("\n Excellent entrance, %s is on desired region", robotName);
      //printf("\n");
      return 1;
    } else {
      //printf("\n Something went wrong, please %s recheck color destination %d", robotName, colorDestination);
      //printf("\n");
      return 0;
    } 
  }
  return 1000;  
}

int going2it(int index){//ok
  int intensity[botCam.width]; //int *intensity = (int *)malloc(sizeof(int)*width);
  int i = 0, index2 = 0, delta = 0;
  int count = 0;

  image = wb_camera_get_image(botDevices.cam);
  wb_robot_step(TIME_STEP);
  cronometer(IMAGE, 0);
  
  if (index == 100) {
    for (i = 0; i < botCam.width; i++) {
      intensity[i] = cont_height_figure(i, color, &botCam, &botState    );
    }
    for (i = 0; i < botCam.width; i++) {
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
      delta = index + (index2-index)/2 - (botCam.width/2);
      index = (botCam.width/2) + delta;
    } else {

      delta = index - (botCam.width/2);
    }  
  } else if ((index >= 0) && (index < botCam.width)) {
    delta = index-(botCam.width/2);
  } else {
    printf("\n no direction defined %s", robotName);
    printf("\n");
    return 0;
  }
  int count = cont_height_figure(index, color, &botCam, &botState    );
  //printf("\n According to direction defined %d by %s the height is %d", index, robotName, count);
  int iter=0;
  if ((index >= 0) && (index < botCam.height)) {
    iter = count-6;
  } else {
    iter = (MAX_SPEED*botCam.height)/(MAX_SPEED+BACKWARD_SPEED);
  } // increase by 1.25 max_speed
  speed[LEFT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/botCam.height;
  speed[RIGHT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/botCam.height;
  // The robot is close enough to the object, i.e., > 75%  
  if (count > PROXIMITY_COLOR) {
    resetDisplay(&displayExtra, botCam.width, botCam.height);
    flagRobot = check4Robot(&displayExtra, &shapeSeen, &pointA, &pointB,   color, ROBOT_COLOR, ROBOT, 0, &iter, &botCam, &botDevices, &botState);

    if (color == CYAN){   
      if (flagRobot) { 
        forward(-15, speed);
        // printf("\n %s found a robot when going to Landmark", robotName); //-- JUAN EDIT
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      detectImage(&displayExtra, &shapeSeen, &pointA, &pointB, color, CYAN, BOX, 255, &iter, &botCam, &botDevices, &botState);
      //printf("\n Difference between A-B %d", pointA-pointB);
      iter = pointA - pointB;
      if (iter > 2) { hitWall(5, speed, &botDevices);}
      else if (iter < -2) { hitWall(-5, speed, &botDevices);}
      else { hitWall(0, speed, &botDevices);}
   
      //printf("\n %s reached cyan landmark!", robotName);
      //printf("\n");
      waiting(1);      
      return 1;
    } else {
      //printf("\n Robot %s is near but...", robotName);
      if (flagRobot) {
        //printf("\n %s found another robot there", robotName);
        forward(-5, speed);       
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      forward(20, speed);
      hitWall(1, speed, &botDevices);
      forward(-7, speed);
      delta = botDevices.ps_value[0] + botDevices.ps_value[1] - botDevices.ps_value[7] - botDevices.ps_value[6];
      if (delta > THRESHOLD_DIST) { turnSteps(3, speed);} // almost 10°
      else if (delta < THRESHOLD_DIST) { turnSteps(-3, speed);}
      /*switch(color){
        case BLUE:
          printf("\n Robot %s is by color BLUE", robotName);
          printf("\n"); break;
        case RED: 
          printf("\n Robot %s is by color RED", robotName); 
          printf("\n"); break;
        case MAGENTA: 
          printf("\n Robot %s is by color MAGENTA", robotName); 
          printf("\n"); break;
        case BLACK: 
          printf("\n Robot %s is by color BLACK", robotName); 
          printf("\n"); break;   
      }*/
      waiting(1);
      return 1;
    }   
  } else { //before being close enough
    // printf("\n %s saw shape with height %d", robotName, count);
    if (readSensors(0, &botDevices) && ((botDevices.ps_value[0] > THRESHOLD_DIST) || (botDevices.ps_value[1] > THRESHOLD_DIST) 
    || (botDevices.ps_value[7] > THRESHOLD_DIST) || (botDevices.ps_value[6] > THRESHOLD_DIST))) { // 1 for obstacle
      //printf("\n %s found obstacle on the way", robotName);
      //printf("\n");
      avoidance(speed, &botDevices);
    }
    
    flagRobot = check4Robot(&displayExtra, &shapeSeen, &pointA, &pointB,   color, ROBOT_COLOR, ROBOT, 0, &iter, &botCam, &botDevices, &botState);
    if (flagRobot) {
      //printf("\n I %s found another robot there", robotName);
      // rand() % (max_n - min_n + 1) + min_n;
      if (rand()%100 > 50) { waiting(10);}
      else { turnSteps(6, speed);}
      return 0;  
    }
    speed[LEFT] = speed[LEFT]+K_TURN*delta;
    speed[RIGHT] = speed[RIGHT]-K_TURN*delta;
    if (flagLoad){ //reducing speed when cargo
      speed[LEFT]=speed[LEFT]*SPEEDCARGO;
      speed[RIGHT]=speed[RIGHT]*SPEEDCARGO;
    }
    wb_differential_wheels_set_speed(speed[LEFT]+K_TURN*delta,
                                     speed[RIGHT]-K_TURN*delta);
    wb_robot_step(TIME_STEP); 
    cronometer(-1, 0); //-1 for movements
  }
  return 0;
} 

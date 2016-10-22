#include "complexMovements.h"

#define TIME_STEP 64

#define K_TURN 4
#define TURN_CACHE -52
#define TURN_90 27
#define TURN_M90 -27
#define SPEEDCARGO 1 
// UML states
#define PICK_SOURCE 100
#define DROP_NEST 103

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
// dsestinations
#define TRAVEL2RED 104
#define TRAVEL2GREY 105
#define TRAVEL2BLUE 106
// communications
#define M2ROBOT 1
#define ROBOT_LEAVING 31
#define ROBOT_ARRIVING 32
#define ROBOT_UPDATING 33
#define ROBOT_NEGATIVE 34
#define M2NEST 2
#define LEAVE 11
#define COME 12

#define ROBOT_COLOR 10
#define ROBOT 306

int followingLine(double *speed, WbDeviceTag *displayExtra){//ok-
  int delta = 0;
  int entering = 0;
  int flagRobot = 0;
  readSensors(0);
  entering = bot.ps_value[5] > 50;
  while(entering) { 
    readSensors(0);
    if ((bot.ps_value[0] > THRESHOLD_DIST) || (bot.ps_value[7] > THRESHOLD_DIST)){ 
      waiting(20);  
      //f printf("\n %d something is in front of me", bot.botNumber);
      //f printf("\n");
    } else {
      bot.image = wb_camera_get_image(bot.cam);
      delta = find_middle(0);
      if ((delta > -1) && (delta < 100)) {
        delta = delta - bot.width/2;
        speed[LEFT] = 220 - K_TURN*abs(delta);
        speed[RIGHT] = 220 - K_TURN*abs(delta);
        wb_differential_wheels_set_speed(speed[LEFT]+K_TURN*delta,speed[RIGHT]-K_TURN*delta);
        wb_robot_step(TIME_STEP);
        cronometer(-1, 0); 
      } else {
        flagRobot = check4Robot(displayExtra);
        if (flagRobot) {
          waiting(30);//20
        } else {
          //printf("\n %s is lost from the line", bot.botNumber);
          //printf("\n");
          return -1; // End of travel
        }  
      }
    }  
  }  
  return 1;  
}

int doorEntrance(double *speed, int steps){
  forward(10, speed);
  turnSteps(TURN_M90, speed);
  readSensors(0);
  if ((bot.ps_value[0] > THRESHOLD_DIST) || (bot.ps_value[7]> THRESHOLD_DIST)) {
     printf("\n %d wrong turn", bot.botNumber);
     printf("\n");
     return 0;
  }
  forward(steps, speed);
  speaking(M2NEST, ROBOT_LEAVING, 0, 0); // To indicate home-nest 
  speaking(    -1, ROBOT_LEAVING, 0, 0); // To indicate friends 
  waiting(1);
  //turnSteps(-10, speed);
  printf("\n robot %d is traveling to %d with floorColor %d", bot.botNumber, bot.currentState, bot.floorColor);
  if (((bot.currentState == TRAVEL2GREY) && (bot.floorColor == BLUE)) ||
     ((bot.currentState == TRAVEL2RED)  && (bot.floorColor == GREY))){ 
     turnSteps(TURN_90, speed); 
  }	else {
    turnSteps(TURN_M90, speed);	  
  }  
  return 1;
}

int setRobotPosition(double *speed, WbDeviceTag *displayExtra){

  int flagRobot = check4Robot(displayExtra);
  // She saw a robot or not Cyan color in front
  if (flagRobot) {
    //printf("\n False Cyan landmark, %s", bot.botNumber);
    forward(-20, speed);
    return 0;
  }
  readSensors(0);
  // hit by sensor 1, turn almost 20 degrees
  if ((bot.ps_value[0] > THRESHOLD_DIST) || (bot.ps_value[1] > THRESHOLD_DIST)) { 
    turnSteps(10, speed);
  } 
    speed[LEFT] = 100;
    speed[RIGHT] = -100;
    int notReady = 1;
    int wrongDoor = 0;
    int counter = 0, aux;
    //printf("\n %s is looking for line of color %d", bot.botNumber, bot.lineColor);
    //printf("\n");
    while(notReady) { 
      wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);  
      readSensors(0);
      counter++;
      if (bot.ps_value[5]> 300) {
        notReady = find_middle(0) < 0; // returns the index -1 if not
        wrongDoor = find_middle(1) > 0; // return 100 if it found it
        flagRobot = check4Robot(displayExtra);
        aux = counter;
        while (flagRobot) {
          flagRobot = check4Robot(displayExtra);
          printf("\n %d waiting for another robot to leave", bot.botNumber);
          printf("\n");
          waiting(20);
          counter++;
        if (counter > 90) {
          printf("\n %d wait for an entire turn and no free way", bot.botNumber);
          printf("\n");
          return -1;
        } else if (flagRobot == 0) {
          counter = aux;
          // printf("\n %d has a clear way", bot.botNumber);
          // printf("\n");
        }
      }
      if (wrongDoor) {
        return -2;
      } 
    } else {
      flagRobot = check4Robot(displayExtra);
      if (flagRobot) {
        printf("\n %d find another robot here",bot.botNumber);
        printf("\n");
        return -1;
      }
      if (counter > 60) {
        printf("\n %d gave a entire turn and no line", bot.botNumber);
        printf("\n");
        return -1;
      }
    }
    cronometer(-1, 0); 
  } 
  return 0;
}

int going2region(double *speed, WbDeviceTag *displayExtra){ //ok
  int endTask = 0, i;
  resetDisplay(displayExtra);
  printf("\n %d getting in position destination %d by line of color %d", bot.botNumber, bot.colorDestination, bot.lineColor);
  printf("\n");
  endTask = setRobotPosition(speed, displayExtra);
  if (endTask == -1) { // found no line
    //while(!run(flagLoad, 50)); //60
	for (i = 0; i<10; i++) {
	  run(5, speed);
	  whereIam(1, speed);
	}
    return 0;
  } else if (endTask == -2) { //found another color
    turnSteps(15, speed);
    //while(!run(flagLoad, 60));
	for (i = 0; i<12; i++) {
	  run(5, speed);
	  whereIam(1, speed);
	}
    return 0;
  }
  endTask = followingLine(speed, displayExtra);

  if (endTask == -1) { //End of travel
    //printf("\n Robot %s going inside", bot.botNumber);
    //printf("\n");
    endTask = doorEntrance(speed, 60); 
    if (endTask == 0) {
      forward(-20, speed);
      return 0;
    }
	int aux = check4Robot(displayExtra);
	if (aux) {
		aux = check4Robot(displayExtra);
		waiting(10); 
		printf("\n %d waiting for robot move out", bot.botNumber);
	}
    whereArrive(speed);
    run(10, speed);
    if (bot.floorColor == bot.colorDestination) {
      printf("\n Excellent entrance, %d is on desired region", bot.botNumber);
      printf("\n");
      return 1;
    } else {
      printf("\n Something went wrong, please %d recheck color destination %d", bot.botNumber, bot.colorDestination);
      printf("\n");
      return 0;
    } 
  }
  return 1000;  
}

int going2it(int index, double *speed, WbDeviceTag *displayExtra){//ok
  int intensity[bot.width]; //int *intensity = (int *)malloc(sizeof(int)*width);
  int i = 0, index2 = 0, delta = 0;
  int count = 0;
  int flagRobot = 0;

  bot.image = wb_camera_get_image(bot.cam);
  wb_robot_step(TIME_STEP);
  cronometer(IMAGE, 0);
  
  if (index == 100) {
    for (i = 0; i < bot.width; i++) {
      intensity[i] = cont_height_figure(i, bot.colorSeeking);
    }
    for (i = 0; i < bot.width; i++) {
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
      delta = index + (index2-index)/2 - (bot.width/2);
      index = (bot.width/2) + delta;
    } else {
      delta = index - (bot.width/2);
    }  
  } else if ((index >= 0) && (index < bot.width)) {
    delta = index-(bot.width/2);
  } else {
    printf("\n no direction defined for %d", bot.botNumber);
    printf("\n");
    return 0;
  }
  count = cont_height_figure(index, bot.colorSeeking);
  //printf("\n According to direction defined %d by %s the height is %d", index, bot.botNumber, count);
  int iter=0;
  if ((index >= 0) && (index < bot.height)) {
    iter = count-6;
  } else {
    iter = (MAX_SPEED*bot.height)/(MAX_SPEED+BACKWARD_SPEED);
  } 
  // increase by 1.25 max_speed
  speed[LEFT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/bot.height;
  speed[RIGHT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/bot.height;
  // The robot is close enough to the object, i.e., > 75%  
  if (count > PROXIMITY_COLOR) {
    resetDisplay(displayExtra);
    flagRobot = check4Robot(displayExtra);
    if (bot.colorSeeking == CYAN){ 
      if (flagRobot) { 
        forward(-15, speed);
        // printf("\n %d found a robot when going to Landmark", bot.botNumber); //-- JUAN EDIT
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      i = bot.colorSeeking;
      index2 = bot.shapeLooking;
      bot.colorSeeking = CYAN;
      bot.shapeLooking = 255;
      detectImage(displayExtra);
      bot.colorSeeking = i;
      bot.shapeLooking = index2;
      //printf("\n Difference between A-B %d", pointA-pointB);
      iter = bot.pointA - bot.pointB;
      if (iter > 2) { hitWall(5, speed);}
      else if (iter < -2) { hitWall(-5, speed);}
      else { hitWall(0, speed);}
 
      printf("\n %d reached cyan landmark!", bot.botNumber);
      printf("\n");
      waiting(1);  
	  if ((bot.flagCommanded == 1) && (bot.suggestedState != bot.currentState)
	    && ((bot.currentState == PICK_SOURCE) || (bot.currentState == DROP_NEST))) {
            bot.suggestedState = bot.currentState;
            bot.flagCommanded = 0;
	    speaking(M2NEST, ROBOT_NEGATIVE, 0, 0);
	  }
      return 1;
    } else {
      printf("\n Robot %d is near but...", bot.botNumber);
      if (flagRobot) {
        //printf("\n %d found another robot there", bot.botNumber);
        forward(-5, speed);   
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      forward(20, speed);
      hitWall(1, speed);
      forward(-7, speed);
      delta = bot.ps_value[0] + bot.ps_value[1] - bot.ps_value[7] - bot.ps_value[6];
      if (delta > THRESHOLD_DIST) { turnSteps(3, speed);} // almost 10°
      else if (delta < THRESHOLD_DIST) { turnSteps(-3, speed);}
      waiting(1);
	  if ((bot.flagCommanded == 1) && (bot.suggestedState != bot.currentState)
	    && ((bot.currentState == PICK_SOURCE) || (bot.currentState == DROP_NEST))) {
            bot.suggestedState = bot.currentState;
            bot.flagCommanded = 0;
	    speaking(M2NEST, ROBOT_NEGATIVE, 0, 0);
	  }
      return 1;
    } 
  } else { //before being close enough
    // printf("\n %d saw shape with height %d", bot.botNumber, count);
    if (readSensors(0) && ((bot.ps_value[0] > THRESHOLD_DIST) || (bot.ps_value[1] > THRESHOLD_DIST) 
        || (bot.ps_value[7] > THRESHOLD_DIST) || (bot.ps_value[6] > THRESHOLD_DIST))) { // 1 for obstacle
      //printf("\n %d found obstacle on the way", bot.botNumber);
      //printf("\n");
      avoidance(speed);
    }
    flagRobot = check4Robot(displayExtra);
    if (flagRobot) {
      //printf("\n I %d found another robot there", bot.botNumber);
      // rand() % (max_n - min_n + 1) + min_n;
      if (rand()%100 > 50) { waiting(10);}
      else { turnSteps(6, speed);}
      return 0;  
    }
    speed[LEFT] = speed[LEFT]+K_TURN*delta;
    speed[RIGHT] = speed[RIGHT]-K_TURN*delta;
    if (bot.flagLoad){ //reducing speed when cargo
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

int levyFlight(double *speed, WbDeviceTag *displayExtra){

  int index = -1;
  // rand() % (max_n - min_n + 1) + min_n;
  int r = rand()%(54-27+1)+27; 
  // Robot needs about 7 steps at 200 speed to have a new image
  while (r > 0) {
    turnSteps(3, speed); // Blind turn
    r -= 3;
    bot.image = wb_camera_get_image(bot.cam);
    wb_robot_step(TIME_STEP);   
    if (cont_height_figure(-10, bot.colorSeeking) > 15) {//18
      //printf("\n Backward %d invading useful region on turn", bot.botNumber);
      //printf("\n");
      forward(-30, speed); //70
    }
    if ((bot.colorSeeking == CYAN) && (cont_height_figure(-11, bot.colorSeeking) > 22)) { //25
      //printf("\n Backward %d invading on turn when CYAN", bot.botNumber);
      //printf("\n");
      forward(-30, speed); //70
    }
    index = detectImage(displayExtra); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
        return doubleCheck(speed, displayExtra);
      }
      //--printf("\n Shape watched on levy - Levy Aborted %d", index);
      //--printf("\n");
      return index;  
    } 
    whereIam(1, speed);
  } 
  r = rand()%(100-40)+41; // walk forward between 100 to 40 steps
  wb_differential_wheels_set_encoders(0,0);
  while (r > 0) {
    run(5, speed); // Blind walk
    r -= 5;
    whereIam(1, speed);
    bot.image = wb_camera_get_image(bot.cam);
    wb_robot_step(TIME_STEP);     
    if (cont_height_figure(-10, bot.colorSeeking) > 15) { //18
      //printf("\n Backward %d invading useful region on walk", bot.botNumber);
      //printf("\n");
      forward(-20, speed); //30
      turnSteps(TURN_CACHE, speed);
    } 
    if ((bot.colorSeeking == CYAN) && (cont_height_figure(-11, bot.colorSeeking) > 22)) {
      //printf("\n Backward %d invading on walk by CYAN", bot_>botNumber);
      //printf("\n");
      forward(-20, speed); //70
      turnSteps(TURN_CACHE/2, speed);
    } 
    index = detectImage(displayExtra); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
         return doubleCheck(speed, displayExtra); 
      }
      //-- printf("\n Color watched on levy - Levy Aborted %d", index);
      //-- printf("\n");
      return index;  
    } 
  }
  return index;
}



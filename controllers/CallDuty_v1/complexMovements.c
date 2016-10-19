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

int followingLine(double *speed, WbDeviceTag *displayExtra, struct robot *bot){//ok-
  int delta = 0;
  int entering = 0;
  int flagRobot = 0;
  readSensors(0, bot);
  entering = bot->ps_value[5] > 50;
  while(entering) { 
    readSensors(0, bot);
    if ((bot->ps_value[0] > THRESHOLD_DIST) || (bot->ps_value[7] > THRESHOLD_DIST)){ 
      waiting(20, bot);  
      printf("\n %d something is in front of me", bot->botNumber);
      printf("\n");
    } else {
      bot->image = wb_camera_get_image(bot->cam);
      delta = find_middle(0, bot);
      if ((delta > -1) && (delta < 100)) {
        delta = delta - bot->width/2;
        speed[LEFT] = 220 - K_TURN*abs(delta);
        speed[RIGHT] = 220 - K_TURN*abs(delta);
        wb_differential_wheels_set_speed(speed[LEFT]+K_TURN*delta,speed[RIGHT]-K_TURN*delta);
        wb_robot_step(TIME_STEP);
        cronometer(-1, 0, bot); 
      } else {
        flagRobot = check4Robot(displayExtra, bot);
        if (flagRobot) {
          waiting(30, bot);//20
        } else {
          //printf("\n %s is lost from the line", bot->botNumber);
          //printf("\n");
          return -1; // End of travel
        }  
      }
    }  
  }  
  return 1;  
}

int doorEntrance(double *speed, int steps, struct robot *bot){
  forward(10, speed, bot);
  turnSteps(TURN_M90, speed, bot);
  readSensors(0, bot);
  if ((bot->ps_value[0] > THRESHOLD_DIST) || (bot->ps_value[7]> THRESHOLD_DIST)) {
     printf("\n %d wrong turn", bot->botNumber);
     printf("\n");
     return 0;
  }
  forward(steps, speed, bot);
  speaking(M2NEST, ROBOT_LEAVING, 0, 0, bot); // To indicate home-nest 
  speaking(    -1, ROBOT_LEAVING, 0, 0, bot); // To indicate friends 
  waiting(1, bot);
  turnSteps(-10, speed, bot);
  return 1;
}

int setRobotPosition(double *speed, WbDeviceTag *displayExtra, struct robot *bot){

  int flagRobot = check4Robot(displayExtra, bot);
  // She saw a robot or not Cyan color in front
  if (flagRobot) {
    //printf("\n False Cyan landmark, %s", bot->botNumber);
    forward(-20, speed, bot);
    return 0;
  }
  readSensors(0, bot);
  // hit by sensor 1, turn almost 20 degrees
  if ((bot->ps_value[0] > THRESHOLD_DIST) || (bot->ps_value[1] > THRESHOLD_DIST)) { 
    turnSteps(10, speed, bot);
  } 
    speed[LEFT] = 100;
    speed[RIGHT] = -100;
    int notReady = 1;
    int wrongDoor = 0;
    int counter = 0, aux;
    //printf("\n %s is looking for line of color %d", bot->botNumber, bot->lineColor);
    //printf("\n");
    while(notReady) { 
      wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);  
      readSensors(0, bot);
      counter++;
      if (bot->ps_value[5]> 300) {
        notReady = find_middle(0, bot) < 0; // returns the index -1 if not
        wrongDoor = find_middle(1, bot) > 0; // return 100 if it found it
        flagRobot = check4Robot(displayExtra, bot);
        aux = counter;
        while (flagRobot) {
          flagRobot = check4Robot(displayExtra, bot);
          printf("\n %d waiting for another robot to leave", bot->botNumber);
          printf("\n");
          waiting(20, bot);
          counter++;
        if (counter > 90) {
          printf("\n %d wait for an entire turn and no free way", bot->botNumber);
          printf("\n");
          return -1;
        } else if (flagRobot == 0) {
          counter = aux;
          printf("\n %d has a clear way", bot->botNumber);
          printf("\n");
        }
      }
      if (wrongDoor) {
        return -2;
      } 
    } else {
      flagRobot = check4Robot(displayExtra, bot);
      if (flagRobot) {
        printf("\n %d find another robot here",bot->botNumber);
        printf("\n");
        return -1;
      }
      if (counter > 60) {
        printf("\n %d gave a entire turn and no line", bot->botNumber);
        printf("\n");
        return -1;
      }
    }
    cronometer(-1, 0, bot); 
  } 
  return 0;
}

int going2region(double *speed, WbDeviceTag *displayExtra, struct robot *bot){ //ok
  int endTask = 0, i;
  resetDisplay(displayExtra, bot);
  printf("\n %d getting in position destination %d by line of color %d", bot->botNumber, bot->colorDestination, bot->lineColor);
  printf("\n");
  endTask = setRobotPosition(speed, displayExtra, bot);
  if (endTask == -1) { // found no line
    //while(!run(flagLoad, 50, bot)); //60
	for (i = 0; i<10; i++) {
	  run(5, speed, bot);
	  whereIam(1, speed, bot);
	}
    return 0;
  } else if (endTask == -2) { //found another color
    turnSteps(15, speed, bot);
    //while(!run(flagLoad, 60, bot));
	for (i = 0; i<12; i++) {
	  run(5, speed, bot);
	  whereIam(1, speed, bot);
	}
    return 0;
  }
  endTask = followingLine(speed, displayExtra, bot);

  if (endTask == -1) { //End of travel
    //printf("\n Robot %s going inside", bot->botNumber);
    //printf("\n");
    endTask = doorEntrance(speed, 60, bot); 
    if (endTask == 0) {
      forward(-20, speed, bot);
      return 0;
    }
	int aux = check4Robot(displayExtra, bot);
	if (aux) {
		aux = check4Robot(displayExtra, bot);
		waiting(10, bot); 
		printf("\n %d waiting for robot move out", bot->botNumber);
	}
    whereArrive(speed, bot);
    run(10, speed, bot);
    if (bot->floorColor == bot->colorDestination) {
      printf("\n Excellent entrance, %d is on desired region", bot->botNumber);
      printf("\n");
      return 1;
    } else {
      printf("\n Something went wrong, please %d recheck color destination %d", bot->botNumber, bot->colorDestination);
      printf("\n");
      return 0;
    } 
  }
  return 1000;  
}

int going2it(int index, double *speed, WbDeviceTag *displayExtra, struct robot *bot){//ok
  int intensity[bot->width]; //int *intensity = (int *)malloc(sizeof(int)*width);
  int i = 0, index2 = 0, delta = 0;
  int count = 0;
  int flagRobot = 0;

  bot->image = wb_camera_get_image(bot->cam);
  wb_robot_step(TIME_STEP);
  cronometer(IMAGE, 0, bot);
  
  if (index == 100) {
    for (i = 0; i < bot->width; i++) {
      intensity[i] = cont_height_figure(i, bot->colorSeeking, bot);
    }
    for (i = 0; i < bot->width; i++) {
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
      delta = index + (index2-index)/2 - (bot->width/2);
      index = (bot->width/2) + delta;
    } else {
      delta = index - (bot->width/2);
    }  
  } else if ((index >= 0) && (index < bot->width)) {
    delta = index-(bot->width/2);
  } else {
    printf("\n no direction defined for %d", bot->botNumber);
    printf("\n");
    return 0;
  }
  count = cont_height_figure(index, bot->colorSeeking, bot);
  //printf("\n According to direction defined %d by %s the height is %d", index, bot->botNumber, count);
  int iter=0;
  if ((index >= 0) && (index < bot->height)) {
    iter = count-6;
  } else {
    iter = (MAX_SPEED*bot->height)/(MAX_SPEED+BACKWARD_SPEED);
  } 
  // increase by 1.25 max_speed
  speed[LEFT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/bot->height;
  speed[RIGHT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/bot->height;
  // The robot is close enough to the object, i.e., > 75%  
  if (count > PROXIMITY_COLOR) {
    resetDisplay(displayExtra, bot);
    flagRobot = check4Robot(displayExtra, bot);
    if (bot->colorSeeking == CYAN){ 
      if (flagRobot) { 
        forward(-15, speed, bot);
        // printf("\n %d found a robot when going to Landmark", bot->botNumber); //-- JUAN EDIT
        waiting(10, bot);
        flagRobot = 0;
        return 0;
      }
      i = bot->colorSeeking;
      index2 = bot->shapeLooking;
      bot->colorSeeking = CYAN;
      bot->shapeLooking = 255;
      detectImage(displayExtra, bot);
      bot->colorSeeking = i;
      bot->shapeLooking = index2;
      //printf("\n Difference between A-B %d", pointA-pointB);
      iter = bot->pointA - bot->pointB;
      if (iter > 2) { hitWall(5, speed, bot);}
      else if (iter < -2) { hitWall(-5, speed, bot);}
      else { hitWall(0, speed, bot);}
 
      //printf("\n %d reached cyan landmark!", bot->botNumber);
      //printf("\n");
      waiting(1, bot);  
      return 1;
    } else {
      //printf("\n Robot %d is near but...", bot->botNumber);
      if (flagRobot) {
        //printf("\n %d found another robot there", bot->botNumber);
        forward(-5, speed, bot);   
        waiting(10, bot);
        flagRobot = 0;
        return 0;
      }
      forward(20, speed, bot);
      hitWall(1, speed, bot);
      forward(-7, speed, bot);
      delta = bot->ps_value[0] + bot->ps_value[1] - bot->ps_value[7] - bot->ps_value[6];
      if (delta > THRESHOLD_DIST) { turnSteps(3, speed, bot);} // almost 10°
      else if (delta < THRESHOLD_DIST) { turnSteps(-3, speed, bot);}
      waiting(1, bot);
      return 1;
    } 
  } else { //before being close enough
    // printf("\n %d saw shape with height %d", bot->botNumber, count);
    if (readSensors(0, bot) && ((bot->ps_value[0] > THRESHOLD_DIST) || (bot->ps_value[1] > THRESHOLD_DIST) 
        || (bot->ps_value[7] > THRESHOLD_DIST) || (bot->ps_value[6] > THRESHOLD_DIST))) { // 1 for obstacle
      //printf("\n %d found obstacle on the way", bot->botNumber);
      //printf("\n");
      avoidance(speed, bot);
    }
    flagRobot = check4Robot(displayExtra, bot);
    if (flagRobot) {
      //printf("\n I %d found another robot there", bot->botNumber);
      // rand() % (max_n - min_n + 1) + min_n;
      if (rand()%100 > 50) { waiting(10, bot);}
      else { turnSteps(6, speed, bot);}
      return 0;  
    }
    speed[LEFT] = speed[LEFT]+K_TURN*delta;
    speed[RIGHT] = speed[RIGHT]-K_TURN*delta;
    if (bot->flagLoad){ //reducing speed when cargo
      speed[LEFT]=speed[LEFT]*SPEEDCARGO;
      speed[RIGHT]=speed[RIGHT]*SPEEDCARGO;
    }
    wb_differential_wheels_set_speed(speed[LEFT]+K_TURN*delta,
                                     speed[RIGHT]-K_TURN*delta);
    wb_robot_step(TIME_STEP); 
    cronometer(-1, 0, bot); //-1 for movements
  }
  return 0;
} 

int levyFlight(double *speed, WbDeviceTag *displayExtra, struct robot *bot){

  int index = -1;
  // rand() % (max_n - min_n + 1) + min_n;
  int r = rand()%(54-27+1)+27; 
  // Robot needs about 7 steps at 200 speed to have a new image
  while (r > 0) {
    turnSteps(3, speed, bot); // Blind turn
    r -= 3;
    bot->image = wb_camera_get_image(bot->cam);
    wb_robot_step(TIME_STEP);   
    if (cont_height_figure(-10, bot->colorSeeking, bot) > 15) {//18
      //printf("\n Backward %d invading useful region on turn", bot->botNumber);
      //printf("\n");
      forward(-30, speed, bot); //70
    }
    if ((bot->colorSeeking == CYAN) && (cont_height_figure(-11, bot->colorSeeking, bot) > 22)) { //25
      //printf("\n Backward %d invading on turn when CYAN", bot->botNumber);
      //printf("\n");
      forward(-30, speed, bot); //70
    }
    index = detectImage(displayExtra, bot); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
        return doubleCheck(speed, displayExtra, bot);
      }
      //--printf("\n Shape watched on levy - Levy Aborted %d", index);
      //--printf("\n");
      return index;  
    } 
    whereIam(1, speed, bot);
  } 
  r = rand()%(100-40)+41; // walk forward between 100 to 40 steps
  wb_differential_wheels_set_encoders(0,0);
  while (r > 0) {
    run(5, speed, bot); // Blind walk
    r -= 5;
    whereIam(1, speed, bot);
    bot->image = wb_camera_get_image(bot->cam);
    wb_robot_step(TIME_STEP);     
    if (cont_height_figure(-10, bot->colorSeeking, bot) > 15) { //18
      //printf("\n Backward %d invading useful region on walk", bot->botNumber);
      //printf("\n");
      forward(-20, speed, bot); //30
      turnSteps(TURN_CACHE, speed, bot);
    } 
    if ((bot->colorSeeking == CYAN) && (cont_height_figure(-11, bot->colorSeeking,  bot) > 22)) {
      //printf("\n Backward %d invading on walk by CYAN", bot_>botNumber);
      //printf("\n");
      forward(-20, speed, bot); //70
      turnSteps(TURN_CACHE/2, speed, bot);
    } 
    index = detectImage(displayExtra, bot); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
         return doubleCheck(speed, displayExtra, bot); 
      }
      //-- printf("\n Color watched on levy - Levy Aborted %d", index);
      //-- printf("\n");
      return index;  
    } 
  }
  return index;
}



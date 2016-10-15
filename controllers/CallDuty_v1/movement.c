#include "movement.h"

#define TIME_STEP 64
#define NB_DIST_SENS 8
#define SAMPLES 1
#define THRESHOLD_DIST 150

#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 600
#define TURN_90 27
#define TURN_M90 -27
#define SPEEDCARGO 1

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
  waiting(1);
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
    //c cronometer(-1, 0);
  }
  waiting(1);
}

void avoidance(double *speed, struct robotDevices *bot){ //ok
  int sense = 1;
  wb_differential_wheels_set_speed(0, 0);
  wb_robot_step(TIME_STEP); 
  readSensors(0, bot);
  if ((bot->ps_value[7] + bot->ps_value[6]) < (bot->ps_value[0] + bot->ps_value[1])) {
    sense = -1;
  }
  turnSteps(TURN_90*sense, speed);
  forward(18, speed); //15
  turnSteps((TURN_M90-3)*sense, speed);
}

int readSensors(int print, struct robotDevices *bot){ 
  int flag = 0, i, k;
  // Reset values
  for(i=0; i<NB_DIST_SENS; i++){ bot->ps_value[i] = 0;}
  //Sensor values
  for (k=0; k<SAMPLES; k++) { 
    for (i=0; i<NB_DIST_SENS; i++) {
      bot->ps_value[i] += (int)wb_distance_sensor_get_value(bot->sensors[i])-bot->ps_offset[i];
    }
    wb_robot_step(TIME_STEP); 
  }  
  for (i=0; i<NB_DIST_SENS; i++){
    bot->ps_value[i] /= SAMPLES;
    if (bot->ps_value[i] > THRESHOLD_DIST) { 
      flag = 1;
      if (print) { //Sensor 5 for follow wall
        printf("\n An obstacle is detected at sensor %d value %d", i, bot->ps_value[i]);
        printf("\n");
      }
    } 
  }
  return flag;
}

int run(int flagLoad, int steps, double *speed, struct robotDevices *bot){ //ok-
  int i, j;
  int matrix[8][2] = {{150,-35},{100, -15},{80, -10},{-10,-10},{-10,-10},{-10,80},{-30,100},{-20,150}};
  while(steps > 0) {  
    readSensors(0, bot);
    for (i = 0; i < 2; i++) {
      speed[i] = 0;
      for (j = 0; j < NB_DIST_SENS; j++) {
        // 0.002 = 1/HalfRange = 512 
        speed[i] += matrix[j] [i] * (1 - bot->ps_value[j]*0.002);
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
    //c cronometer(-1, 0); 
        
    steps--;
// 	  Every 5 steps check ground color
//c    if(steps%5 == 0){ whereIam(1);}
  } 
  waiting(1);
  return 1;
}

int hitWall(int front, double *speed, struct robotDevices *bot){ //ok
  int hit_thres = 300, flag = 1;//200
  int question;
  speed[LEFT] = 300;
  speed[RIGHT] = 300;
  if (front == 5) { // 6 is 1.5 x K_TURN
    speed[LEFT] = 300 + 30;//6*(pointA - pointB);//50
  } else if (front == -5) {
    speed[RIGHT] = 300 + 30;//6*(pointB - pointA);
  }
  while (flag) {
    readSensors(0, bot);
    //printf("\n sensors 0 %d, 1 %d, 6 %d, 7 %d", ps_value[0], ps_value[1], ps_value[6], ps_value[7]);
    if ((front == 0) || (abs(front) == 5)) {
      question = (bot->ps_value[0] > hit_thres) || (bot->ps_value[7] > hit_thres) || (bot->ps_value[6] > hit_thres) || (bot->ps_value[1] > hit_thres);
    } else {
      question = (bot->ps_value[0] > hit_thres) || (bot->ps_value[7] > hit_thres);
    }  
    if (question) {
      if (front == 5) {
        speed[LEFT] = -speed[LEFT];
        wb_robot_step(TIME_STEP);
        wb_robot_step(TIME_STEP);
      } else if (front == -5) {
        speed[RIGHT] = -speed[RIGHT];
        wb_robot_step(TIME_STEP);
        wb_robot_step(TIME_STEP);
      }
      //c waiting(2);
      readSensors(0, bot);
      if (question || (bot->ps_value[6] > hit_thres) || (bot->ps_value[1] > hit_thres)){
        forward(5, speed);
        flag = 0;
      }
    }    
    wb_differential_wheels_set_speed(speed[LEFT], speed[RIGHT]);
    wb_robot_step(TIME_STEP);
    //c cronometer(-1, 0); 
  }
  waiting(1);
  return 1;
} 

int enterTam(struct robotDevices *bot, double *speed){ //ok
  // 1 blue, 2 red, and 3 magenta
  int flag1stCheck = 1; //0 juan check 
  //1 rigth and -1 left
  int dir = 0;
  if (bot->ps_value[1] + bot->ps_value[0] > bot->ps_value[7] + bot->ps_value[6]) {
    dir = 1;
  } else {
    dir = -1;
  }
  //c flag1stCheck = detectTam(); 
  if (!flag1stCheck) { 
    turnSteps(-26*dir, speed);
    //c waiting(1);
    hitWall(1, speed, bot); 
    forward(-6, speed);//8
  }  
  if ((bot->ps_value[0] > THRESHOLD_DIST) && (bot->ps_value[7] > THRESHOLD_DIST) && (bot->ps_value[6] < THRESHOLD_DIST) && (bot->ps_value[1] < THRESHOLD_DIST)) {
    //printf("\n Everything is fine!!");
    //printf("\n");
    return 1; 
  } else {    
    //c flag1stCheck = detectTam();
    if (!flag1stCheck) {
      printf("\n not inside TAM correctly");
      printf("\n");
      waiting(10);
      return 0;
    } else {
      //printf("\n %s entered successfully square",robotName);
      waiting(1);
      return 1;  
    }  
  }
  return -1;    
}

int waiting(int n){ //ok
  wb_differential_wheels_set_speed(0,0);
  while (n > 0) {
    n--;
    wb_robot_step(TIME_STEP);
    //c cronometer(-1, 0);
  } 
  return 1;  
}

/*
 * File:  Call of Duty.c
 * Date:    
 * Description: 
 * Author: Jotasmall    
 * Modifications: 
 */
 
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <webots/led.h>
#include <webots/receiver.h>
#include <webots/emitter.h>

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <windows.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "headerStruct.h" 
#include "botStruct.h"
#include "initBot.h"
#include "dsp.h"
#include "movement.h"
#include "complexMovements.h"
#include "communication.h"
#include "readWriteFiles.h"
#include "registers.h"

#define TIME_STEP 64
#define SPEEDCARGO 1
//Model -1 is for experiments, 0 is 2011, 1 is mine, 2 is UCB, 3 is Greedy 
#define ESSAY 0
#define RANDOMLY 1
#define NEVER 2
#define MODEL 3
#define GREEDY 4
// for different models
int modelTest = NEVER;
// Flags of control
int flagFiles = 0;
int flagMasterRecruiting = 0; //1 RANDOMLY, -1 never, 0 whatever
int flagCom = 1;      //to enable or disable communications
int flagMomento = 0;
int listFriends[] = {2801,2802,2803,2804,2805,2806};
// Other flags
int flagReady = 0;      //to know when she ended a travel
int flagTravel = 0;     //to know if a robot is partitioning or not
// proximity sensors 
#define NB_DIST_SENS 8
// leds
#define NB_LEDS 10 
//cam
WbDeviceTag displayExtra;
// Figures
#define BOX 301
#define TRIANGLE 302
#define CIRCLE 303
#define ALL 304
#define NOTHING 305
#define ROBOT 306
// Colors
#define RED 0
#define DARK_RED 20
#define GREY 1
#define BLUE 2
#define DARK_BLUE 22
#define CYAN 3
#define DARK_CYAN 23
#define MAGENTA 4
#define DARK_MAGENTA 24
#define BLACK 6
#define GREEN 7
#define WHITE 8
#define TAM_WALL 9
#define ROBOT_COLOR 10

#define HEXRED 0xFF0000
#define HEXWHITE 0xFFFFFF
#define HEXBLACK 0x000000
#define HEXYELLOW 0xFFFF00
#define HEXGREEN 0x2AE246
// Movement and oddometry constants
double speed[2];
#define PI 3.1416
#define TURN_CACHE -52
// Communication 
int nRobots = 10;
#define M2ROBOT 1
#define ROBOT_LEAVING 31
#define ROBOT_ARRIVING 32
#define ROBOT_UPDATING 33
#define M2NEST 2
#define LEAVE 11
#define COME 12
// Robot files
#define FSM 0
#define ESTIMATIONS 1
#define LIFE 2
#define DECISIONS 3
#define PERFORMANCE 4 
#define COMMUNICATION 5
char robotName[11];
char fileRobot[] = "DIRPATH\\dd-hh-mm\\e-puck0000-OPTION.txt";
// Special variables
#define PICKING 0
#define DROPPING 1
#define NEST 5
#define SOURCE 10
// To write decisions
#define TRAVELING_AGREE 0
#define TRAVELING_LEVY 1
#define TRAVELING_CALL 2
// Only for UCB algorithm
#define NB_REGIONS 3
// Main core FSM
#define GO2IT 1
#define LEVY 2
#define LOST 3
#define STOP 4
#define STOP_LEVY 5
// UML states
#define PICK_SOURCE 100
#define DROP_NEST 103
#define TRAVEL2RED 104
#define TRAVEL2GREY 105
#define TRAVEL2BLUE 106
#define NONE 555
#define EXPERIMENT 666
#define WAITING 777
#define TALK_NEST 999
int statePrevious;
int output = STOP;
// Main functions
void executeUML();
int moduleUML(int foreground, int shape, int pick_or_drop, int statePartition, int stateNoPartition, int flag);
int moduleTravel();
int moduleFSM();
// Initialization functions
void initVariables();
void reset();
// Miscellaneous functions
void pickingIndication(int on);
double angle(double x, double z);
// Model functions
int computeTraveling(int levy);

char dirPath[] = "/dir-dd-hh-mm";
time_t rawtime;
struct tm * timeinfo;

//struct robot bot = {0};

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  time (&rawtime);
  timeinfo = localtime(&rawtime);
  sprintf(dirPath,"./%d-%d-%d",timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
  //printf("\n path for directories %s", dirPath);
  //printf("\n");

  reset(); 
  wb_robot_step(TIME_STEP);
  
  initVariables();   
  //printf("\n Robot %s is ready to begin with state %d", robotName,bot.currentState);
  //printf("\n");
  executeUML();
  wb_robot_cleanup();
  return 0;
}

void reset(){ //ok-
  resetDevices();
  // Parameters
  bot.alpha = 60;       //in percentage
  bot.flagMomento = flagMomento;    //to enable soft changes
  bot.beta = 10;      //soft adaptations
  bot.gammaUCB = 1000;    //in UCB-model 100/1000-Explote/Explore
  bot.greedy = 0.1;     //in e-Greedy 0.01/0.11-Explote/Explore
  bot.sParam = 1;     //in 2013 6/1-Explote/Explore
  bot.flagCom = flagCom;
  bot.flagListened = 0;
  bot.flagCommanded = 0;
  bot.listFriends = listFriends;
  // Display for user
  bot.image = wb_camera_get_image(bot.cam);
  resetDisplay(&displayExtra);
  // Getting data for initial state
  bot.floorColor = whereIam(0, speed);
  //printf("\n %s was born in region %d", wb_robot_get_name(), bot.floorColor);
  //printf("\n");
  // Create files, enable/disable files records
  bot.fileRobot = fileRobot;
  bot.dirPath = dirPath; 
  if (flagFiles) { 
    bot.flagFilesFSM = 1;
    bot.flagFilesEST = 1;
    bot.flagFilesLIFE = 1;
    bot.flagFilesDM = 1;
    bot.flagFilesPER = 1;
    bot.flagFilesCOM = 1;
    createFiles();
  } 
  initEstimations(NB_REGIONS);
  updateBitacora(0, ESTIMATIONS, 0);
  // Random seed by the number of the robot
  strcpy(robotName, wb_robot_get_name());
  bot.botNumber = atoi(&robotName[6]);
  srand(bot.botNumber*100+timeinfo->tm_mday+timeinfo->tm_hour+timeinfo->tm_min);
  wb_robot_step(TIME_STEP);
  // Adjust sensor noise by offset
  calibrateSensors();
  /*
  printf("\n Calibration offset ");
  int i;
  for (i=0; i<NB_DIST_SENS; i++){
    printf("%d ", bot.ps_offset[i]);
  } 
  */
  speaking(M2NEST, ROBOT_ARRIVING, 0, 0);
}

void initVariables(){
  if (bot.botNumber != 2701) {
    modelTest = ESSAY;
  } 
  bot.flagLoad = 0;
  bot.colorDestination = NONE;
  output = STOP;
  switch(modelTest){
  case ESSAY:
    bot.currentState = EXPERIMENT;//TRAVEL2GREY;
    bot.suggestedState = EXPERIMENT;
    bot.colorSeeking = CYAN;
    bot.shapeLooking = BOX;
  break;
  case NEVER:
    flagMasterRecruiting = -1;
    bot.currentState = PICK_SOURCE;
    bot.suggestedState = PICK_SOURCE;
    bot.colorSeeking = RED;
    bot.shapeLooking = BOX;
    if (bot.floorColor == RED) {
      bot.colorSeeking = BLUE;
    }
  break;
  case RANDOMLY:
    flagMasterRecruiting = 1;
  case MODEL:
  case GREEDY:
    bot.currentState = PICK_SOURCE;
    bot.suggestedState = PICK_SOURCE;
    bot.colorSeeking = RED;
    bot.shapeLooking = BOX;
    if (bot.floorColor == RED) {
      bot.colorSeeking = BLUE;
    }
  break;
  }
  wb_robot_step(32);  
}

void executeUML(){
  int colorSeeking = RED;
  while (wb_robot_step(TIME_STEP) != -1) {
    switch(bot.currentState){
    case EXPERIMENT:
      cronometer(1000, 0);
    break;
    case PICK_SOURCE:
      printf("\n %d state PICK_SOURCE", bot.botNumber);
      printf("\n");
      switch(bot.floorColor){
        case RED:
          colorSeeking = BLUE; break;
        case GREY:
          colorSeeking = DARK_RED; break;
        case BLUE:
          colorSeeking = RED; break;
      }
      if ((bot.flagCommanded == 1) && (bot.flagLoad == 0) && (modelTest == ESSAY)) {
        printf("\n %d from picking was commanded toward %d", bot.botNumber, bot.suggestedState);
        printf("\n");
        bot.currentState = bot.suggestedState;
        bot.suggestedState = NONE;
        bot.flagCommanded = 0;
      } else {
        bot.currentState = moduleUML(colorSeeking, BOX, PICKING, DROP_NEST, bot.colorDestination, 0);
        // avoid commands of nest when loaded
        if (bot.flagLoad) { bot.suggestedState = NONE;}  
      }  
    break;
    case DROP_NEST:
      printf("\n %d state DROP_NEST", bot.botNumber);
      printf("\n");
      if ((bot.flagCommanded == 1) && (bot.flagLoad == 0) && (modelTest == ESSAY)) {
        printf("\n %d from dropping was commanded toward %d", bot.botNumber, bot.suggestedState);
        printf("\n");
        bot.currentState = bot.suggestedState;
        bot.suggestedState = NONE;
        bot.flagCommanded = 0;
      } else {
        bot.currentState = moduleUML(MAGENTA, BOX, DROPPING, PICK_SOURCE, bot.colorDestination, 1); 
        if (bot.flagLoad) { bot.suggestedState = NONE;}
      }
    break;
    case TRAVEL2GREY:
      printf("\n %d state TRAVEL2GREY", bot.botNumber);
      printf("\n");
      bot.colorDestination = GREY;
      bot.currentState = moduleTravel();
    break;  
    case TRAVEL2BLUE:
      printf("\n %d state TRAVEL2BLUE", bot.botNumber);
      printf("\n");
      bot.colorDestination = BLUE;
      bot.currentState = moduleTravel();
      break;
    case TRAVEL2RED:
      printf("\n %d state TRAVEL2RED", bot.botNumber);
      printf("\n");
      bot.colorDestination = GREY;
      bot.currentState = moduleTravel();
      break;
    default:
      printf("\n Big failure on UML machine");
    }
  }  
}

int moduleUML(int foreground, int shape, int pick_or_drop, int stateRemain, int stateTravel, int flag) {

  bot.colorSeeking = foreground;
  bot.shapeLooking = shape;
  statePrevious = bot.currentState;
  int auxShapeSeen = 0; //when working with different shapes
  // Light indications
  if (pick_or_drop == DROPPING) { pickingIndication(1);}
  else { wb_led_set(bot.leds[8], 1);}
 
  printf("\n %s UML module has flagCommanded %d flagLoad %d and modelTest %d", robotName, bot.flagCommanded, bot.flagLoad, modelTest);
  printf("\n");
  
  output = moduleFSM();
  int nextState = stateRemain;
  int flagWait = 1;
  bot.flagLoad = flag;
  if (output == STOP) {
    while(flagWait) {
      flagWait = waiting_color();	
      if (flagWait == 0) {
        // Count only works with UCB
        countObjects(NB_REGIONS);
        forward(-120, speed);   
        turnSteps(TURN_CACHE, speed);
        updateEstimations(statePrevious, auxShapeSeen);
        bot.flagLoad = !flag;
        if (pick_or_drop == DROPPING) { pickingIndication(0);}
        else { wb_led_set(bot.leds[8], 0);}	
        break;
      }  
    }
    flagTravel = 0;//check computeTraveling(0);
    // Once a robot got an item, it has to dropped in that region
    if (flagTravel && (bot.flagLoad == 0)) { 
      float p = ((float)rand())/RAND_MAX;
      if (p > 0.5) {
        bot.colorDestination = bot.floorColor - 1;
        if (bot.colorDestination < 0) {
          bot.colorDestination = 2; // BLUE
        }
      } else {
        bot.colorDestination = bot.floorColor + 1;
        if (bot.colorDestination > 2) {
          bot.colorDestination = 0; // RED
        }
      }
      //printf("\n %s would like to travel towards %d", robotName, colorDestination);
      //nextState = stateTravel;
    }	  
  } 
  return nextState;
}

int moduleTravel(){
  int auxUML = bot.currentState; 
  bot.colorSeeking = CYAN;
  bot.shapeLooking = BOX;

  printf("\n %s travel module has flagCommanded %d flagLoad %d and modelTest %d", robotName, bot.flagCommanded, bot.flagLoad, modelTest);
  printf("\n");
  if (bot.floorColor == bot.colorDestination) {
    printf("\n %s has fulfilled its trip, stay here picking", robotName);
    printf("\n");
    return PICK_SOURCE;
  } 

  output = moduleFSM(); 
  
  if (output == STOP){  
    printf("\n %s is on region %d desiring to go to %d", robotName, bot.floorColor, bot.currentState);
    printf("\n"); 
    if (bot.currentState  == TRAVEL2GREY) {
      if (bot.floorColor == BLUE) {
        bot.lineColor = RED;
        flagReady = going2region(speed, &displayExtra);
      } else if (bot.floorColor ==  GREY) {
        flagReady = 1;
      } else if (bot.floorColor == RED){
        bot.lineColor = BLUE;
        flagReady = going2region(speed, &displayExtra);
      }  
    } else if (bot.currentState == TRAVEL2BLUE)  {
      if (bot.floorColor == GREY){
        bot.lineColor = RED;
        flagReady = going2region(speed, &displayExtra);
      } else if (bot.floorColor == BLUE) {
        flagReady = 1;
      } else if (bot.floorColor == RED){
        bot.lineColor = BLUE;
        flagReady = going2region(speed, &displayExtra);
      }
    } else if (bot.currentState == TRAVEL2RED) {
      if (bot.floorColor == GREY){
        bot.lineColor = BLUE; 
        flagReady = going2region(speed, &displayExtra);
      } else if (bot.floorColor == RED) {
        flagReady = 1;
      } else if (bot.floorColor == BLUE){
        bot.lineColor = RED; 
        flagReady = going2region(speed, &displayExtra);
      } 
    } 
    if (flagReady) { 
      printf("\n Excellent entrance, %d is on desired region", bot.botNumber);
      printf("\n");	
      auxUML = PICK_SOURCE;  
      updateEstimations(bot.currentState, 0);
      /*
        float p = ((float)rand())/RAND_MAX;
        if (p>0.5) {
          auxUML = TRAVEL2RED;
          if (RED == bot.floorColor) {
            auxUML = PICK_SOURCE;  
          }
          if (p>.66) { 
            auxUML = TRAVEL2GREY; 
            if (GREY == bot.floorColor){
              auxUML = PICK_SOURCE;
            }
            if (p>.84) { 
              auxUML = TRAVEL2BLUE; 
              if (BLUE == bot.floorColor){
                auxUML = PICK_SOURCE;
              }
            }
          }
          if (auxUML != PICK_SOURCE){
            printf("\n By luck %s is traveling to %d region", robotName, auxUML);
            printf("\n");
    	  }	  
        } else {
          printf("\n By luck %s is staying here", robotName);
          printf("\n");
          auxUML = PICK_SOURCE;
        }
      */
    } else {
    /*
     flagTravel = computeTraveling(1);
     if (flagTravel) { // Change of mind
       switch(floorColor) {
       case BLUE:
         auxUML = TRAVEL2GREY; break;
       case RED:
         auxUML = TRAVEL2GREY; break;
       case GREY:
         auxUML = TRAVEL2BLUE; break;
       }
     } else {
       auxUML = PICK_SOURCE;
     } */ 
    }
  }
  wb_robot_step(32);
  return auxUML;
}

int moduleFSM(){
  int stateFSM = LEVY;
  int newIndex = 0;
  int index = -1;
  int contLevy = 0;
  int contLost = 0;
  int contViewShape = 0;
  int flagSureSeen = 0;
  int oldShape = 0;
  int flagProximity = 0;
  int flagRobot = 0;
  int flagInside = 0;
  int moduleEnded = 0;
  output = STOP_LEVY; 
  //fsm printf("\n %s is going in FSM to search %d", robotName, bot.currentState);
  //fsm printf("\n");
  while (!moduleEnded) {
    switch(stateFSM) {
      case LEVY:
      index = levyFlight(speed, &displayExtra);
        if (index == -1) {
          contLevy++;
          //printf("\n %s is thinking about her decision", robotName);
          //printf("\n");
          switch(bot.currentState){
          /* 
          case TRAVEL2GREY:
          case TRAVEL2BLUE:
          case TRAVEL2RED:
            flagTravel  = computeTraveling(1);
            if (flagTravel) {
              //-- printf("\n %s decide to keep traveling", robotName);
            } else {
              //-- printf("\n %s cancel travel", robotName);
   	      // when the travel is finished
              updateBitacora(STOP_LEVY, FSM, 0);
              //bot.timeMeasured = 0;
              return STOP_LEVY;
            }
            break;
         */ 
          case PICK_SOURCE:
          case DROP_NEST:
            flagTravel  = computeTraveling(1);
            if (flagTravel) {
              printf("\n %s decide to change to travel", robotName);
              printf("\n");
              // when abandon a task
              updateBitacora(STOP_LEVY, FSM, 0);
              bot.timeMeasured = 0;
              return STOP_LEVY;
            } else {
              printf("\n %s decide to go on in this region", robotName);      
              printf("\n");
            }
            break;       
          }  
        } else {  //it found something
          contLevy = 0;
          updateBitacora(LEVY, FSM, 0);
          stateFSM = GO2IT; 
        }
        if (bot.currentState == PICK_SOURCE) {
          wb_led_set(bot.leds[0], 1);
        } else if (bot.currentState  == DROP_NEST){
          pickingIndication(1);
        } else {
          pickingIndication(0);
          wb_led_set(bot.leds[0], 0);
        } 
        wb_robot_step(32);
        break;  
      case GO2IT:
        //printf("\n %s found something and goes to get it", robotName);
        //printf("\n");
        flagProximity = going2it(index, speed, &displayExtra);
        if (flagProximity){
          flagProximity = 0;
          flagSureSeen = 0;      
          if ((bot.currentState == TRAVEL2BLUE) || (bot.currentState == TRAVEL2RED) || (bot.currentState == TRAVEL2GREY)) {
            flagInside = hitWall(0, speed);
          } else {
            flagInside = enterTam(speed);
          }
          if (flagInside == 1) { //sucessful enter TAM or hit wall
            stateFSM = STOP;
          } else {
            forward(-50, speed);
            stateFSM = LOST;
          }
          updateBitacora(GO2IT, FSM, 0);
        } else { //it is yet far
          newIndex = detectImage(&displayExtra);
          if (newIndex == -1) {
            flagRobot = check4Robot(&displayExtra);
            if (flagRobot) {
              printf("\n %s saw a robot in path to target", robotName);
              printf("\n");
              waiting(5);
              cronometer(-1, 0);
            } else {
              updateBitacora(GO2IT, FSM, 0);
              stateFSM = LOST;
            }
            flagSureSeen = 0;
            contViewShape = 0;
            oldShape = -1;
          } else {
            if (flagSureSeen) { index = 100;}
            else if (newIndex >= 0) { index = newIndex;}
            if (bot.shapeSeen == oldShape) { contViewShape++;}
            if ((contViewShape > 10) && (bot.nComp == 1)) {
              contViewShape = 0;
              flagSureSeen = 1;
              //printf("\n %s is sure that saw shape %d", robotName, oldShape);
              //printf("\n");
              //speaking(M2ROBOT, shapeSeen, shapeSeen, shapeSeen); //to make a rendesvouz
            }     
          }
        }  
        break;
      case LOST:
        //printf("\n %s is lost", robotName);
        run(5, speed);
        whereIam(1, speed);
        index = detectImage(&displayExtra);
        if (index >= 0) {
          contLost = 0;
          updateBitacora(LOST, FSM, 0);
          stateFSM = GO2IT;
        } else {
          contLost++;
          if (contLost > 3) {
            contLost = 0;
            updateBitacora(LOST, FSM, 0);
            stateFSM = LEVY;
          }
        }
        break;
      case STOP:
        moduleEnded = 1;
        //printf("\n %s succesfully ended the module FSM with shapeseen %d", robotName, shapeSeen);
        //printf("\n");
        updateBitacora(STOP, FSM, 0);
        return STOP;
        break;
      default:
        printf("\n %s is with mistakes in FSM module", robotName);
        return 0;  
    }
  }  
  return STOP;
}

void pickingIndication(int on){ //ok
  wb_led_set(bot.leds[1], on);
  wb_led_set(bot.leds[7], on);
}

double angle(double x, double z){ //ok
  double theta = atan(z/x);
  return 180*theta/PI;
}

int computeTraveling (int levy){ 
  float Ppartitioning = 0;
  float p = 0;
  float tFull;
  float tPart;
  // task1 = {Pick-source, Drop-Cache}
  // task2 = {Pick-cache, Drop-Nest}

  tFull = bot.estPickS + bot.estDropN;
  if (tFull == 0) { tFull = 1;}

  tPart = bot.estPickS + bot.estDropN;  
  if (tPart == 0) { tPart = 1;}
  
  switch(modelTest){
    case RANDOMLY:
      writeDecision(1.01, 0.01, TRAVELING_AGREE, flagTravel);
      return 1;
      break;
    case NEVER:
      writeDecision(0.01, 1.01, TRAVELING_AGREE, flagTravel);
      return 0;
      break; 
    case GREEDY: 
      Ppartitioning = ((float)rand())/RAND_MAX;
      if (bot.greedy > Ppartitioning) { //Random
        Ppartitioning = ((float)rand())/RAND_MAX;
        p = ((float)rand())/RAND_MAX;
      } else { //Pick the lowest value
        if (tFull < tPart){ //Non-partitioning
          p = 1;
          Ppartitioning = 0;
        } else { //partitioning
          p = 0;
          Ppartitioning = 1;
        }
      } break;
    case MODEL:
      if ((levy) && (modelTest == MODEL)) {
        switch(bot.currentState){
        case PICK_SOURCE:
          //printf("\n %s levy pick_source", robotName);
          tPart = bot.timeMeasured + bot.estDropN; 
          break;
        case DROP_NEST:
          //printf("\n %s levy drop_nest", robotName);
          tPart = bot.estPickS + bot.timeMeasured;
          break;       
        case TRAVEL2GREY:
        case TRAVEL2BLUE:
        case TRAVEL2RED:
          switch(statePrevious){
            case PICK_SOURCE:
              //printf("\n %s levy travel from pick-source", robotName);
              tPart = bot.timeMeasured + bot.estDropN;
              break;
            case DROP_NEST:
              //printf("\n %s levy travel from drop-nest", robotName);
              tPart = bot.estPickS + bot.timeMeasured; 
              break;
          } break;
        } 
      }
      if (tFull > tPart) {
        Ppartitioning = ((float) 1/(1 + powf(2.7182, ((float)(-bot.sParam*((tFull/tPart)-1))))));
      } else {
        Ppartitioning = ((float) 1/(1 + powf(2.7182, ((float)(-bot.sParam*(1-(tPart/tFull)))))));
      }
      p = ((float)rand())/RAND_MAX;
      break;
    }  
    //printf("\n Robot %s Ppartitioning is %.2f my chance is %.2f and timeFull is %.1f timePart is %.1f",robotName, Ppartitioning, p, tFull, tPart);
    if (levy) {
      writeDecision(Ppartitioning, p, TRAVELING_LEVY, flagTravel);
    } else {
      writeDecision(Ppartitioning, p, TRAVELING_AGREE, flagTravel);
    }
  int result = Ppartitioning > p;
  if (result){
    printf("\n %s goes by partitioning", robotName);
  } else {
    printf("\n %s do the full task", robotName);
  }
  speaking(M2ROBOT, -1, -1, -1);
  wb_robot_step(32); // to update global values
  return result;
}


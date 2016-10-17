/*
 * File:    Call of Duty.c
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

#include "movement.h"
#include "initBot.h"
#include "communication.h"
#include "readWriteFiles.h"
#include "headerStruct.h"
#include "dsp.h"
#include "complexMovements.h"
#include "registers.h"
//void cronometer(int task, int cache, int *suggestedState, int *timeImage, int *timeMeasured, struct robotDevices *botDevices, struct robotState *botState, struct flags4Files *botFlags);




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
int flagFiles = 1;
int flagMasterRecruiting = 0;   //1 RANDOMLY, -1 never, 0 whatever
// Communication flags
int flagCom = 1;                //to enable or disable communications
int flagListened = 0;           //to know if a data was listened or by herself
int listFriends[] = {2801,2802,2803,2804,2805,2806};
// Parameters
int alpha = 60;                 //in percentage
int flagMomento = 0;            //to enable soft changes
int beta = 10;                  //soft adaptations
int gammaUCB = 1000;            //in UCB-model 100/1000-Explote/Explore
float greedy = 0.1;             //in e-Greedy 0.01/0.11-Explote/Explore
float sParam = 1;               //in 2013 6/1-Explote/Explore
// Other flags
int flagReady = 0;              //to know when she ended a travel
int flagLoad = 0;               //to know if she has a load or not
int flagRobot = 0;              //to know if there is a robot in front
int flagPrint1 = 0;
int flagTravel = 0;             //to know if a robot is partitioning or not
int flagHold = 0;               //to keep counting the time  
int flagInside = 0;             //to know if a robot entered a TAM
// proximity sensors 
#define NB_DIST_SENS 8
#define THRESHOLD_DIST 150
#define LOW_S5 300
#define HIGH_S5 1400
//int ps_value[NB_DIST_SENS] = {0,0,0,0,0,0,0,0};
//int ps_offset[NB_DIST_SENS] = {35,35,35,35,35,35,35,35}; 
#define CALIBRATE 50
#define SAMPLES 1
//cam
WbDeviceTag displayExtra;
unsigned short width, height;
const unsigned char *image;
// Figures
#define BOX 301
#define TRIANGLE 302
#define CIRCLE 303
#define ALL 304
#define NOTHING 305
#define ROBOT 306
// Colors
#define RED 0
#define GREY 1
#define BLUE 2
#define CYAN 3
#define MAGENTA 4
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
int color = GREY;
int floorColor = BLUE;
int shapeSeen = NOTHING;
int figura = BOX;
int pointA = 0;
int pointB = 0;
// Threshold for images
#define PROXIMITY_COLOR 28 //the greater the distance, the smaller
#define COLOR_THRES 150
#define COMP_COLOR 80
#define ROBOT_THRES 100
#define BLACK_THRES 10
#define LOW_THRES 50
// leds
#define NB_LEDS 10 
// Movement and oddometry constants
double speed[2];
#define LEFT 0
#define RIGHT 1
#define PI 3.1416
#define MAX_SPEED 600
#define BACKWARD_SPEED 200
#define K_TURN 4
#define TURN_CACHE -52
#define TURN_90 27
#define TURN_M90 -27
// Communication 
WbDeviceTag receiver;
WbDeviceTag emitter;
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
int botNumber;
char fileRobot[] = "DIRPATH\\dd-hh-mm\\e-puck0000-OPTION.txt";
// Special variables
#define IMAGE 201
#define PICKING 0
#define DROPPING 1
#define NEST 5
#define SOURCE 10
// To write decisions
#define TRAVELING_AGREE 0
#define TRAVELING_LEVY 1
#define TRAVELING_CALL 2
//Timers
int timeImage = 0;
int timeMeasured = 0;
int timeListened = 0;
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
int stateUML;
int statePrevious;
int suggestedState=NONE;
int output = STOP;
int travelDestination = NONE;
// Main functions
void executeUML();
int moduleUML(int foreground, int shape, int pick_or_drop, int statePartition, int stateNoPartition, int flag);
int moduleTravel();
int moduleFSM();
// Initialization functions
void init_variables();
void reset();
void initEstimations();
// Miscellaneous functions
void pickingIndication(int on);
double angle(double x, double z);
// Time and performance functions
void countObjects();
// Model functions
int computeTraveling(int levy);

char dirPath[] = "/dir-dd-hh-mm";
time_t rawtime;
struct tm * timeinfo;

struct robotDevices botDevices;
struct robotEstimations botEst;
struct modelParam parameters;
struct flags4Files botFlags = {0};
struct robotCamera botCam;
struct robotState botState = {0};

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  time (&rawtime);
  timeinfo = localtime(&rawtime);
  sprintf(dirPath,"./%d-%d-%d",timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
  printf("\n path for directories %s", dirPath);
  printf("\n");

  reset(); 
  wb_robot_step(TIME_STEP);
  
  init_variables();     
  //printf("\n Robot %s is ready to begin", robotName);
  
  executeUML();
  wb_robot_cleanup();
  return 0;
}

void init_variables(){
  if (botNumber != 2701) {
    modelTest = ESSAY;
  }
  
  switch(modelTest){
    case ESSAY:
      stateUML = EXPERIMENT;//TRAVEL2GREY;
      botState.currentState = EXPERIMENT;
      color = CYAN;
      figura = BOX;
      break;
    case NEVER:
      flagMasterRecruiting = -1;
      stateUML = PICK_SOURCE;
      botState.currentState = PICK_SOURCE;
      output = STOP;
      color = RED;
      figura = BOX;
      if (floorColor == RED) {
         color = BLUE;
      }
    break;
    case RANDOMLY:
      flagMasterRecruiting = 1;
    case MODEL:
    case GREEDY:
      stateUML = PICK_SOURCE;
      botState.currentState = PICK_SOURCE;
      output = STOP;
      color = RED;
      figura = BOX;
      if (floorColor == RED) {
        color = BLUE;
      }
    break;
  }  
}

void executeUML(){
  while (wb_robot_step(TIME_STEP) != -1) {
    switch(botState.currentState){
      case EXPERIMENT:
        //printf("\n %d is on floorColor %d", botNumber, floorColor);
        cronometer(1000, 0, &suggestedState, &timeImage, &timeMeasured, &botDevices, &botState, &botFlags);
      break;
      case PICK_SOURCE:
        //printf("\n state PICK_SOURCE");
        //printf("\n");
        stateUML = moduleUML(RED, BOX, PICKING, DROP_NEST, travelDestination, 0);
        botState.currentState = stateUML;
         //printf("\n pass 2 state %d", stateUML); 
        //printf("\n");
        break;
      case DROP_NEST:
        //printf("\n state DROP_NEST");
        //printf("\n");
        stateUML = moduleUML(MAGENTA, BOX, DROPPING, PICK_SOURCE, travelDestination, 1);
        botState.currentState = stateUML;
		//printf("\n pass 2 state %d", stateUML);
        //printf("\n");
      break;
      case TRAVEL2GREY:
        //printf("\n state TRAVEL2NEST");
        //printf("\n");
        stateUML = moduleTravel();
		botState.currentState = stateUML;
        //printf("\n pass 2 state %d", stateUML);
        //printf("\n");
      break;      
      case TRAVEL2BLUE:
        //printf("\n state TRAVEL2SOURCE");
        //printf("\n");
        stateUML = moduleTravel();
		botState.currentState = stateUML;
        //printf("\n pass 2 state %d", stateUML);
        //printf("\n");
      break;
      case TRAVEL2RED:
        //printf("\n state TRAVEL2SOURCE");
        //printf("\n");
        stateUML = moduleTravel();
		botState.currentState = stateUML;
        //printf("\n pass 2 state %d", stateUML);
        //printf("\n");
      break;
      default:
        printf("\n Big failure on UML machine");
    }
  }  
}

int moduleUML(int foreground, int shape, int pick_or_drop, int stateRemain, int stateTravel, int flag) {

  color = foreground;
  if ((floorColor == RED) && (botState.currentState == PICK_SOURCE)) { color = BLUE;}
  figura = shape;
  statePrevious = botState.currentState;
  int auxShape = 0; //shapeSeen when working with different shapes
  
  if (pick_or_drop == PICKING) { pickingIndication(1);}
  else { wb_led_set(botDevices.leds[8], 1);}
  
  if (suggestedState != NONE) {  
    return suggestedState;
  }
  
  output = moduleFSM();
  //updateEstimations(IMAGE, timeImage, 0);
  int nextState = stateRemain;
  int flagWait = 1;
  flagLoad = flag;
  botState.flagLoad = flag; 
  if (output == STOP) {
    while(flagWait) {
      flagWait = waiting_color(color, color, &botState, &botCam, &botDevices);	
      if (flagWait == 0) {
        // Count only works with UCB
        countObjects();
        forward(-120, speed);     
        turnSteps(TURN_CACHE, speed);
        //waiting(100);  
        updateEstimations(botState.currentState, timeMeasured, auxShape, timeImage, timeMeasured, timeListened, &botState, &parameters, &botEst, &botDevices, &botFlags);
        
        flagLoad = !flag;
        botState.flagLoad = !flag;
        if (pick_or_drop == PICKING) { pickingIndication(0);}
        else { wb_led_set(botDevices.leds[8], 0);}	
        break;
      }  
    }
    flagTravel = computeTraveling(0);
    // Once a robot got an item, 
    // it has to dropped in that region
    if (flagTravel && (!flagLoad)) { 
      float p = ((float)rand())/RAND_MAX;
      if (p > 0.5) {
        travelDestination = floorColor - 1;
        if (travelDestination < 0) {
          travelDestination = 2; // BLUE
        }
      } else {
        travelDestination = floorColor + 1;
        if (travelDestination > 2) {
          travelDestination = 0; // RED
        }
      }
      //printf("\n %s would like to travel towards %d", robotName, travelDestination);
      //nextState = stateTravel;
    }	  
  } 
  return nextState;
}

int moduleTravel(){
  int auxUML = botState.currentState; 
  color = CYAN;
  figura = BOX;
  output = moduleFSM();
  //updateEstimations(IMAGE, timeImage, 0);
  if (output == STOP){  
    printf("\n %s is on region %d desiring to go to %d", robotName, floorColor, botState.currentState);
    printf("\n");
    if (botState.currentState  == TRAVEL2GREY) {
      if (floorColor == BLUE) {
        flagReady = going2region(color, RED, GREY, speed, &displayExtra, &botCam, &shapeSeen, &pointA, &pointB, &botDevices, &botState, &botFlags);
      } else if (floorColor ==  GREY) {
        flagReady = 1;
      }  else {
        flagReady = going2region(color, BLUE, GREY, speed, &displayExtra, &botCam, &shapeSeen, &pointA, &pointB, &botDevices, &botState, &botFlags);
      }  
    } else if (botState.currentState == TRAVEL2BLUE)  {
      if (floorColor == GREY){
        flagReady = going2region(color, RED, BLUE, speed, &displayExtra, &botCam, &shapeSeen, &pointA, &pointB, &botDevices, &botState, &botFlags);
      } else if (floorColor == BLUE) {
        flagReady = 1;
      } else {
        flagReady = going2region(color, BLUE, BLUE, speed, &displayExtra, &botCam, &shapeSeen, &pointA, &pointB, &botDevices, &botState, &botFlags);
      }
    } else if (botState.currentState == TRAVEL2RED) {
      if (floorColor == GREY){
        flagReady = going2region(color, BLUE, RED, speed, &displayExtra, &botCam, &shapeSeen, &pointA, &pointB, &botDevices, &botState, &botFlags);
      } else if (floorColor == RED) {
        flagReady = 1;
      } else {
        flagReady = going2region(color, RED, RED, speed, &displayExtra, &botCam, &shapeSeen, &pointA, &pointB, &botDevices, &botState, &botFlags);
      }
    } 
    if (flagReady) {  
      auxUML = PICK_SOURCE; //JUAN    
      /*if (floorColor == RED) {
        auxUML = TRAVEL2BLUE;
        printf("\n %s is going to BLUE",robotName);
        printf("\n");
      } else if (floorColor == BLUE)  {  
        auxUML = TRAVEL2RED;
        printf("\n %s is going to RED",robotName);
        printf("\n");
      } else {
        float p = ((float)rand())/RAND_MAX;
        if (p>0.5) {
          printf("\n By luck %s is going to BLUE",robotName);
          printf("\n");
          auxUML = TRAVEL2BLUE;
        } else {
          printf("\n By luck %s is going to RED", robotName);
          printf("\n");
          auxUML = TRAVEL2RED;
        }
      }*/
      updateEstimations(botState.currentState, timeMeasured, 0, timeImage,  timeMeasured, timeListened, &botState, &parameters, &botEst, &botDevices, &botFlags);
    }    
  } else {
    /*flagTravel = computeTraveling(1);
    if (flagTravel) { // Change of mind
      switch(floorColor) {
        case BLUE:
          auxUML = TRAVEL2GREY; break;
        case RED:
          auxUML = TRAVEL2GREY; break;
        case GREY:
          auxUML = TRAVEL2BLUE; break;
      }
    } *//*else {
      auxUML = PICK_SOURCE;
    } */ //JUAN   
  }
  wb_robot_step(32);
  return auxUML;
}

int moduleFSM(){
  int stateFSM = LEVY;
  int newIndex = 0;
  int index = -1;
  int nComp;
  int contLevy = 0;
  int contLost = 0;
  int contViewShape = 0;
  int flagSureSeen = 0;
  int oldShape = 0;
  int flagProximity = 0;
  int moduleEnded = 0;
  output = STOP_LEVY; //new
  
  while (!moduleEnded) {
    switch(stateFSM) {
      case LEVY:
        index = levyFlight(figura, color, speed, &botDevices, &botCam, &botState, &displayExtra, &shapeSeen, &pointA, &pointB);
        if (index == -1) {
          contLevy++;
          //printf("\n %s is thinking about her decision", robotName);
          //printf("\n");
          switch(botState.currentState){
           /* case TRAVEL2GREY:
            case TRAVEL2BLUE:
            case TRAVEL2RED:
              flagTravel  = computeTraveling(1);
              if (flagTravel) {
              //-- printf("\n %s decide to keep traveling", robotName);
              } else {
              //-- printf("\n %s cancel travel", robotName);
                updateBitacora(STOP_LEVY, FSM, 0);
                //timeMeasured = 0;
                return STOP_LEVY;
              }
            break;*/ //JUAN
            case PICK_SOURCE:
            case DROP_NEST:
              flagTravel  = computeTraveling(1);
              if (flagTravel) {
                printf("\n %s decide to change to travel", robotName);
                printf("\n");
                updateBitacora(STOP_LEVY, FSM, 0, timeMeasured, timeListened, &botEst, &botFlags, &botDevices, &botState);
                timeMeasured = 0;
                return STOP_LEVY;
              } else {
                printf("\n %s decide to go on in this region", robotName);                
                printf("\n");
              }
            break;                 
          }
        } else {  //it found something
          contLevy = 0;
          updateBitacora(LEVY, FSM, 0, timeMeasured, timeListened, &botEst, &botFlags, &botDevices, &botState);
          stateFSM = GO2IT; 
        }
        if (botState.currentState == PICK_SOURCE) {
          wb_led_set(botDevices.leds[0], 1);
        } else if (botState.currentState  == DROP_NEST){
          pickingIndication(1);
        } else {
          pickingIndication(0);
          wb_led_set(botDevices.leds[0], 0);
        } 
        wb_robot_step(32);
        break;  
      case GO2IT:
        //printf("\n %s found something and goes to get it", robotName);
        //printf("\n");
        flagProximity = going2it(index, color, speed, &displayExtra, &shapeSeen, &pointA, &pointB, &botCam, &botDevices, &botState);
        cronometer(IMAGE, 0, &suggestedState, &timeImage, &timeMeasured, &botDevices, &botState, &botFlags);              

        if (flagProximity){
          flagProximity = 0;
          flagSureSeen = 0;
          waiting(1);
          if ((botState.currentState == TRAVEL2BLUE) || (botState.currentState == TRAVEL2RED) || (botState.currentState == TRAVEL2GREY)) {
            flagInside = hitWall(0, speed, &botDevices);
          } else {
            flagInside = enterTam(&botDevices, speed);
          }
          if (flagInside == 1) { //sucessful enter TAM or hit wall
            stateFSM = STOP;
          } else {
            forward(-50, speed);
            stateFSM = LOST;
          }
          updateBitacora(GO2IT, FSM, 0, timeMeasured, timeListened, &botEst, &botFlags, &botDevices, &botState);
        } else { //it is yet far
          newIndex = detectImage(&displayExtra, &shapeSeen, &pointA, &pointB,   color, color, figura, 0, &nComp, &botCam, &botDevices, &botState);
          if (newIndex == -1) {
            flagRobot = check4Robot(&displayExtra, &shapeSeen, &pointA, &pointB,   color, ROBOT_COLOR, ROBOT, 0, &nComp, &botCam, &botDevices, &botState);
            if (flagRobot) {
              printf("\n %s saw a robot in path to target", robotName);
              printf("\n");
              waiting(5);
              cronometer(-1, 0, &suggestedState, &timeImage, &timeMeasured, &botDevices, &botState, &botFlags);              
            } else {
              updateBitacora(GO2IT, FSM, 0, timeMeasured, timeListened, &botEst, &botFlags, &botDevices, &botState);
              stateFSM = LOST;
            }
            flagSureSeen = 0;
            contViewShape = 0;
            oldShape = -1;
          } else {
            if (flagSureSeen) { index = 100;}
            else if (newIndex >= 0) { index = newIndex;}
            if (shapeSeen == oldShape) { contViewShape++;}
            if ((contViewShape > 10) && (nComp == 1)) {
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
      printf("\n %s is lost", robotName);
      run(flagLoad, 5, speed, &botDevices);
      whereIam(1, color, speed, &botCam, &botDevices, &botState);
      index = detectImage(&displayExtra, &shapeSeen, &pointA, &pointB,   color, color, figura, 0, &nComp, &botCam, &botDevices, &botState);
      if (index >= 0) {
        contLost = 0;
        updateBitacora(LOST, FSM, 0, timeMeasured, timeListened, &botEst, &botFlags, &botDevices, &botState);
        stateFSM = GO2IT;
      } else {
        contLost++;
        if (contLost > 3) {
          contLost = 0;
          updateBitacora(LOST, FSM, 0, timeMeasured, timeListened, &botEst, &botFlags, &botDevices, &botState);
          stateFSM = LEVY;
        }
      }
      break;
    case STOP:
      moduleEnded = 1;
      //printf("\n %s succesfully ended the module FSM with shapeseen %d", robotName, shapeSeen);
      //printf("\n");
      updateBitacora(STOP, FSM, 0, timeMeasured, timeListened, &botEst, &botFlags, &botDevices, &botState);
      return STOP;
    break;
    default:
      printf("\n %s is with mistakes in FSM module", robotName);
      return 0;      
    }
  }  
  return STOP;
}

void reset(){ //ok-
  resetDevices(&botDevices);
  //width = botDevices.width;
  //height = botDevices.height;
  //working on these
  botCam.width = botDevices.width;
  botCam.height = botDevices.height;
  botCam.image = wb_camera_get_image(botDevices.cam);
  // Display for user
  resetDisplay(&displayExtra, botCam.width, botCam.height);
  // Getting data for initial state
  floorColor = whereIam(0, color, speed, &botCam, &botDevices, &botState);
  botState.floorColor = whereIam(0, color, speed, &botCam, &botDevices, &botState);
  //printf("\n %s was born in region %d", wb_robot_get_name(), floorColor);
  //printf("\n");
  // Create files, enable/disable files records
  botFlags.fileRobot = fileRobot;
  botFlags.dirPath = dirPath; 
  if (flagFiles) { 
    botFlags.flagFilesFSM = 0;
    botFlags.flagFilesEST = 1;
    botFlags.flagFilesLIFE = 1;
    botFlags.flagFilesDM = 0;
    botFlags.flagFilesPER = 0;
    botFlags.flagFilesCOM = 0;
    createFiles(&botFlags);
  } 
  // model parameters structure
  parameters.flagMomento = flagMomento;
  parameters.alpha = alpha;
  parameters.beta = beta;
  parameters.gammaUCB = gammaUCB; //in UCB-model 100/1000-Explote/Explore
  parameters.greedy = greedy;    //in e-Greedy 0.01/0.11-Explote/Explore
  parameters.sParam = sParam;      //in 2013 6/1-Explote/Explore in 2011 is 2.5

  initEstimations(&botEst, NB_REGIONS);
  updateBitacora(0, ESTIMATIONS, 0, timeMeasured, timeListened, &botEst, &botFlags, &botDevices, &botState);
  // Random seed by the number of the robot
  strcpy(robotName, wb_robot_get_name());
  botNumber = atoi(&robotName[6]);
  botState.botNumber = botNumber;
  srand(botNumber+timeinfo->tm_mday+timeinfo->tm_hour+timeinfo->tm_min);
  wb_robot_step(TIME_STEP);
  // Adjust sensor noise by offset
  calibrateSensors(&botDevices);
  /* 
  printf("\n Calibration offset ");
  int i;
  for (i=0; i<NB_DIST_SENS; i++){
    printf("%d ", botDevices.ps_offset[i]);
  } 
  */ 
  botDevices.flagCom = flagCom;
  botDevices.flagListened = flagListened;
  speaking(&botDevices, botNumber, M2NEST, ROBOT_ARRIVING, 0, 0, &botFlags);
}

void pickingIndication(int on){ //ok
  wb_led_set(botDevices.leds[1], on);
  wb_led_set(botDevices.leds[7], on);
}

double angle(double x, double z){ //ok
  double theta = atan(z/x);
  return 180*theta/PI;
}

void countObjects(){
  switch(botState.currentState){
    case DROP_NEST:
      botEst.nDrop[floorColor]++; break;
    case PICK_SOURCE:
      botEst.nPick[floorColor]++; break;
  }
  printf("\n We have %d objects picked", botEst.nPick[floorColor]);
  printf("\n");
  if (botFlags.flagFilesPER) {
    createDir(PERFORMANCE, 0, &botFlags);
    //printf("\n %s is counting objects in %s", robotName, fileRobot);
    //printf("\n");
    FILE *fper = fopen(fileRobot, "a+");
    int i;
    for (i=0; i<NB_REGIONS; i++){ 
      fprintf(fper, "%d, %d, ", botEst.nPick[i], botEst.nDrop[i]);
    }
    fprintf(fper, "\n");  
    fclose(fper); //-- JUAN EDIT FILES
  }  
}



int computeTraveling (int levy){ 
  float Ppartitioning = 0;
  float p = 0;
  float tFull;
  float tPart;
  // task1 = {Pick-source, Drop-Cache}
  // task2 = {Pick-cache, Drop-Nest}

  tFull = botEst.estPickS + botEst.estDropN;
  if (tFull == 0) { tFull = 1;}

  tPart = botEst.estPickS + botEst.estDropN;  
  if (tPart == 0) { tPart = 1;}
  
  switch(modelTest){
    case RANDOMLY:
      writeDecision(1.01, 0.01, TRAVELING_AGREE, flagTravel, &botFlags);
      return 1;
    break;
    case NEVER:
      writeDecision(0.01, 1.01, TRAVELING_AGREE, flagTravel, &botFlags);
      return 0;
    break; 
    case GREEDY: 
      Ppartitioning = ((float)rand())/RAND_MAX;
      if (greedy > Ppartitioning) { //Random
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
        switch(botState.currentState){
          case PICK_SOURCE:
            //printf("\n %s levy pick_source", robotName);
            tPart = timeMeasured + botEst.estDropN; 
          break;
          case DROP_NEST:
            //printf("\n %s levy drop_nest", robotName);
            tPart = botEst.estPickS + timeMeasured;
          break;                 
          case TRAVEL2GREY:
          case TRAVEL2BLUE:
          case TRAVEL2RED:
            switch(statePrevious){
              case PICK_SOURCE:
                //printf("\n %s levy travel from pick-source", robotName);
                tPart = timeMeasured + botEst.estDropN;
              break;
             case DROP_NEST:
                //printf("\n %s levy travel from drop-nest", robotName);
                tPart = botEst.estPickS + timeMeasured; 
              break;
          } break;
      }   
    }
    if (tFull > tPart) {
      Ppartitioning = ((float) 1/(1 + powf(2.7182, ((float)(-sParam*((tFull/tPart)-1))))));
    } else {
      Ppartitioning = ((float) 1/(1 + powf(2.7182, ((float)(-sParam*(1-(tPart/tFull)))))));
    }
    p = ((float)rand())/RAND_MAX;
    break;
  }  
  
  //printf("\n Robot %s Ppartitioning is %.2f my chance is %.2f and timeFull is %.1f timePart is %.1f",robotName, Ppartitioning, p, tFull, tPart);
  if (levy) {
    writeDecision(Ppartitioning, p, TRAVELING_LEVY, flagTravel, &botFlags);
  } else {
    writeDecision(Ppartitioning, p, TRAVELING_AGREE, flagTravel, &botFlags);
  }
  
  int result = Ppartitioning > p;
  if (result){
    printf("\n %s goes by partitioning", robotName);
  } else {
    printf("\n %s do the full task", robotName);
  }
  speaking(&botDevices, botNumber, M2ROBOT, -1, -1, -1, &botFlags);
  wb_robot_step(32); // to update global values
  return result;
}
/*
//void cronometer(int task, int cache){//ok-
void cronometer(int task, int cache, int *suggestedState, int *timeImage, int *timeMeasured, struct robotDevices *botDevices, struct robotState *botState, struct flags4Files *botFlags){//ok-
  
  if (task == IMAGE) { 
    (*timeImage)++;
    printf("\n Time images %d", *timeImage);
    printf("\n");
  } else {  
    (*timeMeasured)++;
  }
  //printf("\n %s is listening", robotName);
  //printf("\n");
//juan  listening(botDevices.receiver, floorColor, botNumber, listFriends, &stateUML, &suggestedState, &botFlagFiles); //--JUAN EDIT FILES
  listening(botDevices->receiver, floorColor, botNumber, listFriends, &botState->currentState, suggestedState, botFlags); //--JUAN EDIT FILES

  if (botFlags->flagFilesLIFE) {
    createDir(LIFE, 0, botFlags);
    //printf("\n %s is updating in %s", robotName, fileRobot);
    //printf("\n");
    FILE *flife = fopen(fileRobot,"a+");
    if (task == IMAGE) { 
      fprintf(flife, "image, %d \n", *timeImage);
    } else {  
     fprintf(flife, "state %d, %d\n", botState->currentState, *timeMeasured);
    }
    fclose(flife); //-- JUAN EDIT FILES 
  } 
}
*/
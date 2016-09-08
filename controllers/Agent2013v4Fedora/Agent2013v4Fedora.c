/*
 * File:    Agent2013v4.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#define TIME_STEP 64
//Model -1 is for experiments, 0 is 2011, 1 is mine, 2 is UCB, 3 is Greedy 
#define ESSAY 0
#define ALWAYS 1
#define NEVER 2
#define BASE2011 3
#define BASE2013 4 
#define MINE 5
#define UCB 6
#define GREEDY 7         
// for different models
int modelTest = NEVER;
// Communication flags
int flagCom = 1;                //to enable or disable communications
int flagListened = 0;           //to know if a data was listened or by herself
int listFriends[] = {2801,2802,2803,2804,2805,2806};
// Parameters
int alpha = 60;                 //in percentage
int flagMomento = 0;            //to enable soft changes
int beta = 10;                  //soft adaptations
int gammaUCB = 1000;               //in UCB-model 100/1000-Explote/Explore
float greedy = 0.1;            //in e-Greedy 0.01/0.11-Explote/Explore
float sParam = 1;                  //in 2013 6/1-Explote/Explore
// Flags of control
int flagMasterPartitioning = 0; //1 always, -1 never, 0 whatever
int flagMasterGiveup = 0;      // 1 always, 0 whatever  
// Other flags
int flagReady = 0;              //to know when she ended a travel
int flagLoad = 0;               //to know if she has a load or not
int flagRobot = 0;              //to know if there is a robot in front
int flagPrint1 = 1;
int flagPartition = 0;          //to know if a robot is partitioning or not
int flagGiveup = 0;             //to know if a robot give up
int flagHold = 0;               //to keep counting the time  
int flagInside = 0;             //to know if a robot entered a TAM
// proximity sensors 
#define NB_DIST_SENS 8
#define THRESHOLD_DIST 150
#define LOW_S5 300
#define HIGH_S5 1400
WbDeviceTag Robotps[NB_DIST_SENS];	
int ps_value[NB_DIST_SENS] = {0,0,0,0,0,0,0,0};
int ps_offset[NB_DIST_SENS] = {35,35,35,35,35,35,35,35}; 
#define CALIBRATE 50
#define SAMPLES 1
//cam
WbDeviceTag cam;
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
#define BLUE 1
#define RED 2
#define CYAN 3
#define MAGENTA 4
#define BLACK 6
#define GREEN 7
#define WHITE 8
#define TAM_WALL 9
#define GREY 10
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
WbDeviceTag RobotLed[NB_LEDS];
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
// Robot files
#define FSM 0
#define ESTIMATIONS 1
#define LIFE 2
#define DECISIONS 3
#define PERFORMANCE 4 
char robotName[11];
char fileRobot[] = "dd-hh-mm\\e-puck0000-OPTION.txt";
// Special variables
#define IMAGE 201
#define PICKING 0
#define DROPPING 1
#define NEST 5
#define SOURCE 10
// To write decisions
#define PARTITION_TASK 0
#define PARTITION_HEAR 1
#define PARTITION_LEVY 2
#define GIVEUP_LEVY -2
#define GIVEUP_CACHE -3
#define GIVEUP_HEAR -4
//Timers
#define NB_CACHE 4
int WaitingCacheDrop[NB_CACHE];
int WaitingCachePick[NB_CACHE];
int estPickC = 0;
int estPickS = 0;
int estDropC = 0;
int estDropN = 0;
int estHarvest = 0;
int estStore = 0;
int lastImage = 0, timeImage = 0;
int timeMeasured = 0;
int timeListened = 0;
// Only for UCB algorithm
int nPick = 1;
int nDrop = 1;
int nHarvest = 1;
int nStore = 1;
// Main core FSM
#define GO2IT 1
#define LEVY 2
#define LOST 3
#define STOP 4
#define STOP_LEVY 5
// UML states
#define PICK_SOURCE 100
#define PICK_CACHE 101
#define DROP_CACHE 102
#define DROP_NEST 103
#define TRAVEL2NEST 104
#define TRAVEL2SOURCE 105
#define STORE 106
#define HARVEST 107
#define EXPERIMENT 666
#define WAITING 777
int stateUML;
int statePrevious;
int output = STOP;
// Main functions
void executeUML();
int moduleUML(int foreground, int shape, int pick_or_drop, int statePartition, int stateNoPartition, int flag);
int moduleTravel(int nextState, int holdState, int stateLoad, int stateUnload);
int moduleFSM();
// Initialization functions
void reset();
void resetDisplay();
void createDir(int option, int dir);
void createFiles();
void initEstimations();
// Miscellaneous functions
int readSensors(int print);
void pickingIndication(int on);
int waiting(int n);
double angle(double x, double z);
// Image-depending functions
int whereIam(int avoiding);
int find_middle(int entrace);
int check4Robot();
int waiting_color(int foreground);
int cont_height_figure(int indexP);
int compareColorPixel(int pixelX, int pixelY, int foreground);
int detectImage(int foreground, int shape, int numImage, int *numberComponents);
int whatIsee(float Eccentricity, float Extent, int squareWidth, int middleAxisH, int middleAxisV, int numImage);
int doubleCheck();
// Movement functions
void avoidance();
int followingLine(int colorLine, int entrace);
void turnSteps(int steps);
int run(int steps);
void forward(int steps);
int levyFlight();
int speedAdjustment(int index, int delta);
int hitWall(int front);
int hitLandmark(); 
int checkingArrival(int colorLine);
int enterTam(); 
int detectTam();
int adjustWallTurn(int steps);
// Reaching targets
int going2Nest(); 
int going2Source(); 
int going2it(int index); 
// Time and performance functions
void cronometer(int task, int cache);
void countObjects();
void updateEstimations(int task, int value, int cache);
void updateBitacora(int codeTask, int estimations, int cache);
void writeDecision(float boundP, float realP, int mechanism);
// Communication functions
int speaking(int codeTask, int time, int cache);//checked
int listening();                                //checked
// Model functions
int computePartition(int levy);
int computeGiveUp(int levy, int cache);

char dirPath[] = "dir-dd-hh-mm";
time_t rawtime;
struct tm * timeinfo;

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  time (&rawtime);
  timeinfo = localtime(&rawtime);

  sprintf(dirPath,"./%d-%d-%d",timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
  reset(); 
  wb_robot_step(TIME_STEP);
  
  //int flagMasterPartitioning = 0; //1 always, -1 never, 0 whatever
  //int flagMasterGiveup = 0;      // 1 always, 0 whatever  
  switch(modelTest){
    case ESSAY:
      stateUML = TRAVEL2NEST;
      color = CYAN;
      figura = ALL;
    break;
    case NEVER:
      flagMasterPartitioning = -1;
      if (floorColor == BLUE) {
        stateUML = HARVEST;
        output = STOP;
        color = RED;
        figura = BOX;
      } else {
        stateUML = TRAVEL2SOURCE;
        output = STOP;
        color = CYAN;
        figura = ALL;
      }
    break;
    case ALWAYS:
      flagMasterPartitioning = 1;
    case BASE2011:
    case BASE2013:
    case MINE:
    case UCB:
    case GREEDY:
      if (floorColor == BLUE) {
        stateUML = PICK_SOURCE;
        color = RED;
        figura = BOX;
      } else {
        stateUML = PICK_CACHE;
        color = MAGENTA;
        figura = ALL;
      }  
    break;
  }  
  
  printf("\n Robot %s is ready to begin", robotName);
  executeUML();
  wb_robot_cleanup();
  return 0;
}

void executeUML(){
  int nComp = -1;
  while (wb_robot_step(TIME_STEP) != -1) {
    switch(stateUML){
      case EXPERIMENT:
        //printf("\n %d", detectImage(color, BOX, 0, &nComp));
        //nComp = levyFlight();
        printf("\n Levy result %d", nComp);
        hitLandmark();
        going2Source();
      break;
      case STORE:
        //printf("\n state STORE");
        stateUML = moduleUML(BLUE, BOX, DROPPING, PICK_CACHE, TRAVEL2SOURCE, 1);
        //printf("\n state %d", stateUML);
      break;
      case DROP_NEST:
        //printf("\n state DROP_NEST");
        stateUML = moduleUML(BLUE, BOX, DROPPING, PICK_CACHE, TRAVEL2SOURCE, 1);
        //printf("\n state %d", stateUML);
      break;
      case HARVEST:
        //printf("\n state HARVEST");
        stateUML = moduleUML(RED, BOX, PICKING, DROP_CACHE, TRAVEL2NEST, 0);
        //printf("\n state %d", stateUML);
      break;
      case PICK_SOURCE:
        //printf("\n state PICK_SOURCE");
        stateUML = moduleUML(RED, BOX, PICKING, DROP_CACHE, TRAVEL2NEST, 0);
        //printf("\n state %d", stateUML); 
      break;
      case PICK_CACHE:
        //printf("\n state PICK_CACHE");
        stateUML = moduleUML(MAGENTA, ALL, PICKING, DROP_NEST, TRAVEL2SOURCE, 0);   
        //printf("\n state %d", stateUML);
      break;
      case DROP_CACHE:
        //printf("\n state DROP_CACHE");
        stateUML = moduleUML(MAGENTA, ALL, DROPPING, PICK_SOURCE, TRAVEL2NEST, 1);
        //printf("\n state %d", stateUML);
      break;
      case TRAVEL2NEST:
        //printf("\n state TRAVEL2NEST");
        stateUML = moduleTravel(STORE, PICK_CACHE, DROP_CACHE, PICK_SOURCE);
        //printf("\n state %d", stateUML);
      break;      
      case TRAVEL2SOURCE:
        //printf("\n state TRAVEL2SOURCE");
        stateUML = moduleTravel(HARVEST, DROP_CACHE, DROP_NEST, PICK_CACHE);
        //printf("\n state %d", stateUML);
      break;
      default:
        printf("\n Big failure on UML machine");
    }
  }  
}

int moduleUML(int foreground, int shape, int pick_or_drop, int statePartition, int stateNoPartition, int flag) {
//  output = 0; 
  color = foreground;
  figura = shape;
  statePrevious = stateUML;
  int auxShape = 0; //shapeSeen when working with different shapes
  
  if (pick_or_drop == PICKING) { pickingIndication(1);}
  else { wb_led_set(RobotLed[8], 1);}
  
  output = moduleFSM();
  //updateEstimations(IMAGE, timeImage, 0);
  int nextState = stateNoPartition;
  int flagWait = 1;
  flagLoad = flag; 
  if (output == STOP) {
    while(flagWait) {
      flagWait = waiting_color(color);	
      if (flagWait == 0) {
        if ((stateUML == PICK_CACHE) || (stateUML == DROP_CACHE)) {
          //updateEstimations(stateUML, timeMeasured, auxShape);
          nextState = statePartition;
        } 
        // Count only works with UCB
        //if (modelTest == UCB) { countObjects();}
        countObjects();
        forward(-120);     
        turnSteps(TURN_CACHE);   
        flagWait = 1;	 
        updateEstimations(stateUML, timeMeasured, auxShape);
        flagLoad = !flag;
        if (pick_or_drop == PICKING) { pickingIndication(0);}
        else { wb_led_set(RobotLed[8], 0);}	
        break;
      } else {
        if ((stateUML == PICK_CACHE) || (stateUML == DROP_CACHE)) {  
	   flagGiveup = 0; //computeGiveUp(shapeSeen); maybe each 100 steps
          if (flagGiveup) {
            //updateEstimations(stateUML, timeMeasured, 0);
	     break;
          }		  
        } 
      }  
    }
    if ((stateUML != PICK_CACHE) && (stateUML != DROP_CACHE)) {
      flagPartition = computePartition(0);
      if (flagPartition) {
        nextState = statePartition;
      }
    }  	  
  } 
  return nextState;
}

int moduleTravel(int nextState, int holdState, int stateLoad, int stateUnload){
  int auxUML = stateUML; 
  color = CYAN;
  figura = ALL;
  switch(statePrevious){
    case PICK_SOURCE:
    case DROP_CACHE:
    case HARVEST:
    case PICK_CACHE:
    case DROP_NEST:
    case STORE:
      flagHold = 1;
      if (output == STOP) { flagHold = 0;} break;
    default:
      printf("\n Problem when traveling");
      flagHold = 0;
  } 
  output = moduleFSM();
  //updateEstimations(IMAGE, timeImage, 0);
  if (output == STOP){  
    if (stateUML  == TRAVEL2SOURCE) {
      flagReady = going2Source();
    } else {
      flagReady = going2Nest();
    }  
    if (flagReady) {  
      auxUML = nextState;
      if (flagHold) {
        auxUML = holdState;
        updateEstimations(stateUML, timeMeasured, 0);
        if (statePrevious == DROP_CACHE) { auxUML = DROP_NEST;}
        if (statePrevious == PICK_CACHE) { auxUML = PICK_SOURCE;}
      }
    }    
  } else {
    //updateEstimations(stateUML, timeMeasured, 0); 
    if (flagLoad) {
      auxUML = stateLoad;
    } else {
      auxUML = stateUnload;
    }    
  }
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
        wb_led_set(RobotLed[0], 1);
        wb_robot_step(32);
        index = levyFlight();
        if (index == -1) {
          contLevy++;
          printf("\n %s is thinking about her decision", robotName);
          switch(stateUML){
            case TRAVEL2NEST:
            case TRAVEL2SOURCE:
              flagPartition = computePartition(1);
              if (flagPartition){
                printf("\n %s cancel travel", robotName);
                updateBitacora(STOP_LEVY, FSM, 0);
                //timeMeasured = 0;
                return STOP_LEVY;
              } else {
                printf("\n %s decide to keep traveling", robotName);
              }
            break;
            case PICK_SOURCE:
            case DROP_NEST:
              flagPartition = computePartition(1);
              if (flagPartition) {
                printf("\n %s decide to go on with pick-drop", robotName);
              } else {
                printf("\n %s decide to change to full task", robotName);
                updateBitacora(STOP_LEVY, FSM, 0);
                timeMeasured = 0;
                return STOP_LEVY;
              }
            break;                 
            case HARVEST:
            case STORE:
              flagPartition = computePartition(1);
              if (flagPartition) {
                printf("\n %s decide to change to partition", robotName);
                updateBitacora(STOP_LEVY, FSM, 0);
                timeMeasured = 0;
                return STOP_LEVY;
              } else {
                printf("\n %s decide to go on harvest-store", robotName);
              }
            break;
            case DROP_CACHE:
            case PICK_CACHE:
              flagGiveup = computeGiveUp(1, 0);
              if (flagGiveup) {
                printf("\n %s give up of cache %d", robotName, shapeSeen);
                updateBitacora(STOP_LEVY, FSM, 0);
                timeMeasured = 0;
                return STOP_LEVY;
              } else {
                printf("\n %s decide to go on cache", robotName);
              }
            break;
        }
      } else {  //she found something
        contLevy = 0;
        updateBitacora(LEVY, FSM, 0);
        stateFSM = GO2IT; 
      }
      wb_led_set(RobotLed[0], 0);
      wb_robot_step(32);      
    break;  
    case GO2IT:
      //printf("\n %s found something and goes to get it", robotName);
      flagProximity = going2it(index);
      //printf("\n %s proximity is %d", robotName, flagProximity);
      if (flagProximity){
        flagProximity = 0;
        flagSureSeen = 0;
        waiting(1);
        if ((stateUML == TRAVEL2NEST)  || (stateUML == TRAVEL2SOURCE)) {
          flagInside = hitLandmark();
        } else {
          flagInside = enterTam();
        }
        if (flagInside) { //sucessful enter TAM or hit wall
          stateFSM = STOP;
        } else {
          forward(-50);
          stateFSM = LOST;
        }
        updateBitacora(GO2IT, FSM, 0);
      } else { //she is yet far
        newIndex = detectImage(color, figura, 0, &nComp);
        if (newIndex == -1) {
          flagRobot = check4Robot();
          if (flagRobot) {
            printf("\n %s saw a robot in path to target", robotName);
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
          if (shapeSeen == oldShape) { contViewShape++;}
          if ((contViewShape > 10) && (nComp == 1)) {
            contViewShape = 0;
            flagSureSeen = 1;
            printf("\n %s is sure that saw shape %d", robotName, oldShape);
            //speaking(shapeSeen, shapeSeen, shapeSeen); //to make a rendesvouz
          }           
        }
      }  
    break;
    case LOST:
      //printf("\n %s is lost", robotName);
      run(5);
      whereIam(1);
      index = detectImage(color, figura, 0, &nComp);
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
      printf("\n %s succesfully ended the module FSM with shapeseen %d", robotName, shapeSeen);
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

void reset(){ //ok-
  int i, k;
  // get distance sensors
  char textRobotps[4] = "ps0";
  for (i=0;i<NB_DIST_SENS; i++) {
    Robotps[i] = wb_robot_get_device(textRobotps);
    textRobotps[2]++;
    wb_distance_sensor_enable(Robotps[i], TIME_STEP);
  }
  // get camera
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP);
  width = wb_camera_get_width(cam);
  height = wb_camera_get_height(cam);
  // Display for user
  displayExtra = wb_robot_get_device("displayExtra");
  wb_display_set_color(displayExtra, HEXWHITE);
  resetDisplay();
  // enabling encoders
  wb_differential_wheels_enable_encoders(TIME_STEP);
  wb_differential_wheels_set_encoders(0,0);
  // get leds
  char text[5] = "led0"; 
  for (i=0; i<NB_LEDS; i++) {
    RobotLed[i] = wb_robot_get_device(text); 
    text[3]++; 
  }
  // communication module
  receiver = wb_robot_get_device("receiver");
  emitter = wb_robot_get_device("emitter");
  wb_receiver_enable(receiver,TIME_STEP);
  // Getting data
  floorColor = whereIam(0);
  createFiles();
  initEstimations();
  strcpy(robotName, wb_robot_get_name());
  // Random seed by the number of the robot
  int botNumber = atoi(&robotName[6]);
  k = botNumber+timeinfo->tm_mday+timeinfo->tm_hour+timeinfo->tm_min;
  srand(k);
  wb_robot_step(TIME_STEP);
  // during the following n-1 simulation steps, increment the arrays
  for (k = 0; k <CALIBRATE; k++){
      for (i=0; i<NB_DIST_SENS; i++){
        ps_offset[i] += (int) wb_distance_sensor_get_value(Robotps[i]);
      }
      wb_robot_step(TIME_STEP);
  } 
  printf("\n Calibration offset ");
  for (i=0; i<NB_DIST_SENS; i++){
    ps_offset[i] /= CALIBRATE-1;
    printf("%d ", ps_offset[i]);
  }
}

void resetDisplay(){ //ok
  wb_display_set_color(displayExtra, HEXBLACK);
  wb_display_draw_rectangle(displayExtra, 0, 0, width, height);
}

void createDir(int option, int dir){
  switch(option){
    case FSM:
      sprintf(fileRobot, "./%s-FSM", dirPath); break;
    case ESTIMATIONS:
      sprintf(fileRobot, "./%s-EST", dirPath); break;
    case LIFE:
      sprintf(fileRobot, "./%s-REC", dirPath); break;
    case DECISIONS:
      sprintf(fileRobot, "./%s-DM", dirPath); break;
    case PERFORMANCE:
      sprintf(fileRobot, "./%s-OBJ", dirPath); break;
   }
   
   if (dir){ mkdir(fileRobot,0700);} 
    
   strcat(fileRobot, "/");
   strcat(fileRobot, wb_robot_get_name()); 
   switch(option){
     case FSM:
       strcat(fileRobot, "-FSM.txt"); break;
     case ESTIMATIONS:
       strcat(fileRobot, "-EST.txt"); break;
     case LIFE:
       strcat(fileRobot, "-REC.txt"); break;
     case DECISIONS:
       strcat(fileRobot, "-DM.txt"); break;
     case PERFORMANCE:
       strcat(fileRobot, "-OBJ.txt"); break;
  }
}

void createFiles(){ //ok
  // File for each cycle of FSM
  createDir(FSM, 1); 
  FILE *ffsm = fopen(fileRobot,"w");
  if (ffsm == NULL) {
    printf("Error opening file bot fsm \n");
    exit(1);
  }
  fprintf(ffsm, "StateUML, Suceess/Fail, timeMeasured \n");
  fclose(ffsm);
  // File for evolution of estimations
  createDir(ESTIMATIONS, 1); 
  FILE *fest = fopen(fileRobot, "w");
  if (fest == NULL) {
    printf("Error opening estimation file \n");
    exit(1);
  }
  fprintf(fest,"who, tStore, tHarvest, tPickS, tDropC,"
               " tePickC, tDropN, WCacheDrop, WCachePick,"
                "tImage \n");
  fclose(fest);
  // File for record entire life
  createDir(LIFE, 1); 
  FILE *flife = fopen(fileRobot, "w");
  if (flife == NULL) {
    printf("Error opening estimation file \n");
    exit(1);
  }
  fprintf(flife,"what, time \n");
  fclose(flife);
  // File for decisions
  createDir(DECISIONS, 1); 
  FILE *fpart = fopen(fileRobot, "w");
  if (fpart == NULL) {
    printf("Error opening partition file \n");
    exit(1);
  }
  fprintf(fpart, "Case, decision \n");
  fclose(fpart);
  // File for performance 
  createDir(PERFORMANCE, 1);
  FILE *fper = fopen(fileRobot, "w");
  if (fper == NULL) {
    printf("Error opening performance file \n");
    exit(1);
  }
  fprintf(fper, "PICK, DROP, HARVEST, STORE\n");
  fclose(fper);
}

void initEstimations(){ //ok-
  memset(WaitingCacheDrop, 0, NB_CACHE);
  memset(WaitingCachePick, 0, NB_CACHE);
  // rand() % (max_n - min_n + 1) + min_n;
  estPickC = rand() % (500-200+1)+200; 
  estDropC = rand() % (800-200+1)+230;
  estPickS = rand() % (500-200+1)+200;
  estHarvest = rand() % (2500-1200+1)+1000;
  estStore = rand() % (2500-1200+1)+1000;
  estDropN = rand() % (500-200+1)+200;
  timeMeasured = 0;
  timeListened = 0;
  lastImage = 0;
  timeImage = 0;    
  wb_robot_step(32);
  updateBitacora(0, ESTIMATIONS, 0);
}

int readSensors(int print){ //ok-
  int flag = 0, i, k;
  // Reset values
  for(i=0; i<NB_DIST_SENS; i++){ ps_value[i] = 0;}
  //Sensor values
  for (k=0; k<SAMPLES; k++) { 
    for (i=0; i<NB_DIST_SENS; i++) {
      ps_value[i] += (int)wb_distance_sensor_get_value(Robotps[i])-ps_offset[i];
    }
    wb_robot_step(TIME_STEP); 
  }  
  for (i=0; i<NB_DIST_SENS; i++){
    ps_value[i] /= SAMPLES;
    if (ps_value[i] > THRESHOLD_DIST) { 
      flag = 1;
      if (print) { //Sensor 5 for follow wall
        printf("\n An obstacle is detected at sensor %d value %d", i, ps_value[i]);
      }
    } 
  }
  return flag;
}

void pickingIndication(int on){ //ok
  wb_led_set(RobotLed[1], on);
  wb_led_set(RobotLed[7], on);
}

int waiting(int n){ //ok
  wb_differential_wheels_set_speed(0,0);
  while (n > 0) {
    n--;
    wb_robot_step(TIME_STEP);
    cronometer(-1, 0);
  } 
  return 1;  
}

double angle(double x, double z){ //ok
  double theta = atan(z/x);
  return 180*theta/PI;
}

int whereIam(int avoiding){ 
  image = wb_camera_get_image(cam);
  wb_robot_step(TIME_STEP);
  // cronometer(IMAGE, 0); //This is a fast operation
  
  if (cont_height_figure(-20) > 104 ) { 
    return BLUE;
  } else if (cont_height_figure(-21) > 104 ) {
    return RED;
  } 
  if (avoiding){
    turnSteps(TURN_CACHE);
    run(7);//15
    printf("\n Missing my region %s", robotName);
  }    
  return GREY;
} 

int find_middle(int entrace){ //ok 
  int i, aux, index1 = -1, index2 = -1, foreground = BLUE;
  if (stateUML == TRAVEL2SOURCE) {
    foreground = RED;
  }
  // new world
  if (entrace == 1) {
    if (foreground == RED) {
      foreground = BLUE;
    } else {
      foreground = RED;
    }
  }
  for (i = 0; i<width; i++){
    aux = compareColorPixel(i, height-1, foreground);
    if (aux == 1) {
      if (index1 == -1) { // the 1st time see the color
        index1 = i;
      } else { // the final index where the color is seen
        index2 = i;
      }  
    }
  }  
  if (index1 == -1) { return -1;} // followLine
  return (index2-index1)/2+index1;
}

int check4Robot(){//ok-
  int nComp, sizeRobot = 0;
  sizeRobot = detectImage(GREY, ROBOT, 0, &nComp);
  //printf("\n I %s looking a robot in the image of height %d components %d", robotName, sizeRobot, nComp);
  if ((sizeRobot > 4) || (nComp >= 1)) {//3
    printf("\n I %s am seeing a robot in the image of height %d components %d", robotName, sizeRobot, nComp);
    return 1;
  }
  return 0;
}

int waiting_color(int foreground) {//ok
  image = wb_camera_get_image(cam);
  waiting(1);
  // cronometer(IMAGE, 0); // disable because it's only one column
  int count = 0;
  count = cont_height_figure(101);
  if (flagPrint1) {
    printf("\n Intensity %d half line", count);
    flagPrint1 = 0;
  }
  if (count > 26){
    if ((stateUML == PICK_CACHE) || (stateUML == DROP_CACHE)) {
      cronometer(WAITING, 0); //shapeSeen); //when using different shapes
      cronometer(-1, 0); 
    } else {
      cronometer(-1, 0);
    }
    return 1; //keep waiting
  }    
  printf("\n Intensity gets down");
  if (foreground == BLACK) {
    color = MAGENTA;
    printf("\n Now, please %s, wait for cache delivery", robotName);
    while(waiting_color(MAGENTA)); 
  } 
  return 0; //wait no longer
}

int cont_height_figure(int indexP){ //ok
  int count=0;
  int maxCount = 0;
  int beginY = 0;
  int endX = width-1;
  int foreground = color;
  int i, j;
  switch (indexP){
    case -21: // checking red-nest ground color
      beginY = height - 5;
      foreground = RED; break;
    case -20: // checking blue-source ground color
      beginY = height - 5;
      foreground = BLUE; break;
    case -10: // On levy avoid colors 18 
      if (stateUML == PICK_CACHE) { foreground = BLUE;} // nest TAM
      else if (stateUML == DROP_CACHE) { foreground = RED;} // source TAM
      else { foreground = MAGENTA;} break; // cache TAM
    case -11:
      if (stateUML == TRAVEL2NEST) { foreground = RED;} // source TAM
      else if (stateUML == TRAVEL2SOURCE) {foreground = BLUE;} break; // nest TAM
    case 100: // checking tam_wall color
      foreground = TAM_WALL; break;
    case 101: // waiting on TAM 
      if ((stateUML == TRAVEL2NEST) || (stateUML == TRAVEL2SOURCE)){
        foreground = CYAN;
      } else {
        if (foreground != BLACK) { foreground = WHITE;}
      } break;
    default:  // Normal processing
      if ((indexP >= 0) && (indexP < width)) { endX = 0;}
  } 

  for (i = 0; i <= endX; i++) {
    if (endX == 0) { i = indexP;} // Only that point
    for (j = beginY; j < height; j++) {
      count += compareColorPixel(i, j, foreground);  
    } 
    if (count > maxCount) { maxCount = count;}
    if (beginY != (height - 5)) { count = 0;}
  } 
  //printf("\n count value %d index %d", maxCount, i);
  return maxCount;
}

int compareColorPixel(int pixelX, int pixelY, int foreground){ //ok-
  int auxColor = 0;
  int pixelR = wb_camera_image_get_red(image, width, pixelX, pixelY);
  int pixelG = wb_camera_image_get_green(image, width, pixelX, pixelY);
  int pixelB = wb_camera_image_get_blue(image, width, pixelX, pixelY);
  switch(foreground){
    case RED:
      auxColor = (pixelR > COLOR_THRES) && (pixelB < 20) && (pixelG < 20); //only red
      break;
    case GREEN:
      auxColor = (pixelG > COLOR_THRES) && (pixelB < LOW_THRES) && (pixelR < LOW_THRES); //only green
      break;  
    case BLUE:
      auxColor = (pixelB > COLOR_THRES) && (pixelR < LOW_THRES) && (pixelG < LOW_THRES); //only blue  
      break;
    case CYAN:     // green+blue
      auxColor = (pixelG > COMP_COLOR) && (pixelB > COMP_COLOR) && (pixelR < LOW_THRES);
      break;
    case MAGENTA:  // red+blue
      auxColor = (pixelR > COMP_COLOR) && (pixelB > COMP_COLOR) && (pixelG < LOW_THRES);
      break;
    case BLACK:
      auxColor = (pixelR < BLACK_THRES) && (pixelG < BLACK_THRES) && (pixelB < BLACK_THRES);
      break;
    case GREY:
      auxColor = (pixelR > ROBOT_THRES) && (pixelG > ROBOT_THRES) && (pixelB > ROBOT_THRES); 
      break;
    case WHITE:
      auxColor = (pixelR > COLOR_THRES) || (pixelB > COLOR_THRES);  
      break;
    case TAM_WALL:
      auxColor = (pixelR < LOW_THRES) && (pixelG < LOW_THRES) && (pixelB < LOW_THRES); 
      break;  
    default:
      auxColor =  0;
  }
  return auxColor;
}

int detectImage(int foreground, int shape, int numImage, int *numberComponents){ //ok
  int flagSeen = -1;
  int middleH = -1;
  int aux = 0;
  int newShapeSeen = 0;
  int distMiddle = 0;
  int maxProx = 0;
  int comp = 1;
  int nearest = 100;
  int realComp = 0;
  int i, j, k;
  int left, up;
  int minV = height, maxV = 0, minH = width, maxH = 0, area = 0;
  
  int imaComp[width][height];
  memset(imaComp, -1, width*height*sizeof(int));
  int relations[40];
  memset(relations, 0, 40*sizeof(int));
  int check[20];
  memset(check, 0, 20*sizeof(int));

  image = wb_camera_get_image(cam);
  wb_robot_step(TIME_STEP);
  cronometer(IMAGE, 0); // for image processing

  // Segmentation process
  for (i = 0; i < width; i++) {
    for (j = 0; j < height; j++) {
      aux = compareColorPixel(i, j, foreground); 
      if (aux){    
        // Identifying component through a N-neighborhood strategy
        left = i-1; 
        if (left < 0) { left = 0;}      
        up = j-1;
        if (up < 0) { up = 0;}
        // Case 1 - new 
        if ((imaComp[i][up] == -1) && (imaComp[left][j] == -1)){
          imaComp[i][j] = comp;
          relations[comp] = comp;
          comp++;
        } else {
          if (imaComp[left][j] > 0) {
            imaComp[i][j] = imaComp[left][j]; //Case 2
            if (imaComp[i][up] > 0) {
              relations[imaComp[i][up]] = imaComp[left][j];
              imaComp[i][up] = imaComp[left][j]; // save E2 = E1
            }
          } else {
            imaComp[i][j] = imaComp[i][up]; //Case 2
          }
        }
        wb_display_set_color(displayExtra, HEXWHITE);
      } else {
        wb_display_set_color(displayExtra, HEXBLACK);
      }
      wb_display_draw_pixel(displayExtra, i, j);
    }
  }
  // Replacing overlapping in components
  aux = 0;
  for (k = comp; 0 < k; k--) {
    if (relations[k] != k) {
      for (i = 0; i < width; i++) {
        for (j = 0; j < height; j++) {
          if (imaComp[i][j] == k) {
            imaComp[i][j] = relations[k];
          }
        }
      }
      comp--;
    } else {
      check[aux]=relations[k];
      aux++;
    }  
  }
  comp = aux;
  //  *numberComponents = comp; 
  //  FILE *fp4 = fopen("image_descritors_4n.csv","a");  
  for (k = 0; k < comp; k++) {
    // reset values to find them in a new component
    minV = height; maxV = 0; minH = width; maxH = 0; area = 0; 
    for (i = 0; i < width; i++) {
      for (j = 0; j < height; j++) {
        // If the pixel has the same component
        if (imaComp[i][j] == check[k]) {
          area++;
          // Checking boundaries
          if (maxH < i) { maxH = i;}
          if (minH > i) { minH = i;}
          if (maxV < j) { maxV = j;}
          if (minV > j) { minV = j;}
        } 
      }
    }
    if ((numImage == 255) && (color == CYAN)) {
      pointA = cont_height_figure(minH+1); 
      pointB = cont_height_figure(maxH-1);
      printf("\n Really close and sure it is not a robot, go for the center");
      return 100;
    }
//    if (((area > 10) && (foreground != CYAN)) || ((foreground == CYAN) && (area > 25))) { 
    if (((area > 10) && (foreground != CYAN)) || ((foreground == CYAN) && (area > 15))) { 
      int squareWidth = maxH-minH+1;
      int squareHeight = maxV-minV+1;  
      // Middle axis width within the square
      int middleAxisH = 0;
      aux = (int)squareHeight/2+minV;
      for (i = minH; i <= maxH; i++) {
        if (imaComp[i][aux] > 0) {
          middleAxisH++;  
        } 
      }
      wb_display_set_color(displayExtra, HEXRED);
      wb_display_draw_line(displayExtra, minH, aux, maxH, aux); 
      // Middle axis height within the square
      int middleAxisV = 0;
      aux = (int)squareWidth/2+minH;
      for (i = minV; i <= maxV; i++) {
        if (imaComp[aux][i] > 0) {
          middleAxisV++;
        }
      }
      wb_display_set_color(displayExtra, HEXRED);
      wb_display_draw_line(displayExtra, aux, minV, aux, maxV); 
      int x = aux; //middle index horizontal 
      int areaSquare = squareWidth * squareHeight;   
      float extent = (float) area/areaSquare;
      float eccentricity = (float) middleAxisV/middleAxisH;
      // Increase padding of 1 for window of component
      if (minV > 0) { minV--;}
      if (minH > 0) { minH--;}
      wb_display_set_color(displayExtra, HEXYELLOW);
      wb_display_draw_rectangle(displayExtra, minH, minV, squareWidth+1, squareHeight+1);
      // return the horizontal position as delta value
      distMiddle = abs(width/2-x);
      realComp++;
      *numberComponents = realComp;
      // A great enough region
      if ((squareWidth > 4) && (squareHeight > 4)) {
        //1 Triangle, 2 Box, 3 Circle, 4 Nothing, 0 ReallyNothing, 5 All, 6 Robot
         newShapeSeen = whatIsee(eccentricity, extent, squareWidth, middleAxisH, middleAxisV, numImage);
         if (shape == ROBOT){
           if (newShapeSeen == ROBOT) {
             if ((x > 23) && (x < 29) && (areaSquare > 600)) {
               waiting(15);
               printf("\n Robot %s found a robot in front of her", robotName);
             } 
             return squareHeight; //only returned when checkRobot is used  
           } 
         } else {
           switch(newShapeSeen){
             case NOTHING:
               //last value to check and nothing was seen clearly
               if ((flagSeen == -1) && (k == comp-1) && (squareHeight < 15)) { 
                 *numberComponents = 1; 
                 return 100;
               } break;
             case TRIANGLE:
             case CIRCLE:
             case BOX:
               if ((shape == ALL) || (shape == newShapeSeen)) { 
                 flagSeen = 1;
               } else if (k == comp-1){
                 return 100;
               }
               //if in k component was seen, then check if it is better
               if (flagSeen == 1) { 
                 if (maxProx < middleAxisH) {
                   nearest = distMiddle;
                   maxProx = middleAxisH;
                   middleH = x;
                   shapeSeen = newShapeSeen;
                   //printf("\n Robot %s found a new shape %d higher %d and closer to the middle %d", robotName, newShapeSeen, maxProx, middleH);
                 } else if (maxProx == middleAxisH) {
                   if (nearest > distMiddle) {
                     nearest = distMiddle;
                     middleH = x;
                     shapeSeen = newShapeSeen;
                     //printf("\n Robot %s found a new shape %d just closer to the middle %d", robotName, newShapeSeen, middleH);
                   }
                 }
                 flagSeen = 0;
               }              
               *numberComponents = realComp; 
               break;
           }  
        }
      }
    }  
  }
  *numberComponents = realComp;
  return middleH; 
}      

int whatIsee(float Eccentricity, float Extent, int squareWidth, int middleAxisH, int middleAxisV, int numImage){
    // 1 Triangle, 2 Box, 3 Circle, 4 Nothing, 5 All, 6 Robot, -1 ReallyNothing
    int shapeFound = -1; // Weka 3rd generation 16feb16
    if (Eccentricity <= 1.2) {
      if (Extent <= 0.889) {
        if (Extent <= 0.711) {
          if (squareWidth <= 11) {
            //printf("\n Circle (4.0/1.0)");
            shapeFound = CIRCLE;
          } else {
            //printf("\n Robot (176.0)");
            shapeFound = ROBOT;
          }
        } else {
          if (Eccentricity <= 0.818) {
            //printf("\n Robot (6.0)");
            shapeFound = ROBOT;
          } else {
            //printf("\n Circle (135.0/1.0)");
            shapeFound = CIRCLE;
          }
        }
      } else {
        //printf("\n Box (132.0)");
        shapeFound = BOX;
      }
    } else {
      if (Extent <= 0.708) { 
        if (Extent <= 0.474) {
          //printf("\n Robot (20.0/1.0)");
          shapeFound = ROBOT;
        } else {
          if (Eccentricity <= 2.455) {
            if (Extent <= 0.613) {
              if (middleAxisH <= 3) {
                //printf("\n No triangle (4.0)");
                shapeFound = NOTHING;
              } else {
                //printf("\n Triangle (99.0/3.0)");
                shapeFound = TRIANGLE;
              } 
            } else {
              if (squareWidth <= 7) {
                if (middleAxisV <= 7) {
                  //printf("\n Triangle (15.0)");
                  shapeFound = TRIANGLE;
                } else {
                  //printf("\n No circle (5.0)");
                  shapeFound = NOTHING;
                }
              } else {
                //printf("\n No triangle (34.0)");
                shapeFound = NOTHING;
              }
            }
          } else {
            //printf("\n No triangle (35.0/1.0)");
            shapeFound = NOTHING;
          }
        }
      } else {
        //printf("\n No circle (105.0)");
        shapeFound = NOTHING;
        if (color == CYAN ){ shapeFound = BOX;}
      }    
    }  
    return shapeFound;
}

int doubleCheck(){
  int index = -1;
  int nComp;
  run(10); //forward(5);
 
  index = detectImage(color, figura, 1, &nComp);
  if ((index == -1) || (index == 100)){
    printf("\n False alarm %d- %s continue my searchingT", index, robotName);
    return -1;
  } 
  printf("\n Shape %d watched on 2check", figura);
  return index; 
}

void avoidance(){ //ok
  int sense = 1;
  waiting(1);
  readSensors(0);
  if ((ps_value[7] + ps_value[6]) > (ps_value[0] + ps_value[1])) {
    printf("\n Obstacle in left-side");
  } else {
    printf("\n Obstacle in right-side");
    sense = -1;
  }
  turnSteps(TURN_90*sense);
  forward(18); //15
  turnSteps((TURN_M90-3)*sense);
}

int followingLine(int colorLine, int entrace){//ok-
  int delta = 0;
  int entering = 0;
  int flagRobot = 0;
  readSensors(0);
  entering = ps_value[5] > 50;
  while(entering) { 
    readSensors(0);
    if ((ps_value[0] > THRESHOLD_DIST) || (ps_value[7] > THRESHOLD_DIST)){ 
      waiting(20);  
      printf("\n %s something is in front of me", robotName);
    } else {
      image = wb_camera_get_image(cam);
      // cronometer(IMAGE, 0); // Disable because it is only one row
      if (entrace) {         
        delta = find_middle(1);
      } else {
        delta = find_middle(0);
      }  
      //printf("\n value sensor 5 %d and delta %d", ps_value[5], delta);
      if (delta > -1) {
        delta = delta - width/2;
        speed[LEFT] = 220 - K_TURN*abs(delta);
        speed[RIGHT] = 220 - K_TURN*abs(delta);
        
        wb_differential_wheels_set_speed(speed[LEFT]+K_TURN*delta,speed[RIGHT]-K_TURN*delta);
        wb_robot_step(TIME_STEP);
        cronometer(-1, 0); 
      } else {
        if (delta < 0) {
          flagRobot = check4Robot();
          if (flagRobot) {
            waiting(20);
            //printf("\n Robot is out by line");
          } else {
            printf("\n %s is lost from the line", robotName);
            return 1;//-1
          }  
        } else {
          return -2;
        }
      }
    }  
  }  
  return 1;  
}

void turnSteps(int steps){ //ok-
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
    cronometer(-1, 0);
  }
  waiting(1);
}

int run(int steps){ //ok-
  int i, j;
  int matrix[8][2] = {{150,-35},{100, -15},{80, -10},{-10,-10},{-10,-10},{-10,80},{-30,100},{-20,150}};
  while(steps > 0) {  
    readSensors(0);
    for (i = 0; i < 2; i++) {
      speed[i] = 0;
      for (j = 0; j < NB_DIST_SENS; j++) {
        // 0.002 = 1/HalfRange = 512 
        speed[i] += matrix[j] [i] * (1 - ps_value[j]*0.002);
      }
      if (speed[i] > MAX_SPEED) {
        speed[i] = MAX_SPEED;
      } else if (speed[i] < -MAX_SPEED) {
        speed[i] = -MAX_SPEED;
      }
    }
    
    wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);
    wb_robot_step(TIME_STEP);
    cronometer(-1, 0); 
        
    steps--;
    // Every 5 steps check ground color
    if(steps%5 == 0){ whereIam(1);}
  } 
  waiting(1);
  return 1;
}

void forward(int steps){ //ok-
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
    cronometer(-1, 0); // -1 for let it find the task
  }
  waiting(1);  
}

int levyFlight(){
  // rand() % (max_n - min_n + 1) + min_n;
  int index = -1;
  int nComp;
  int r = rand()%(54-27+1)+27; 
  // Robot needs about 7 steps at 200 speed to have a new image
  printf("\n Robot %s turning random %d", robotName, r); 
  while (r > 0) {
    turnSteps(3); // Blind turn
    r -= 3;
    whereIam(1);
    
    image = wb_camera_get_image(cam);
    wb_robot_step(TIME_STEP);
    // cronometer(IMAGE, 0) //It is only a line
    //listening();
    
    if (cont_height_figure(-10) > 15) {//18
      printf("\n Backward invading useful region on turn");
      forward(-30); //70
    }
    if ((color == CYAN) && (cont_height_figure(-11) > 22)) { //25
      printf("\n Backward invading on turn %d",cont_height_figure(-11));
      forward(-30); //70
    }
    index = detectImage(color, figura, 0, &nComp); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
        return doubleCheck();
      }
      printf("\n Shape watched on levy - Levy Aborted %d", index);
      return index;  
    } 
  } 
  r = rand()%(600-40)+41; // walk forward between 100 to 40 steps
  printf("\n %s Walking forward %d", robotName, r);
  wb_differential_wheels_set_encoders(0,0);
  while (r > 0) {
    run(5); // Blind walk
    r -= 5;
    whereIam(1);

    image = wb_camera_get_image(cam);
    wb_robot_step(TIME_STEP);
    // cronometer(IMAGE, 0) //It is only a line
    //listening();
    
    if (cont_height_figure(-10) > 15) { //18
      printf("\n Backward invading useful region on walk");
      forward(-30); //70
      turnSteps(TURN_CACHE);
    }   
    if ((color == CYAN) && (cont_height_figure(-11) > 22)) {
      printf("\n Backward invading on walk %d", cont_height_figure(-11));
      forward(-20); //70
      turnSteps(TURN_CACHE);
    } 
    index = detectImage(color, figura, 0, &nComp); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
        return doubleCheck(); 
      }
      printf("\n Color watched on levy - Levy Aborted %d", index);
      return index;  
    } 
  }
  return index;
}

int speedAdjustment(int index, int delta) { //ok
  int count = cont_height_figure(index);
  //printf("\n According to direction defined %d by %s the height is %d", index, robotName, count);
  int iter=0;
  if ((index >= 0) && (index < height)) {
    iter = count-6;
  } else {
    iter = (MAX_SPEED*height)/(MAX_SPEED+BACKWARD_SPEED);
  } // increase by 1.25 max_speed
  speed[LEFT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/height;
  speed[RIGHT] = MAX_SPEED-(MAX_SPEED+BACKWARD_SPEED)*iter/height;
  // The robot is close enough to the object, i.e., > 75%  
  if (count > PROXIMITY_COLOR) {
    if ((stateUML == DROP_CACHE) && (flagPartition)) {
      printf("\n Robot %s would like to partition on shape %d", robotName, shapeSeen);
      //speaking(shapeSeen, shapeSeen, shapeSeen); // to make a rendesvouz in a shape
    }
    resetDisplay();
    flagRobot = check4Robot();
    if (color == CYAN){   
      if (flagRobot) { 
        forward(-15);
        printf("\n %s found a robot when going to Landmark", robotName);
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      detectImage(CYAN, ALL, 255, &iter);
      //printf("\n Difference between A-B %d", pointA-pointB);
      iter = pointA - pointB;
      if (iter > 2) { hitWall(5);}
      else if (iter < -2) { hitWall(-5);}
      else { hitWall(0);}
      printf("\n Cyan landmark reached!");
      waiting(1);      
      return 1;
    } else {
      //printf("\n Robot %s is near but...", robotName);
      if (flagRobot) {
        printf("\n %s found another robot there", robotName);
        forward(-5);       
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      forward(20);
      // HERE SEEMS TO BE THE PROBLEM
      hitWall(1);
      forward(-7);
      delta = ps_value[0] + ps_value[1] - ps_value[7] - ps_value[6];
      if (delta > THRESHOLD_DIST) { turnSteps(3);} // almost 10°
      else if (delta < THRESHOLD_DIST) { turnSteps(-3);}
      switch(color){
        case BLUE:
          printf("\n Robot %s is by color BLUE", robotName); break;
        case RED: 
          printf("\n Robot %s is by color RED", robotName); break;
        case MAGENTA: 
          printf("\n Robot %s is by color MAGENTA", robotName); break;
        case BLACK: 
          printf("\n Robot %s is by color BLACK", robotName); break;   
      }
      waiting(1);
      return 1;
    }   
  } else { //before being close enough
    // printf("\n %s saw shape with height %d", robotName, count);
    if (readSensors(0) && ((ps_value[0] > THRESHOLD_DIST) || (ps_value[1] > THRESHOLD_DIST) 
    || (ps_value[7] > THRESHOLD_DIST) || (ps_value[6] > THRESHOLD_DIST))) { // 1 for obstacle
      printf("\n %s found obstacle on the way", robotName);
      avoidance();
    }
    flagRobot = check4Robot();
    if (flagRobot) {
      printf("\n I %s found another robot there", robotName);
      // rand() % (max_n - min_n + 1) + min_n;
      if (rand()%100 > 50) { waiting(10);}
      else { turnSteps(6);}
      return 0;  
    }
    wb_differential_wheels_set_speed(speed[LEFT]+K_TURN*delta,
                                     speed[RIGHT]-K_TURN*delta);
    wb_robot_step(TIME_STEP); 
    cronometer(-1, 0); //-1 for movements
  }
  return 0;
}

int hitWall(int front){ //ok
  int hit_thres = 200, flag = 1;
  int question;
  speed[LEFT] = 300;
  speed[RIGHT] = 300;
  if (front == 5) { // 6 is 1.5 x K_TURN
    speed[LEFT] = 300 + 50;//6*(pointA - pointB);
  } else if (front == -5) {
    speed[RIGHT] = 300 + 50;//6*(pointB - pointA);
  }
  while(flag){
    readSensors(0);
//    printf("\n sensors 0 %d, 1 %d, 6 %d, 7 %d", ps_value[0], ps_value[1], ps_value[6], ps_value[7]);
    if ((front == 0) || (abs(front) == 5)) {
      question = (ps_value[0] > hit_thres) || (ps_value[7] > hit_thres) || (ps_value[6] > hit_thres) || (ps_value[1] > hit_thres);
    } else {
      question = (ps_value[0] > hit_thres) || (ps_value[7] > hit_thres);
    }  
    if (question) {
      waiting(5);
      readSensors(0);
      if (question || (ps_value[6] > hit_thres) || (ps_value[1] > hit_thres)){
        flag = 0;
      }
    }    
    
    wb_differential_wheels_set_speed(speed[LEFT], speed[RIGHT]);
    wb_robot_step(TIME_STEP);
    cronometer(-1, 0); 
    
  }
  waiting(1);
  printf("\n Robot %s hit against a wall...", robotName);
  return 1;
}

int hitLandmark(){
  hitWall(0);
  int flagRobot = check4Robot();
  // She saw a robot or not Cyan color in front
  if (flagRobot) {
    printf("\n False Cyan landmark, %s", robotName);
    forward(-20);
    return 0;
  } 
  printf("\n Begining adjust...");
  forward(2);
  readSensors(0);
  // hit by sensor 1, turn almost 20 degrees
  if ((ps_value[0] > THRESHOLD_DIST) || (ps_value[1] > THRESHOLD_DIST)) { turnSteps(10);} 
  speed[LEFT] = 100;
  speed[RIGHT] = -100;
  int notReady = 1;
  int counter = 0;
  while(notReady) { 
    wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);      
    readSensors(0);
    counter++;
    if (ps_value[5]> 250) {
      notReady = find_middle(1) < 0;
    } else {
      flagRobot = check4Robot();
      if (flagRobot) {
        printf("\n %s find another robot here",robotName);
        return 0;
      }
    }
    cronometer(-1, 0); 
  }
  printf("\n Ending hitLandmark");
  //waiting(1);
  return 1;
}

int checkingArrival(int toward){

    run(100);
    // To add randomness in the entrance 
    //if (luck > 0.5) { turnSteps(-15);}  
    if (toward == SOURCE) {
      if (!flagLoad) {  
        turnSteps(TURN_M90); 
      } else {
        turnSteps(TURN_90);
      }
      printf("\n Robot %s ended her walk to Source", robotName);
    } else {
      if (flagLoad) {
        turnSteps(TURN_90);
        forward(5);  
      } else {
        turnSteps(TURN_M90);
      }
      printf("\n Robot %s ended her walk to Nest", robotName);
    }
    waiting(2);
    return 1;
}


int detectTam(){ //ok
  image  = wb_camera_get_image(cam);
  wb_robot_step(TIME_STEP);
  // cronometer(IMAGE, 0) //It is only a line
  
  waiting(1); 
  if (cont_height_figure(101) < 35){ //30 checking wall tam
    printf("\n Something went wrong entering");
    printf("\n");
    waiting(1);
    return 0;  
  } 
  return 1;  
}      

int enterTam(){ //ok
  // 1 blue, 2 red, and 3 magenta
  int flag1stCheck = 0; 
  int dir = 0, enteredBy = 0;//, nComp,  index = 26;
  //1 rigth and -1 left
  int left_side = ps_value[7] + ps_value[6];
  int right_side = ps_value[1] + ps_value[0];
  if (right_side > left_side) {
    dir = 1;
    if (ps_value[0] > THRESHOLD_DIST) { enteredBy = 1;}
    printf("\n case right %d", enteredBy); 
  } else {
    dir = -1;
    if (ps_value[7] > THRESHOLD_DIST) { enteredBy = 1;}
    printf("\n case left %d", enteredBy); 
  }
  flag1stCheck = detectTam(); 
  if (!flag1stCheck) { 
    turnSteps(-26*dir);
    waiting(1);
    hitWall(1); 
    forward(-6);//8
  }  
  if ((ps_value[0] > THRESHOLD_DIST) && (ps_value[7] > THRESHOLD_DIST) && (ps_value[6] < THRESHOLD_DIST) && (ps_value[1] < THRESHOLD_DIST)) {
    printf("Everything is fine!!");
    return 1; 
  } else {    
    flag1stCheck = detectTam();
    if (!flag1stCheck) {
      printf("\n %s am not inside TAM correctly", robotName);
      printf("\n");
      while(!waiting(10));
      return 0;
    } else {
      printf("\n Robot %s entered successfully square",robotName);
      waiting(2);
      return 1;  
    }  
  }
  return -1;    
}

int adjustWallTurn(int steps){
  forward(10);
  turnSteps(TURN_M90);
  forward(steps);
  waiting(1);
  if (find_middle >= 1) {
    return 0; 
  } else {
    return 1;
  }  
}

int going2Source(){ //ok
  int endTask = 0;
  resetDisplay();
  printf("\n %s getting in position destination Source", robotName);
  readSensors(0);

  if (ps_value[5] < 50){
    printf("\n %s sensor is far from redWall", robotName);
    return 0;
  }  
  while(endTask == 0) {
    endTask = followingLine(BLUE, 1);
  }
  printf("\n Robot %s going on 1st wall", robotName);
  endTask = adjustWallTurn(30);
  while(endTask <= 0) {
    endTask = followingLine(RED, 0);
  }
  forward(60); //30
  endTask = checkingArrival(SOURCE);
  if (whereIam(0) == BLUE) {
    endTask = 1;
  } else {
    printf("\n Robot %s did something wrong going to source", robotName);
    endTask = 0;
  }  
  return endTask;
}

int going2Nest(){ //ok
  int endTask = 0;
  resetDisplay();
  readSensors(0);
  printf("\n %s getting in position destination Nest", robotName);

  if (ps_value[5] < 50) {
    printf("\n %s sensor is out of blueWall", robotName);
    return 0;
  }
  while (endTask == 0){
    endTask = followingLine(RED, 1);
  } 
  printf("\n Robot %s going on 1st wall", robotName);
  endTask = adjustWallTurn(30);
  while(endTask <= 0) {
    endTask = followingLine(BLUE, 0); //0 only on the 3rd wall
  }

  printf("\n Robot %s going on 2nd wall", robotName);
  endTask = adjustWallTurn(30);  
  while(endTask <= 0) {
    endTask = followingLine(BLUE, 0); //0 only on the 3rd wall
  }
  printf("\n Robot %s going on 3rd wall", robotName);
  endTask = adjustWallTurn(30);
  while(endTask <= 0) {
    endTask = followingLine(BLUE, 0); //0 only on the 3rd wall
  }
  forward(60);
  endTask = checkingArrival(NEST); 
  if (whereIam(0) == RED) {   
    endTask = 1;
  } else {
    printf("\n Robot %s did something wrong going to nest", robotName);
    endTask = 0;
  }  
  return endTask;
}

int going2it(int index){//ok
  int intensity[width]; //int *intensity = (int *)malloc(sizeof(int)*width);
  int i = 0, index2 = 0, delta = 0;
  int count = 0;

  image = wb_camera_get_image(cam);
  wb_robot_step(TIME_STEP);
  cronometer(IMAGE, 0);
  
  if (index == 100) {
    for (i = 0; i < width; i++) {
      intensity[i] = cont_height_figure(i);
    }
    for (i = 0; i < width; i++) {
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
      delta = index + (index2-index)/2 - (width/2);
      index = (width/2) + delta;
    } else {

      delta = index - (width/2);
    }  
  } else if ((index >= 0) && (index < width)) {
    delta = index-(width/2);
  } else {
    printf("\n no direction defined %s", robotName);
    return 0;
  }
  return speedAdjustment(index,delta);  
}

void cronometer(int task, int cache){//ok-
  createDir(LIFE, 0);
  FILE *flife = fopen(fileRobot,"a+");
  
  if (task == IMAGE) { 
    timeImage++;
    fprintf(flife, "image, %d \n", timeImage);
  } else if (task == WAITING) {
    if (stateUML == DROP_CACHE) {
      WaitingCacheDrop[cache]++;
      fprintf(flife, "drop %d, %d\n", cache, WaitingCacheDrop[cache]);
    }	else if (stateUML == PICK_CACHE) {
      WaitingCachePick[cache]++;
      fprintf(flife, "pick %d, %d\n", cache, WaitingCachePick[cache]);
    } 
  } else {  
    timeMeasured++;
    fprintf(flife, "state %d, %d\n", stateUML, timeMeasured);
  }
  fclose(flife);  
  listening();
}

void countObjects(){
  switch(stateUML){
    case STORE:
      nStore++; break;
    case HARVEST:
      nHarvest++; break;
    case DROP_NEST:
      nDrop++; break;
    case PICK_SOURCE:
      nPick++; break;
  }
  createDir(PERFORMANCE, 0);
  FILE *fper = fopen(fileRobot, "a+");
  fprintf(fper, "%d, %d, %d, %d\n", nPick, nDrop, nHarvest, nStore);
  printf("\n Updated UCB");
  fclose(fper);
}

void updateEstimations(int task, int value, int cache){ //ok-
  //0 means no listen nothing 1 forget all for new data
  int codeTask = task; 
  if (flagMomento != 1) { beta = 0;}
  if (flagListened == 0) {
    value = timeMeasured;
    wb_robot_step(32);
    //printf("\n for state %d time measured is %d", codeTask, value);
  } 

  switch(task){
    case PICK_CACHE:
      //value += WaitingCachePick[cache]; //need to disable count in waiting_color
      estPickC = (estPickC * (100 - alpha) + value * alpha + (estPickC - value) * beta) / 100;
      break;
    case DROP_CACHE:
      //value += WaitingCacheDrop[cache]; //need to disable count in waiting_color
      estDropC = (estDropC * (100 - alpha) + value * alpha + (estDropC - value) * beta ) / 100;
      break;
    case PICK_SOURCE:
      estPickS = (estPickS * (100 - alpha) + value * alpha + (estPickS - value) * beta) / 100;
      break;
    case DROP_NEST:
      estDropN = (estDropN * (100 - alpha) + value * alpha + (estDropN - value) * beta ) / 100;
      break;
    case STORE:
      estStore = (estStore * (100 - alpha) + value * alpha + (estStore - value) * beta ) / 100;
      break;
    case HARVEST:
      estHarvest = (estHarvest * (100 - alpha) + value * alpha + (estHarvest - value) * beta ) / 100;
      break;
    case TRAVEL2NEST:
      if (flagHold) {
        switch (statePrevious) {
        case PICK_SOURCE:
          value += estPickC;
          estPickS = (estPickS * (100 - alpha) + value * alpha + (estPickS - value) * beta ) / 100;
          codeTask = PICK_SOURCE;
        break;
        case DROP_CACHE:
          value += estDropN;
          estDropC = (estDropC * (100 - alpha) + value * alpha + (estDropC - value) * beta ) / 100;
          codeTask = DROP_CACHE;
        break;
        case HARVEST:
          value += estPickC;
          estHarvest = (estHarvest * (100 - alpha) + value * alpha + (estHarvest - value) * beta ) / 100;
          codeTask = HARVEST;
        break;
      }
    }   break;
    case TRAVEL2SOURCE:
      if (flagHold) {
        switch (statePrevious) {
        case PICK_CACHE:
          value += estPickS;
          estPickC = (estPickC * (100 - alpha) + value * alpha + (estPickC - value) * beta ) / 100;
          codeTask = PICK_CACHE;
        break;
        case DROP_NEST:
          value += estDropC;
          estDropN = (estDropN * (100 - alpha) + value * alpha + (estDropN - value) * beta ) / 100;
          codeTask = DROP_NEST;
        break;
        case STORE:
          value += estDropC;
          estStore = (estStore * (100 - alpha) + value * alpha + (estDropC - value) * beta ) / 100;
          codeTask = STORE;
        break;
      }
    } break;    
  }
  cache = 0; // comment when working with shapes
  updateBitacora(0, ESTIMATIONS, cache);
  if (codeTask != IMAGE) { 
    updateBitacora(codeTask, FSM, cache); 
    if (flagListened) {
      flagListened = 0;
    } else {
      printf("\n Everybody listen, I am %s, my %d cost me %d", robotName, codeTask, value);
      speaking(codeTask, value, cache);
    }  
  } 
  lastImage = timeImage;
  timeImage = 0;  
  timeMeasured = 0;
  timeListened = 0;
  WaitingCachePick[cache] = 0;
  WaitingCacheDrop[cache] = 0;
  wb_robot_step(32);
}

void updateBitacora(int codeTask, int estimations, int cache){ //ok-
  if (estimations == ESTIMATIONS) { 
    createDir(ESTIMATIONS, 0); 
    FILE *fbot = fopen(fileRobot, "a+");
    if (fbot==NULL) {
      printf("Error opening file of estimations bot\n");
      exit(1);
    }
    
    if (flagListened == 1) {
      fprintf(fbot, "Update after heard for cache %d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n", 
                 cache, estStore, estHarvest, estPickS, estDropC, 
                 estPickC, estDropN, WaitingCacheDrop[cache], WaitingCachePick[cache], lastImage);
    } else {
      fprintf(fbot, "Update after finish for cache %d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n", 
                 cache, estStore, estHarvest, estPickS, estDropC, 
                 estPickC, estDropN, WaitingCacheDrop[cache], WaitingCachePick[cache], lastImage);
    }             
    fclose(fbot);
  } else {    
    createDir(FSM, 0); 
    FILE *fbot = fopen(fileRobot, "a+");    
    if (fbot==NULL) {
      printf("Error opening file bot\n");
      exit(1);
    }
    
    char stringState[] = "SEARCHING SOMETHING";
    int innerState = stateUML;
    if (flagListened) { innerState = codeTask;}
    
    switch(innerState){
    case PICK_SOURCE:
      sprintf(stringState, "PICK SOURCE"); break;
    case DROP_CACHE:
      sprintf(stringState, "DROP CACHE-%d", cache); break;
    case PICK_CACHE:
      sprintf(stringState, "PICK CACHE-%d", cache); break;    
    case DROP_NEST:
      sprintf(stringState, "DROP NEST"); break;
    case STORE:
      sprintf(stringState, "STORE"); break;
    case HARVEST:
      sprintf(stringState, "HARVEST"); break;    
    case TRAVEL2SOURCE:
      sprintf(stringState, "TRAVEL HARVEST");
      if (flagHold) {
        switch(statePrevious){
        case PICK_CACHE:
          sprintf(stringState, "PICK CACHE-L%d", cache); break;
        case DROP_NEST:
          sprintf(stringState, "DROP NEST-L"); break;  
        case STORE:
          sprintf(stringState, "STORE LEVY"); break;  
        }
      } break; 
    case TRAVEL2NEST:
      sprintf(stringState, "TRAVEL STORE");
      if (flagHold) {
        switch(statePrevious){
        case DROP_CACHE:
          sprintf(stringState, "DROP CACHE-L%d",cache); break;
        case PICK_SOURCE:
          sprintf(stringState, "PICK SOURCE-L"); break;  
        case HARVEST:
          sprintf(stringState, "HARVEST LEVY"); break;  
        }
      } break;
    }
    if (flagListened) {
      fprintf(fbot,"Listened %s, 0, %d\n", stringState, timeListened);
    } else {
      fprintf(fbot,"Executed %s, %d, %d\n", stringState, codeTask, timeMeasured);
    }
    fclose(fbot);             
  }               
} 

void writeDecision(float boundP, float realP, int mechanism){ //ok-
  // File for decisions
  createDir(DECISIONS, 0); 
  FILE *file = fopen(fileRobot, "a+");
  if (file==NULL) {
    printf("Error opening file of estimations bot\n");
    exit(1);
  }

  if (mechanism == PARTITION_TASK) {
    fprintf(file, "\n Partitioning %d, %.2f, %.2f", boundP>realP, boundP, realP);
  } else if (mechanism == PARTITION_LEVY){
    fprintf(file, "\n Partitioning-Levy %d, %.2f, %.2f", boundP>realP, boundP, realP);
  } else if (mechanism == PARTITION_HEAR){
    fprintf(file, "\n Partitioning by hearing %d, 99, 99", flagPartition);
  } else if (mechanism == GIVEUP_LEVY){
    fprintf(file, "\n I give up on Levy %d, %.2f, %.2f, %d", boundP>realP, boundP, realP, timeMeasured);
  } else if (mechanism == GIVEUP_CACHE){
    fprintf(file, "\n I give up %d, %.2f, %.2f, %d", boundP > realP, boundP, realP, timeMeasured);
  } else if (mechanism == GIVEUP_HEAR){
    fprintf(file, "\n I have flag give up on %d, 99, 99, -1", flagGiveup);
  } 
  fclose(file);
}

int speaking(int codeTask, int time, int cache){ //ok-
  if (flagCom == 0) { return 0;}

  char message[30];
  // wb_emitter_set_channel(emitter, WB_CHANNEL_BROADCAST);
  if (time == -1) { // reporting just to have the same number of lines
    sprintf(message, "P");
    wb_emitter_send(emitter, message, strlen(message)+1);
  } else if (time == -2) {
    sprintf(message, "G");
    wb_emitter_send(emitter, message, strlen(message)+1);
  }  else {
    sprintf(message, "Me%c%c%c%cC%dT%dX%d",robotName[6], robotName[7], robotName[8], robotName[9], codeTask, time, cache);
    wb_emitter_send(emitter, message, strlen(message)+1);
  }
  wb_robot_step(32);
  return 1;
}

int listening() { //ok-
  while(wb_receiver_get_queue_length(receiver)>0){  
    const char *data = wb_receiver_get_data(receiver);
    // a stores x, y and z
    const double *a = wb_receiver_get_emitter_direction(receiver);
    double signal = wb_receiver_get_signal_strength(receiver);
    //printf("\n I am %s and receive %s Signal strength %g from (%g, %g, %g)", robotName, data, signal, a[0], a[1], a[2]);
    double theta = angle(a[2],a[0]);
    //printf("\n Received %c from angle %.2f", data[0], theta);
    
    if (data[0] == 'P') {
      //printf("\n %s will update its state of partitioning %d", robotName, flagPartition);
      writeDecision(0, 0, PARTITION_HEAR);
      wb_receiver_next_packet(receiver);
      wb_robot_step(32);
      return 0;
    } else if (data[0] == 'G') {
      //printf("\n %s will update its state of give up %d", robotName, flagGiveup);
      writeDecision(0, 0, GIVEUP_HEAR);
      wb_receiver_next_packet(receiver);
      wb_robot_step(32);
      return 0;
    } 
    /*  
       A standard message Me0000C777T9999X3
       Me0000 = number of robot
       C777 = code of task
       T9999 = time of task
       X3 = type of cache found //still not used
    */
    int name = atoi(&data[2]);
    int i;
    for (i = 0; i < nRobots; i++){
      if (name == listFriends[i]){
        // proceed to listen the information
      }
    }
    int codeReceived = atoi(&data[7]);
    timeListened = atoi(&data[11]);
    char *p = (char*) data;
    p+=11;
    while(*p) {
      if (*p == 'X') { 
        p++;	  
        break; 
      } else {
        p++;
      }
    }
    int cacheReceived = atoi(p);
    /* Robot codes 
    301 TRIANGLE     100 timePickSource       106 timeStore
    302 BOX          101 timepickCache        107 timeHarvest
    303 CIRCLE       102 timeDropCache        201 timeImage
    304 ALL          103 timeDropNest         202 timeCache
    305 NOTHING      104 timeTravel2Nest      777 waiting
    306 ROBOT        105 timeTravel2Source
    */
    if ((codeReceived >= 300) && (codeReceived <= 304)) { 
      if (stateUML == PICK_CACHE){
        figura = codeReceived;
        //color = BLACK;
        printf("\n %s got news about figure %d ", robotName, figura);
      }
    } else if ((codeReceived >= 100) && (codeReceived <= 107)){	
      printf("\n Thanks %d buddy, %s will consider your estimations for %d of time %d", name, robotName, codeReceived, timeListened);
      flagListened = 1;
      wb_robot_step(32); // to update global values
      updateEstimations(codeReceived, timeListened, cacheReceived);
    }
    wb_receiver_next_packet(receiver);
  }  
  return 1;
}

int computePartition (int levy){ //ok-
  float Ppartitioning = 0;
  float p = 0;
  float tFull;
  float tPart;
  // task1 = {Pick-source, Drop-Cache}
  // task2 = {Pick-cache, Drop-Nest}

  tFull = estHarvest + estStore;
  if (tFull == 0) { tFull = 1;}

  tPart = estPickS + estDropC + estPickC + estDropN;  
  if (tPart == 0) { tPart = 1;}
  
  switch(modelTest){
    case ALWAYS:
      return 1;
    break;
    case NEVER:
      return 0;
    break; 
    case UCB:
      if ((stateUML == DROP_NEST) || (stateUML == STORE)) {
        tPart = estPickC - gammaUCB*sqrt(2*log(nPick+nHarvest)/nPick);
        tFull = estHarvest - gammaUCB*sqrt(2*log(nPick+nHarvest)/nHarvest); 
      } else if ((stateUML == PICK_SOURCE) || (stateUML == HARVEST)) {
        tPart = estDropC - gammaUCB*sqrt(2*log(nDrop+nStore)/nDrop);
        tFull = estStore - gammaUCB*sqrt(2*log(nDrop+nStore)/nStore);       
      } 
      if (tFull < tPart) { //Non-partitioning is more profitable
        p = 1;
        Ppartitioning = 0;
      } else { //partitioning is more profitable
        p = 0;
        Ppartitioning = 1;        
      } break;
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
    case BASE2011:
      // Steepness factor 2.5 from table 1 pag 14 2011
      sParam = 2.5;
    case BASE2013:
    case MINE:
      if ((levy) && (modelTest == MINE)) {
        switch(stateUML){
          case PICK_SOURCE:
            //printf("\n %s levy pick_source", robotName);
            tPart = timeMeasured + estDropC + estPickC + estDropN; 
          break;
          case DROP_NEST:
            //printf("\n %s levy drop_nest", robotName);
            tPart = estPickS + estDropC + estPickC + timeMeasured;
          break;                 
          case HARVEST:
            //printf("\n %s levy harvest", robotName);
            tFull = timeMeasured + estStore;
          break;
          case STORE:
            //printf("\n %s levy store", robotName);
            tFull = estHarvest + timeMeasured;
          break; // to update global values
          case TRAVEL2NEST:
          case TRAVEL2SOURCE:
            switch(statePrevious){
              case PICK_SOURCE:
                //printf("\n %s levy travel from pick-source", robotName);
                tPart = timeMeasured + estDropC + estPickC + estDropN;
              break;
              case DROP_CACHE:
                //printf("\n %s levy travel from drop-cache", robotName);
                tPart = estPickS + timeMeasured + estPickC +  estDropN;
              break;
              case HARVEST:
                //printf("\n %s levy travel from harvest", robotName);
                tFull = timeMeasured + estStore;
              break;
              case PICK_CACHE:
                //printf("\n %s levy travel from pick-cache", robotName);
                tPart = estPickS + estDropC + timeMeasured + estDropN;
              break;
              case DROP_NEST:
                //printf("\n %s levy travel from drop-nest", robotName);
                tPart = estPickS + estDropC + estPickC + timeMeasured; 
              break;
              case STORE:
                //printf("\n %s levy travel store", robotName);
                tFull = estHarvest + timeMeasured;
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
    writeDecision(Ppartitioning, p, PARTITION_LEVY);
  } else {
    writeDecision(Ppartitioning, p, PARTITION_TASK);
  }
  
  int result = Ppartitioning > p;
  if (result){
    printf("\n %s goes by partitioning", robotName);
  } else {
    printf("\n %s do the full task", robotName);
  }
  speaking(-1, -1, -1);
  wb_robot_step(32); // to update global values
  return result;
}

int computeGiveUp(int levy, int cache){ //ok-
  if (flagMasterGiveup == 1) { // Always
    return 1;
  } 
  // K = 0.6 and O = 5 from table 1 pag 14 2011
  float kParam = -0.6;
  int oParam = -5;
  int mParam = 1;
  float Pgive = 0;
  float p = 1;

  int tCache = timeMeasured;
 
  int tTask;
  if (stateUML == DROP_CACHE) {
    tTask = estDropC;
    //tCache = WaitingCacheDrop[cache];
  } else {
    tTask = estPickC;
    //tCache = WaitingCachePick[cache];
  }
 
  int estPart = estPickC + estDropC;
  if (estPart <= 0) { estPart = 1;}

  
  switch(modelTest){
    case ALWAYS:
    case NEVER:
      return 0;
    break; 
    case UCB:
    case GREEDY: 
    case BASE2013:
      tTask = estPart / 2;
      if (timeMeasured > 3*tTask) { // According to Eq. 4 in 2013
        Pgive = 1;
        p = 0;
      } break;
    case BASE2011:
    case MINE:
      if (modelTest == MINE) { mParam = 5;} 
      
      Pgive = ((float) 1/(1 + powf(2.7182, (float)(kParam*(mParam*(tCache-tTask)/(estPart)+oParam))))); //without 5, it would never give up
      p = ((float)rand())/RAND_MAX;
      //printf("\n Robot %s PGiveUp is %.2f so she got %.2f for timeCache %d and tTask %d", robotName, Pgive, p, tCache, tTask); 
      break;
  } 
  
  if (cache == GIVEUP_LEVY) {
    writeDecision(Pgive, p, GIVEUP_LEVY);
  } else {
    writeDecision(Pgive, p, GIVEUP_CACHE);
  }
//  printf("\n Robot %s PGiveUp is %.2f so she got %.2f for timeCache %d and tTask %d when estimated is %d", robotName, Pgive, p, tCache, tTask, estPart); 

  int result = Pgive > p;
  if (result){
    printf("\n %s give up", robotName);
  } else {
    printf("\n %s will continue", robotName);
  }
  speaking(-2, -2, -2);
  wb_robot_step(32);  // to update global values
  return result;
}


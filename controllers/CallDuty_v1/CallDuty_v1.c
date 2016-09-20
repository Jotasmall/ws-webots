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

#define TIME_STEP 64

//Model -1 is for experiments, 0 is 2011, 1 is mine, 2 is UCB, 3 is Greedy 
#define ESSAY 0
#define RANDOMLY 1
#define NEVER 2
#define MODEL 3
#define GREEDY 4
// for different models
int modelTest = NEVER;

int flagFiles = 1;
int flagFilesFSM = 1;
int flagFilesEST = 1;
int flagFilesLIFE = 1;
int flagFilesDM = 1;
int flagFilesPER = 1;
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
// Flags of control
int flagMasterRecruiting = 0;   //1 RANDOMLY, -1 never, 0 whatever
// Other flags
int flagReady = 0;              //to know when she ended a travel
int flagLoad = 0;               //to know if she has a load or not
int flagRobot = 0;              //to know if there is a robot in front
int flagPrint1 = 1;
int flagTravel = 0;             //to know if a robot is partitioning or not
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
int estPickS = 0;
int estDropN = 0;
int estTravelGrey = 0;
int estTravelRed = 0;
int estTravelBlue = 0;
int lastImage = 0, timeImage = 0;
int timeMeasured = 0;
int timeListened = 0;
// Only for UCB algorithm
#define NB_REGIONS 3
int nPick[3] = {1, 1, 1};
int nDrop[3] = {1, 1 ,1};
// Main core FSM
#define GO2IT 1
#define LEVY 2
#define LOST 3
#define STOP 4
#define STOP_LEVY 5
// UML states
#define PICK_SOURCE 100
#define DROP_NEST 103
#define TRAVEL2GREY 104
#define TRAVEL2BLUE 105
#define TRAVEL2RED 106
#define NONE 555
#define EXPERIMENT 666
#define WAITING 777
#define TALK_NEST 999
int stateUML;
int statePrevious;
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
void resetDisplay();
void createDir(int option, int dirBuild);
void createFiles();
void initEstimations();
// Miscellaneous functions
int readSensors(int print);
void pickingIndication(int on);
int waiting(int n);
double angle(double x, double z);
// Image-depending functions
int whereIam(int avoiding);
int find_middle(int wrongLine, int colorLine);
int check4Robot();
int waiting_color(int foreground);
int cont_height_figure(int indexP);
int compareColorPixel(int pixelX, int pixelY, int foreground);
int detectImage(int foreground, int shape, int numImage, int *numberComponents);
int whatIsee(float Eccentricity, float Extent, int squareWidth, int middleAxisH, int middleAxisV, int numImage);
int doubleCheck();
// Movement functions
void avoidance();
int followingLine(int colorLine);
void turnSteps(int steps);
int run(int steps);
void forward(int steps);
int levyFlight();
int speedAdjustment(int index, int delta);
int hitWall(int front);
int hitLandmark(); 
int whereArrive();
int enterTam(); 
int detectTam();
int setRobotPosition(int colorLine);
int doorEntrance(int steps);
// Reaching targets
int going2Region(int colorLine, int colorDestination);
int going2it(int index); 
// Time and performance functions
void cronometer(int task, int cache);
void countObjects();
void updateEstimations(int task, int value, int cache);
void updateBitacora(int codeTask, int estimations, int cache);
void writeDecision(float boundP, float realP, int mechanism);
// Communication functions
int speaking(int toWhom, int codeTask, int time, int cache);//checked
int listening();                                //checked
// Model functions
int computeTraveling(int levy);

char dirPath[] = "//dir-dd-hh-mm";
time_t rawtime;
struct tm * timeinfo;

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
  printf("\n Robot %s is ready to begin", robotName);
  
  executeUML();
  wb_robot_cleanup();
  return 0;
}

void init_variables(){
  switch(modelTest){
    case ESSAY:
      stateUML = TRAVEL2GREY;
      color = CYAN;
      figura = BOX;
      break;
    case NEVER:
      flagMasterRecruiting = -1;
      stateUML = PICK_SOURCE;
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
    switch(stateUML){
      case EXPERIMENT:
        printf("\n %d is on floorColor %d", botNumber, floorColor);
      break;
      case PICK_SOURCE:
        printf("\n state PICK_SOURCE");
        printf("\n");
        stateUML = moduleUML(RED, BOX, PICKING, DROP_NEST, travelDestination, 0);
        printf("\n pass 2 state %d", stateUML); 
        printf("\n");
        break;
      case DROP_NEST:
        printf("\n state DROP_NEST");
        printf("\n");
        stateUML = moduleUML(MAGENTA, BOX, DROPPING, PICK_SOURCE, travelDestination, 1);
        printf("\n pass 2 state %d", stateUML);
        printf("\n");
      break;
      case TRAVEL2GREY:
        printf("\n state TRAVEL2NEST");
        printf("\n");
        stateUML = moduleTravel();
        printf("\n pass 2 state %d", stateUML);
        printf("\n");
      break;      
      case TRAVEL2BLUE:
        printf("\n state TRAVEL2SOURCE");
        printf("\n");
        stateUML = moduleTravel();
        printf("\n pass 2 state %d", stateUML);
        printf("\n");
      break;
      case TRAVEL2RED:
        printf("\n state TRAVEL2SOURCE");
        printf("\n");
        stateUML = moduleTravel();
        printf("\n pass 2 state %d", stateUML);
        printf("\n");
      break;
      default:
        printf("\n Big failure on UML machine");
    }
  }  
}

int moduleUML(int foreground, int shape, int pick_or_drop, int stateRemain, int stateTravel, int flag) {

  color = foreground;
  if ((floorColor == RED) && (stateUML == PICK_SOURCE)) { color = BLUE;}
  figura = shape;
  statePrevious = stateUML;
  int auxShape = 0; //shapeSeen when working with different shapes
  
  if (pick_or_drop == PICKING) { pickingIndication(1);}
  else { wb_led_set(RobotLed[8], 1);}
  
  output = moduleFSM();
  //updateEstimations(IMAGE, timeImage, 0);
  int nextState = stateRemain;
  int flagWait = 1;
  flagLoad = flag; 
  if (output == STOP) {
    while(flagWait) {
      flagWait = waiting_color(color);	
      if (flagWait == 0) {
        // Count only works with UCB
        countObjects();
        forward(-120);     
        turnSteps(TURN_CACHE);
        //waiting(100);  
        updateEstimations(stateUML, timeMeasured, auxShape);
        
        flagLoad = !flag;
        if (pick_or_drop == PICKING) { pickingIndication(0);}
        else { wb_led_set(RobotLed[8], 0);}	
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
      printf("\n %s would like to travel towards %d", robotName, travelDestination);
      //-- nextState = stateTravel;
    }	  
  } 
  return nextState;
}

int moduleTravel(){
  int auxUML = stateUML; 
  color = CYAN;
  figura = BOX;
  output = moduleFSM();
  //updateEstimations(IMAGE, timeImage, 0);
  if (output == STOP){  
    if (stateUML  == TRAVEL2GREY) {
      if (floorColor == BLUE) {
        flagReady = going2Region(RED, GREY);
      } else {
        flagReady = going2Region(BLUE, GREY);
      }  
    } else if (stateUML == TRAVEL2BLUE)  {
      if (floorColor == GREY){
        flagReady = going2Region(RED, BLUE);
      } else {
        flagReady = going2Region(BLUE, BLUE);
      }
    } else if (stateUML == TRAVEL2RED) {
      if (floorColor == GREY){
        flagReady = going2Region(BLUE, RED);
      } else {
        flagReady = going2Region(RED, RED); 
      }
    } 
    if (flagReady) {  
      //auxUML = PICK_SOURCE; //JUAN
      if (floorColor == RED) {
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
      }
      updateEstimations(stateUML, timeMeasured, 0);
    }    
  } else {
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
    } /*else {
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
        index = levyFlight();
        if (index == -1) {
          contLevy++;
          printf("\n %s is thinking about her decision", robotName);
          printf("\n");
          switch(stateUML){
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
                updateBitacora(STOP_LEVY, FSM, 0);
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
          updateBitacora(LEVY, FSM, 0);
          stateFSM = GO2IT; 
        }
        if (stateUML == PICK_SOURCE) {
          wb_led_set(RobotLed[0], 1);
        } else if (stateUML  == DROP_NEST){
          pickingIndication(1);
        } else {
          pickingIndication(0);
          wb_led_set(RobotLed[0], 0);
        } 
        wb_robot_step(32);
        break;  
      case GO2IT:
        //printf("\n %s found something and goes to get it", robotName);
        //printf("\n");
        flagProximity = going2it(index);
        if (flagProximity){
          flagProximity = 0;
          flagSureSeen = 0;
          waiting(1);
          if ((stateUML == TRAVEL2BLUE) || (stateUML == TRAVEL2RED) || (stateUML == TRAVEL2GREY)) {
            flagInside = hitWall(0);
          } else {
            flagInside = enterTam();
          }
          if (flagInside == 1) { //sucessful enter TAM or hit wall
            stateFSM = STOP;
          } else {
            forward(-50);
            stateFSM = LOST;
          }
          updateBitacora(GO2IT, FSM, 0);
        } else { //it is yet far
          newIndex = detectImage(color, figura, 0, &nComp);
          if (newIndex == -1) {
            flagRobot = check4Robot();
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
            if (shapeSeen == oldShape) { contViewShape++;}
            if ((contViewShape > 10) && (nComp == 1)) {
              contViewShape = 0;
              flagSureSeen = 1;
              printf("\n %s is sure that saw shape %d", robotName, oldShape);
              printf("\n");
              //speaking(M2ROBOT, shapeSeen, shapeSeen, shapeSeen); //to make a rendesvouz
            }           
          }
        }  
      break;
    case LOST:
      printf("\n %s is lost", robotName);
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
  printf("\n %s was born in region %d", wb_robot_get_name(), floorColor);
  printf("\n");
  if (flagFiles) { createFiles();}  
  initEstimations();
  strcpy(robotName, wb_robot_get_name());
  // Random seed by the number of the robot
  botNumber = atoi(&robotName[6]);
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
  // printf("\n Calibration offset ");
  for (i=0; i<NB_DIST_SENS; i++){
    ps_offset[i] /= CALIBRATE-1;
    // printf("%d ", ps_offset[i]);
  }
  speaking(M2NEST, ROBOT_ARRIVING, 0, 0);
}

void resetDisplay(){ //ok
  wb_display_set_color(displayExtra, HEXBLACK);
  wb_display_draw_rectangle(displayExtra, 0, 0, width, height);
}

void createDir(int option, int dirBuild){
  switch(option){
    case FSM:
      sprintf(fileRobot, "%s-FSM", dirPath);
      //printf("\n fileRobot %s", fileRobot);
      //printf("\n"); 
      break;
    case ESTIMATIONS:
      sprintf(fileRobot, "%s-EST", dirPath); 
      //printf("\n fileRobot %s", fileRobot);
      //printf("\n"); 
      break;
    case LIFE:
      sprintf(fileRobot, "%s-REC", dirPath);
      //printf("\n fileRobot %s", fileRobot);
      //printf("\n"); 
      break;
    case DECISIONS:
      sprintf(fileRobot, "%s-DM", dirPath); 
      //printf("\n fileRobot %s", fileRobot);
      //printf("\n"); 
      break;
    case PERFORMANCE:
      sprintf(fileRobot, "%s-OBJ", dirPath);
      //printf("\n fileRobot %s", fileRobot);
      //printf("\n"); 
      break;
   }
   
   //if (dir){ mkdir(fileRobot,0700);} 
   if (dirBuild){ CreateDirectory(fileRobot, NULL);} 
    
   strcat(fileRobot, "/");
   strcat(fileRobot, wb_robot_get_name()); 
   //printf("\n fileRobot %s", fileRobot);
   //printf("\n"); 

   switch(option){
     case FSM:
       strcat(fileRobot, "-FSM.txt");
       //printf("\n fileRobot %s", fileRobot);
       //printf("\n"); 
       break;       
     case ESTIMATIONS:
       strcat(fileRobot, "-EST.txt");
       //printf("\n fileRobot %s", fileRobot);
       //printf("\n");
       break;       
     case LIFE:
       strcat(fileRobot, "-REC.txt");
       //printf("\n fileRobot %s", fileRobot);
       //printf("\n"); 
       break;
     case DECISIONS:
       strcat(fileRobot, "-DM.txt"); 
       //printf("\n fileRobot %s", fileRobot);
       //printf("\n"); 
       break;
     case PERFORMANCE:
       strcat(fileRobot, "-OBJ.txt"); 
       //printf("\n fileRobot %s", fileRobot);
       //printf("\n"); 
       break;
  }
  if (dirBuild){
    printf("\n %s is accessing to %s", robotName, fileRobot);
  }  
}

void createFiles(){ //ok
  // File for each cycle of FSM
  createDir(FSM, 1); 
  FILE *ffsm = fopen(fileRobot,"w");
  if (ffsm == NULL) {
    printf("Error opening file bot fsm \n");
    printf("\n");
    exit(1);
  }
  fprintf(ffsm, "StateUML, Suceess/Fail, timeMeasured \n");
  fclose(ffsm);
  // File for evolution of estimations
  createDir(ESTIMATIONS, 1); 
  FILE *fest = fopen(fileRobot, "w");
  if (fest == NULL) {
    printf("Error opening estimation file \n");
    printf("\n");
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
    printf("\n");
    exit(1);
  }
  fprintf(flife,"what, time \n");
  fclose(flife);
  // File for decisions
  createDir(DECISIONS, 1); 
  FILE *fpart = fopen(fileRobot, "w");
  if (fpart == NULL) {
    printf("Error opening partition file \n");
    printf("\n");
    exit(1);
  }
  fprintf(fpart, "Case, decision \n");
  fclose(fpart);
  // File for performance 
  createDir(PERFORMANCE, 1);
  FILE *fper = fopen(fileRobot, "w");
  if (fper == NULL) {
    printf("Error opening performance file \n");
    printf("\n");
    exit(1);
  }
  fprintf(fper, "PICK, DROP, HARVEST, STORE\n");
  fclose(fper);
}

void initEstimations(){ //ok-
  // rand() % (max_n - min_n + 1) + min_n;
  estPickS = rand() % (500-200+1)+200; 
  estDropN = rand() % (500-200+1)+200;
  estTravelGrey = rand() % (1000-350+1)+350;
  estTravelRed = rand() % (1000-350+1)+350;
  estTravelBlue = rand() % (1000-350+1)+350;
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
        printf("\n");
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
  int groundDetected = GREY;
  // cronometer(IMAGE, 0); //This is a fast operation
  
  if (cont_height_figure(-20) > 104 ) { 
    groundDetected = BLUE;
  } else if (cont_height_figure(-21) > 104 ) {
    groundDetected = RED;
  } else if (cont_height_figure(-22) > 104) {
    groundDetected = GREY;
  }
  if ((avoiding) && (groundDetected != floorColor)) {
    float p = ((float)rand())/RAND_MAX; //-- JUAN EDITED
    if (p>0.5) {p = 1;} else { p = -1;} //-- JUAN EDITED
    turnSteps((int) p*TURN_CACHE/2);    //-- JUAN EDITED
    run(7);//15
    printf("\n Missing my region %s", robotName);
    printf("\n");
  }    
  return groundDetected;
} 

int check4Robot(){//ok-
  int nComp, sizeRobot = 0;

  sizeRobot = detectImage(ROBOT_COLOR, ROBOT, 0, &nComp);
  if (((sizeRobot > 9) && (nComp > 1)) || ((sizeRobot > 4) && (nComp > 3))) {//4 3
    printf("\n %s sees a robot of height %d components %d", robotName, sizeRobot, nComp);
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
  int countArriving = 0;
  countArriving = cont_height_figure(102);
  /* if (flagPrint1) {
    if (count > countArriving) {
      printf("\n Intensity %d half line", count);
    } else {
      printf("\n Intensity %d half line", countArriving);  
    }
  
    flagPrint1 = 0;
  } */
  if ((count > 26) || (countArriving > 26)) {
    if (stateUML == PICK_SOURCE) {
      cronometer(WAITING, 0); //shapeSeen); //when using different shapes
    } 
    cronometer(-1, 0);
    return 1; //keep waiting
  }    
  //printf("\n Intensity gets down");
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
    case -22:
      beginY = height - 5;
      foreground = GREY; break;
    case -21: // checking red-nest ground color
      beginY = height - 5;
      foreground = RED; break;
    case -20: // checking blue-source ground color
      beginY = height - 5;
      foreground = BLUE; break;
    case -11: // looking for landmark
      foreground = MAGENTA; // nest TAM
      break; 
    case -10: // On levy avoid colors 18 
      foreground = MAGENTA;
      if (stateUML == DROP_NEST) { 
        foreground = RED; // source TAM
        if (floorColor == RED) { foreground = BLUE;} // source TAM
      } break; 
    case 100: // checking tam_wall color
      foreground = TAM_WALL; break;
    case 101: // waiting on TAM 
      if ((stateUML == TRAVEL2BLUE) || (stateUML == TRAVEL2RED) || (stateUML == TRAVEL2GREY)){
        foreground = CYAN;
      } else {
        if (foreground != BLACK) { foreground = WHITE;}
      } break;
    case 102: // checking for sources 
      if (stateUML == PICK_SOURCE) {
        foreground = RED;
        if (floorColor == RED) {
          foreground = BLUE;
        }
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
  // printf("\n count value %d index %d", maxCount, i);
  return maxCount;
}

int compareColorPixel(int pixelX, int pixelY, int foreground){ //ok-
  int auxColor = 0;
  int low = 38, lowdark = 34, high = 200;  //38 - 30 - 200
  int pixelR = wb_camera_image_get_red(image, width, pixelX, pixelY);
  int pixelG = wb_camera_image_get_green(image, width, pixelX, pixelY);
  int pixelB = wb_camera_image_get_blue(image, width, pixelX, pixelY);
  if ((foreground == CYAN) && (floorColor == GREY)) { foreground = WHITE;}
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
      auxColor = (pixelG > LOW_THRES) && (pixelB > LOW_THRES) && (pixelR < 20);
      break;
    case MAGENTA:  // red+blue
      auxColor = (pixelR > COMP_COLOR) && (pixelB > COMP_COLOR) && (pixelG < LOW_THRES);
      break;
    case BLACK:
      auxColor = (pixelR < BLACK_THRES) && (pixelG < BLACK_THRES) && (pixelB < BLACK_THRES);
      break;
    case ROBOT_COLOR:  
      auxColor = (pixelR < low) && (pixelG < low) && (pixelB < low); 
      auxColor = auxColor && ((pixelR > lowdark) && (pixelG > lowdark) && (pixelB > lowdark));
      auxColor = auxColor || ((pixelR > ROBOT_THRES) && (pixelR < high) && (pixelG > ROBOT_THRES) && (pixelG < high) && (pixelB > ROBOT_THRES) &&  (pixelB < high));
      break;
    case GREY:
      auxColor = (pixelR < COMP_COLOR) && (pixelG < COMP_COLOR) && (pixelB < COMP_COLOR);
      auxColor = auxColor && ((pixelR > BLACK_THRES) && (pixelG > BLACK_THRES) && (pixelB > BLACK_THRES));	  
      break;	
    case WHITE:
      auxColor = (pixelR > COLOR_THRES) && (pixelB > COLOR_THRES) && (pixelG > COLOR_THRES);  
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
      printf("\n %s really close and sure it is not a robot, go for the center", robotName);
      printf("\n");
      return 100;
    }
//    if (((area > 10) && (foreground != CYAN)) || ((foreground == CYAN) && (area > 15))) { 
    if (((area > 10) && (foreground != CYAN)) || ((foreground == CYAN) && (area > 25))) { 
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
      if ((squareWidth >= 4) && (squareHeight >= 4)) {
        //1 Triangle, 2 Box, 3 Circle, 4 Nothing, 0 ReallyNothing, 5 All, 6 Robot
         newShapeSeen = whatIsee(eccentricity, extent, squareWidth, middleAxisH, middleAxisV, numImage);
         if (shape == ROBOT){
           if (newShapeSeen == ROBOT) {
             if ((x > 23) && (x < 29) && (areaSquare > 600)) { waiting(15);} 
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
    printf("\n False alarm %d - %s continue searching", index, robotName);
    printf("\n");
    return -1;
  } 
  printf("\n Shape %d watched on 2check", figura);
  printf("\n");
  return index; 
}

void avoidance(){ //ok
  int sense = 1;
  waiting(1);
  readSensors(0);
  if ((ps_value[7] + ps_value[6]) < (ps_value[0] + ps_value[1])) {
    //printf("\n Obstacle in right-side");
    sense = -1;
  } /*else {
    printf("\n Obstacle in left-side");
  }*/ //-- JUAN EDIT
  turnSteps(TURN_90*sense);
  forward(18); //15
  turnSteps((TURN_M90-3)*sense);
}

int followingLine(int colorLine){//ok-
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
      printf("\n");
    } else {
      image = wb_camera_get_image(cam);
      // cronometer(IMAGE, 0); // Disable because it is only one row
      delta = find_middle(0, colorLine);
      if ((delta > -1) && (delta < 100)) {
        delta = delta - width/2;
        speed[LEFT] = 220 - K_TURN*abs(delta);
        speed[RIGHT] = 220 - K_TURN*abs(delta);
        
        wb_differential_wheels_set_speed(speed[LEFT]+K_TURN*delta,speed[RIGHT]-K_TURN*delta);
        wb_robot_step(TIME_STEP);
        cronometer(-1, 0); 
      } else {
        flagRobot = check4Robot();
        if (flagRobot) {
          waiting(30);//20
        } else {
          printf("\n %s is lost from the line", robotName);
          printf("\n");
          return -1; // End of travel
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
    if (flagLoad){ //reducing speed when cargo
      speed[LEFT]*=.6;
      speed[RIGHT]*=.6;
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
  if (flagLoad){ //reducing speed when cargo
    speed[LEFT]*=.6;
    speed[RIGHT]*=.6;
  }
  wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);
  while (k < steps) {
    k++;
    wb_robot_step(TIME_STEP); 
    cronometer(-1, 0); // -1 for let it find the task
  }
  waiting(1);  
}

int levyFlight(){

  int index = -1;
  int nComp;
  // rand() % (max_n - min_n + 1) + min_n;
  int r = rand()%(54-27+1)+27; 
  // Robot needs about 7 steps at 200 speed to have a new image
  //printf("\n Robot %s turning random %d", robotName, r); //-- JUAN EDIT
  //printf("\n"); //-- JUAN EDIT
  while (r > 0) {
    turnSteps(3); // Blind turn
    r -= 3;
    
    image = wb_camera_get_image(cam);
    wb_robot_step(TIME_STEP);
    // cronometer(IMAGE, 0) //It is only a line
    //listening();
    
    if (cont_height_figure(-10) > 15) {//18
      printf("\n Backward invading useful region on turn");
      printf("\n");
      forward(-30); //70
    }
    if ((color == CYAN) && (cont_height_figure(-11) > 22)) { //25
      printf("\n Backward invading on turn %d",cont_height_figure(-11));
      printf("\n");
      forward(-30); //70
    }
    index = detectImage(color, figura, 0, &nComp); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
        return doubleCheck();
      }
      printf("\n Shape watched on levy - Levy Aborted %d", index);
      printf("\n");
      return index;  
    } 
    whereIam(1);
  } 
  r = rand()%(100-40)+41; // walk forward between 100 to 40 steps
  //printf("\n %s Walking forward %d", robotName, r); //-- JUAN EDIT
  //printf("\n"); //-- JUAN EDIT
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
      printf("\n");
      forward(-20); //30
      turnSteps(TURN_CACHE);
    }   
    if ((color == CYAN) && (cont_height_figure(-11) > 22)) {
      printf("\n Backward invading on walk %d", cont_height_figure(-11));
      printf("\n");
      forward(-20); //70
      turnSteps(TURN_CACHE/2);
    } 
    index = detectImage(color, figura, 0, &nComp); // Open her eyes
    if (index != -1) {
      if (index == 100){ //double-check mechanism
        return doubleCheck(); 
      }
      printf("\n Color watched on levy - Levy Aborted %d", index);
      printf("\n");
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
    resetDisplay();
    flagRobot = check4Robot();

    if (color == CYAN){   
      if (flagRobot) { 
        forward(-15);
        // printf("\n %s found a robot when going to Landmark", robotName); //-- JUAN EDIT
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      detectImage(CYAN, BOX, 255, &iter);
      //printf("\n Difference between A-B %d", pointA-pointB);
      iter = pointA - pointB;
      if (iter > 2) { hitWall(5);}
      else if (iter < -2) { hitWall(-5);}
      else { hitWall(0);}
   
      printf("\n %s reached cyan landmark!", robotName);
      printf("\n");
      waiting(1);      
      return 1;
    } else {
      //printf("\n Robot %s is near but...", robotName);
      if (flagRobot) {
        //printf("\n %s found another robot there", robotName);
        forward(-5);       
        waiting(10);
        flagRobot = 0;
        return 0;
      }
      forward(20);
      hitWall(1);
      forward(-7);
      delta = ps_value[0] + ps_value[1] - ps_value[7] - ps_value[6];
      if (delta > THRESHOLD_DIST) { turnSteps(3);} // almost 10°
      else if (delta < THRESHOLD_DIST) { turnSteps(-3);}
      switch(color){
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
      }
      waiting(1);
      return 1;
    }   
  } else { //before being close enough
    // printf("\n %s saw shape with height %d", robotName, count);
    if (readSensors(0) && ((ps_value[0] > THRESHOLD_DIST) || (ps_value[1] > THRESHOLD_DIST) 
    || (ps_value[7] > THRESHOLD_DIST) || (ps_value[6] > THRESHOLD_DIST))) { // 1 for obstacle
      printf("\n %s found obstacle on the way", robotName);
      printf("\n");
      avoidance();
    }
    
    flagRobot = check4Robot();
    if (flagRobot) {
      //printf("\n I %s found another robot there", robotName);
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
    readSensors(0);
    //printf("\n sensors 0 %d, 1 %d, 6 %d, 7 %d", ps_value[0], ps_value[1], ps_value[6], ps_value[7]);
    if ((front == 0) || (abs(front) == 5)) {
      question = (ps_value[0] > hit_thres) || (ps_value[7] > hit_thres) || (ps_value[6] > hit_thres) || (ps_value[1] > hit_thres);
    } else {
      question = (ps_value[0] > hit_thres) || (ps_value[7] > hit_thres);
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
      waiting(2);
      readSensors(0);
      if (question || (ps_value[6] > hit_thres) || (ps_value[1] > hit_thres)){
        forward(5);
        flag = 0;
      }
    }    
    wb_differential_wheels_set_speed(speed[LEFT], speed[RIGHT]);
    wb_robot_step(TIME_STEP);
    cronometer(-1, 0); 
  }
  waiting(1);
  printf("\n Robot %s hit against a wall...", robotName);
  printf("\n");
  return 1;
}

int detectTam(){ //ok
  image  = wb_camera_get_image(cam);
  wb_robot_step(TIME_STEP);
  // cronometer(IMAGE, 0) //It is only a line
  
  waiting(1); 
  if ((cont_height_figure(101) < 35) && (cont_height_figure(102) < 35)){ //30 checking wall tam
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
  //1 rigth and -1 left
  int dir = 0;
  if (ps_value[1] + ps_value[0] > ps_value[7] + ps_value[6]) {
    dir = 1;
  } else {
    dir = -1;
  }
  flag1stCheck = detectTam(); 
  if (!flag1stCheck) { 
    turnSteps(-26*dir);
    waiting(1);
    hitWall(1); 
    forward(-6);//8
  }  
  if ((ps_value[0] > THRESHOLD_DIST) && (ps_value[7] > THRESHOLD_DIST) && (ps_value[6] < THRESHOLD_DIST) && (ps_value[1] < THRESHOLD_DIST)) {
    //printf("\n Everything is fine!!");
    //printf("\n");
    return 1; 
  } else {    
    flag1stCheck = detectTam();
    if (!flag1stCheck) {
      printf("\n %s am not inside TAM correctly", robotName);
      printf("\n");
      waiting(10);
      return 0;
    } else {
      printf("\n %s entered successfully square",robotName);
      waiting(2);
      return 1;  
    }  
  }
  return -1;    
}

int find_middle(int wrongLine, int colorLine){ //ok 
  int i;
  int aux, index1 = -1, index2 = -1;
  int foreground = colorLine;
  if (wrongLine) { 
    if (foreground == BLUE) {
      foreground = RED;
    } else {
      foreground = BLUE;
    }
  }
  // new world
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
  aux = (index2-index1)/2+index1;
  if (wrongLine) {
    aux = 100;
    printf("\n %s had found a wrong line color", robotName);
    printf("\n");
  }
  return aux;    
}

int whereArrive(){
    // To add randomness in the entrance 
    waiting(2);
    // Verify if not robot is close
    if ((readSensors(0) == 0) && (check4Robot() == 0)) {
      floorColor = whereIam(0);
      printf("\n %s arrived into a land of color %d", robotName, floorColor);
      printf("\n");
      speaking(M2NEST, ROBOT_ARRIVING, 0, 0);
    } else {
      waiting(10);
      printf("\n Waiting to have a clear ground");
      printf("\n");
      return whereArrive();
    }
    return 1;
}

int doorEntrance(int steps){
  printf("\n %s is entering a new region", robotName);
  printf("\n");
  forward(10);
  turnSteps(TURN_M90);
  readSensors(0);
  if ((ps_value[0] > THRESHOLD_DIST) || (ps_value[7]> THRESHOLD_DIST)) {
    printf("\n %s wrong turn", robotName);
    printf("\n");
    return 0;
  }
  forward(steps);
  speaking(M2NEST, ROBOT_LEAVING, 0, 0); // To indicate home-nest 
  speaking(-1, ROBOT_LEAVING, 0, 0); // To indicate friends 
  waiting(1);
  turnSteps(-10);
  return 1;
}

int setRobotPosition(int colorLine){
  int flagRobot = check4Robot();
  // She saw a robot or not Cyan color in front
  if (flagRobot) {
    //printf("\n False Cyan landmark, %s", robotName);
    forward(-20);
    return 0;
  }
  readSensors(0);
  // hit by sensor 1, turn almost 20 degrees
  if ((ps_value[0] > THRESHOLD_DIST) || (ps_value[1] > THRESHOLD_DIST)) { turnSteps(10);} 
  speed[LEFT] = 100;
  speed[RIGHT] = -100;
  int notReady = 1;
  int wrongDoor = 0;
  int counter = 0, aux;
  printf("\n %s is looking for line of color %d", robotName, colorLine);
  printf("\n");
  while(notReady) { 
    wb_differential_wheels_set_speed(speed[LEFT],speed[RIGHT]);      
    readSensors(0);
    counter++;
    if (ps_value[5]> 300) {
      notReady = find_middle(0, colorLine) < 0; // returns the index -1 if not
      wrongDoor = find_middle(1, colorLine) > 0; // return 100 if it found it
      flagRobot = check4Robot();
      aux = counter;
      while (flagRobot) {
        flagRobot = check4Robot();
        printf("\n %s waiting for another robot to leave", robotName);
        printf("\n");
        waiting(20);
        counter++;
        if (counter > 90) {
          printf("\n %s wait for an entire turn and no free way", robotName);
          printf("\n");
          return -1;
        } else if (flagRobot == 0) {
          counter = aux;
          printf("\n %s has a clear way", robotName);
          printf("\n");
        }
      }
      if (wrongDoor) {
        return -2;
      } 
    } else {
      flagRobot = check4Robot();
      if (flagRobot) {
        printf("\n %s find another robot here",robotName);
        printf("\n");
        return -1;
      }
      if (counter > 60) {
        printf("\n %s gave a entire turn and no line", robotName);
        printf("\n");
        return -1;
      }
    }
    cronometer(-1, 0); 
  } 
  return 0;
}

int going2Region(int colorLine, int colorDestination){ //ok
  int endTask = 0;
  resetDisplay();
  printf("\n %s getting in position destination %d by line of color %d", robotName, colorDestination, colorLine);
  printf("\n");
  endTask = setRobotPosition(colorLine);
  if (endTask == -1) { // found no line
    while(!run(50)); //60
    return 0;
  } else if (endTask == -2) { //found another color
    turnSteps(15);
    while(!run(60));
    return 0;
  }
  endTask = followingLine(colorLine);
  if (endTask == -1) { //End of travel
    //printf("\n Robot %s going inside", robotName);
    //printf("\n");
    endTask = doorEntrance(60); 
    if (endTask == 0) {
      forward(-20);
      return 0;
    }
    whereArrive();
    if (floorColor == colorDestination) {
      printf("\n Excellent entrance, %s is on desired region", robotName);
      printf("\n");
      return 1;
    } else {
      printf("\n Something went wrong, please %s recheck color destination %d", robotName, colorDestination);
      printf("\n");
      return 0;
    } 
  }
  return 1000;  
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
    printf("\n");
    return 0;
  }
  return speedAdjustment(index,delta);  
}

void cronometer(int task, int cache){//ok-
  
  if (task == IMAGE) { 
    timeImage++;
  } else {  
    timeMeasured++;
  }
  //printf("\n %s is listening", robotName);
  //printf("\n");
  listening(); //--JUAN EDIT FILES
  if (flagFilesLIFE) {
    createDir(LIFE, 0);
    //printf("\n %s is updating in %s", robotName, fileRobot);
    //printf("\n");
    FILE *flife = fopen(fileRobot,"a+");
    if (task == IMAGE) { 
      fprintf(flife, "image, %d \n", timeImage);
    } else {  
     fprintf(flife, "state %d, %d\n", stateUML, timeMeasured);
    }
    fclose(flife); //-- JUAN EDIT FILES 
  } 
}

void countObjects(){
  switch(stateUML){
    case DROP_NEST:
      nDrop[floorColor]++; break;
    case PICK_SOURCE:
      nPick[floorColor]++; break;
  }
  if (flagFilesPER) {
    createDir(PERFORMANCE, 0);
    printf("\n %s is counting objects in %s", robotName, fileRobot);
    printf("\n");
    FILE *fper = fopen(fileRobot, "a+");
    int i;
    for (i=0; i<NB_REGIONS; i++){ 
      fprintf(fper, "%d, %d, ", nPick[i], nDrop[i]);
    }
    fprintf(fper, "\n");  
    fclose(fper); //-- JUAN EDIT FILES
  }  
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
    case PICK_SOURCE:
      estPickS = (estPickS * (100 - alpha) + value * alpha + (estPickS - value) * beta) / 100;
      break;
    case DROP_NEST:
      estDropN = (estDropN * (100 - alpha) + value * alpha + (estDropN - value) * beta ) / 100;
      break;
    case TRAVEL2GREY:
      estTravelGrey = (estTravelGrey  * (100 - alpha) + value * alpha + (estTravelGrey  - value) * beta ) / 100;
      break;
    case TRAVEL2BLUE:
      estTravelBlue = (estTravelBlue  * (100 - alpha) + value * alpha + (estTravelBlue  - value) * beta ) / 100;
      break;
    case TRAVEL2RED:
      estTravelRed = (estTravelRed  * (100 - alpha) + value * alpha + (estTravelRed  - value) * beta ) / 100;
      break;
  }
  cache = 0; // comment when working with shapes
  updateBitacora(0, ESTIMATIONS, cache);
  if (codeTask != IMAGE) { 
    updateBitacora(codeTask, FSM, cache); 
    if (flagListened) {
      flagListened = 0;
    } else {
      printf("\n Everybody listen, I am %s, my %d cost me %d", robotName, codeTask, value);
      printf("\n");
      speaking(M2ROBOT, codeTask, value, cache);
    }  
  } 
  lastImage = timeImage;
  timeImage = 0;  
  timeMeasured = 0;
  timeListened = 0;
  wb_robot_step(32);
}

void updateBitacora(int codeTask, int estimations, int cache){ //ok-
  if (estimations == ESTIMATIONS) { 
    if (flagFilesEST) { 
      createDir(ESTIMATIONS, 0); 
      //printf("\n %s is estimating times in %s", robotName, fileRobot);
      //printf("\n");
      FILE *fbot = fopen(fileRobot, "a+");
      if (fbot==NULL) {
        printf("Error opening file of estimations bot\n");
        printf("\n");
        exit(1);
      }
      
      if (flagListened == 1) {
        fprintf(fbot, "Update after heard for cache %d, %d, %d, %d, %d, %d \n", 
                   estPickS, estDropN, estTravelGrey, estTravelBlue, estTravelRed, lastImage);
      } else {
        fprintf(fbot, "Update after finish for cache %d, %d, %d, %d, %d, %d \n", 
                   estPickS, estDropN, estTravelGrey, estTravelBlue, estTravelRed, lastImage);
      }             
      fclose(fbot); //-- JUAN EDIT FILES
      }
  } else {    
    if (flagFilesFSM) {
      createDir(FSM, 0); 
      //printf("\n %s is on state machine times in %s", robotName, fileRobot);
      //printf("\n");
      FILE *fbot = fopen(fileRobot, "a+");    
      if (fbot==NULL) {
        printf("Error opening file bot\n");
        printf("\n");
        exit(1);
      }
      
      char stringState[] = "SEARCHING SOMETHING";
      int innerState = stateUML;
      if (flagListened) { innerState = codeTask;}
      
      switch(innerState){
        case PICK_SOURCE:
          sprintf(stringState, "PICK SOURCE"); break;
        case DROP_NEST:
          sprintf(stringState, "DROP NEST"); break;
        case TRAVEL2GREY:
          sprintf(stringState, "TRAVEL2GREY"); break;
        case TRAVEL2BLUE:
          sprintf(stringState, "TRAVEL2BLUE"); break;    
        case TRAVEL2RED:
          sprintf(stringState, "TRAVEL2RED"); break;
      }
      if (flagListened) {
        fprintf(fbot,"Listened %s, 0, %d\n", stringState, timeListened);
      } else {
        fprintf(fbot,"Executed %s, %d, %d\n", stringState, codeTask, timeMeasured);
      }
      fclose(fbot);  //-- JUAN EDIT FILES           
    }                
  } 
}

void writeDecision(float boundP, float realP, int mechanism){ //ok-
  if (flagFilesDM) {
    // File for decisions
    createDir(DECISIONS, 0);
    printf("\n %s is registering its decision in %s", robotName, fileRobot);
    printf("\n");	
    FILE *file = fopen(fileRobot, "a+");
    if (file==NULL) {
      printf("Error opening file of estimations bot\n");
      printf("\n");
      exit(1);
    }
  
    if (mechanism == TRAVELING_AGREE) {
      fprintf(file, "\n Partitioning %d, %.2f, %.2f", boundP>realP, boundP, realP);
    } else if (mechanism == TRAVELING_LEVY){
      fprintf(file, "\n Partitioning-Levy %d, %.2f, %.2f", boundP>realP, boundP, realP);
    } else if (mechanism == TRAVELING_CALL){
      fprintf(file, "\n Partitioning by hearing %d, 99, 99", flagTravel);
    }
    fclose(file); //-- JUAN EDIT FILES
  }
}

int speaking(int toWhom, int codeTask, int time, int cache){ //ok-
  if (flagCom == 0) { return 0;}

  char message[30];
  // wb_emitter_set_channel(emitter, WB_CHANNEL_BROADCAST);
  if (toWhom == M2ROBOT) {
    if (time == -1) { // reporting just to have the same number of lines
      sprintf(message, "U");
    } else if (toWhom == -1){ 
      sprintf(message, "R2R%dR%d",botNumber, ROBOT_LEAVING);
    } else {
      sprintf(message, "R2R%dC%dT%d",botNumber, codeTask, time);
    }
  } else if (toWhom == M2NEST) {
    if (time == -1) {
      printf("\n %s will update your estimation NEST %d", robotName, floorColor);
    } else {
      if (codeTask == ROBOT_LEAVING) {
        sprintf(message,"R2T%dT%dX%d",botNumber, ROBOT_LEAVING, floorColor);
        printf("\n %s is leaving NEST %d", robotName, floorColor);
        printf(" message %s", message);
        printf("\n");
      } else if (codeTask == ROBOT_ARRIVING) {
        sprintf(message,"R2T%dT%dX%d",botNumber, ROBOT_ARRIVING, floorColor);
        printf("\n %s is arriving into NEST %d", robotName, floorColor);
        printf(" message %s", message);        
        printf("\n");
      } else if (codeTask == ROBOT_UPDATING) {
        sprintf(message,"R2T%dT%dX%d",botNumber, ROBOT_UPDATING, estPickS + estDropN);
        printf("\n %s is arriving into NEST %d", robotName, floorColor);
        printf(" message %s", message);        
        printf("\n");
      }
    }
  }
  wb_emitter_send(emitter, message, strlen(message)+1);
  wb_robot_step(32);
  return 1;
}

int listening() { //ok-
  int i;
  while(wb_receiver_get_queue_length(receiver)>0){  
    const char *data = wb_receiver_get_data(receiver);
    if (data[0] == 'U') {
      //printf("\n %s will update its state of partitioning %d", robotName, flagTravel);
      writeDecision(0, 0, TRAVELING_CALL);
      wb_receiver_next_packet(receiver);
    } else if ((data[0] == 'T') && (data[2] == 'R')) {
      // "T2R0T77X9"
      int place = atoi(&data[3]);
      printf("\n %s has received %s message from %d", robotName, data, place);
      printf("\n");
      if (place == floorColor) {
        //-- printf("\n %s is listening its nest location %d", robotName, place);
        //-- printf("\n");
        int action = atoi(&data[5]);
        if (action == LEAVE) {
          //-- int destination = atoi(&data[8]); 
          //-- printf("\n %s is being requested to leave from %d to %d", robotName, place, destination);
          //-- printf("\n");
        } else if (action == botNumber){ 
          int newFriend = atoi(&data[10]);
          int flagNew = 1, pos = nRobots; 
          //-- printf("\n %s was introduced to %d", robotName, newFriend);
          //-- printf("\n");
          for (i = 0; i < nRobots; i++) { 
            if (newFriend == listFriends[i]) {
              flagNew = 0; // it already exists
              break;
            }
            if (listFriends[i] == 0) {
              if (pos > i) {
                pos = i; // to fill the empty spaces
              }
            }
          }
          if (flagNew) {
            listFriends[pos] = newFriend;
            printf("\n %s added to its friends %d", robotName, newFriend);
            printf("\n");
          }
        }    
      }
      wb_receiver_next_packet(receiver);
    } else if ((data[0] == 'R') && (data[2] == 'R')) { 
      /*  
         A standard message R2R0000C777T9999X3
         0000 = number of robot
         C777 = code of task
         T9999 = time of task
         X3 = type of cache found //still not used
      */
      int name = atoi(&data[3]);
      int codeReceived = atoi(&data[8]);
      for (i = 0; i < nRobots; i++) {
        if (name == listFriends[i]) {
          if (codeReceived == ROBOT_LEAVING) {
            printf("\n %s bye bye %d", robotName, name);
            printf("\n");
            listFriends[i] = 0;
          } else {
            // proceed to listen the information
            timeListened = atoi(&data[12]);
            /* Robot codes 
            301 TRIANGLE     100 timePickSource       106 timeStore
            302 BOX          101 timepickCache        107 timeHarvest
            303 CIRCLE       102 timeDropCache        201 timeImage
            304 ALL          103 timeDropNest         202 timeCache
            305 NOTHING      104 timeTravel2Nest      777 waiting
            306 ROBOT        105 timeTravel2Source
            */
            if ((codeReceived >= 100) && (codeReceived <= 107)){	
              printf("\n Thanks %d buddy, %s will consider your estimations for %d of time %d", name, robotName, codeReceived, timeListened);
              printf("\n");
              flagListened = 1;
              wb_robot_step(32); // to update global values
              updateEstimations(codeReceived, timeListened, 0);
            }
          }
          break;
        } else {
          printf("\n %s receive a message from %d, but it is not for him %s", robotName, name, data);
          printf("\n");
        }
      }
      wb_receiver_next_packet(receiver);
    } else {
      printf("\n %s receive a message no for him %s", robotName, data);
      printf("\n");
      wb_receiver_next_packet(receiver);
    }
  }  
  return 1;
}

int computeTraveling (int levy){ 
  float Ppartitioning = 0;
  float p = 0;
  float tFull;
  float tPart;
  // task1 = {Pick-source, Drop-Cache}
  // task2 = {Pick-cache, Drop-Nest}

  tFull = estPickS + estDropN;
  if (tFull == 0) { tFull = 1;}

  tPart = estPickS + estDropN;  
  if (tPart == 0) { tPart = 1;}
  
  switch(modelTest){
    case RANDOMLY:
      writeDecision(1.01, 0.01, TRAVELING_AGREE);
      return 1;
    break;
    case NEVER:
      writeDecision(0.01, 1.01, TRAVELING_AGREE);
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
        switch(stateUML){
          case PICK_SOURCE:
            //printf("\n %s levy pick_source", robotName);
            tPart = timeMeasured + estDropN; 
          break;
          case DROP_NEST:
            //printf("\n %s levy drop_nest", robotName);
            tPart = estPickS + timeMeasured;
          break;                 
          case TRAVEL2GREY:
          case TRAVEL2BLUE:
          case TRAVEL2RED:
            switch(statePrevious){
              case PICK_SOURCE:
                //printf("\n %s levy travel from pick-source", robotName);
                tPart = timeMeasured + estDropN;
              break;
             case DROP_NEST:
                //printf("\n %s levy travel from drop-nest", robotName);
                tPart = estPickS + timeMeasured; 
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
    writeDecision(Ppartitioning, p, TRAVELING_LEVY);
  } else {
    writeDecision(Ppartitioning, p, TRAVELING_AGREE);
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
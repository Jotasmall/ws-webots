/*
 * File:          TAM_4x3.c
 * Date:          
 * Description:   
 * Author: Jotasmall        
 * Modifications: 
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
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
#include <windows.h>

#define TIME_STEP 128
#define TIME4LOAD 75 // Time to load or unload a package
#define TIME4ARRIVAL 30  // Time to robot confirm arrival

// This is the cost to get each object 75, 150, 350, 700, 1000
#define TIME4NEST 75
#define MINUTES_EMPTY 0
float pDisableNestRed  = 0.0;
float pDisableNestGrey = 0.2;
float pDisableNestBlue = 0.6;
float pDisable = 0.0;
#define nRobots 7
int listWorkers[nRobots] = {0}; // number of robots
int listBusy[nRobots] = {0};
int lastVisitor = 0;
int flagOnebyOne = 1;
int flagWaitingDeparture = 0;
// Communication flags
int flagFiles = 0;
int flagCom = 1;                //to enable or disable communications 
WbDeviceTag receiver;
WbDeviceTag emitter;
#define M2ROBOT 1
#define M2NEST 2
#define M2WORKERS 3
#define LEAVE 11
#define COME 12
#define ROBOT_LEAVING 31
#define ROBOT_ARRIVING 32
#define ROBOT_UPDATING 33
#define ROBOT_NEGATIVE 34
#define ROBOT_AFFIRMATIVE 35
// Destinations
#define RED 0
#define GREY 1
#define BLUE 2
// Robot data
char robotName[8];
int codeTam = 0;
int robotLeaving;
int place2Go;
#define NEIGHBORS 3
float utility[] = {nRobots,nRobots,nRobots};
int resources[] = {0, 0, 0};
#define NB_TAM 10
#define noS 5
int sensorSource[] = {5,6,7,8,9};
#define noN 5
int sensorNest[] = {0,1,2,3,4};

WbDeviceTag led[NB_TAM];
WbDeviceTag ds1[NB_TAM]; // each led has two sensors
WbDeviceTag ds2[NB_TAM];
int ds_value1[NB_TAM];
int ds_offset1[NB_TAM];
int ds_value2[NB_TAM];
int ds_offset2[NB_TAM];

#define THRESHOLD_DIST 350
#define THRESHOLD_AWAY 100

#define FREE_SOURCE 1
#define LOAD_SOURCE 2
#define LEAVE_SOURCE 3
#define WAIT_SOURCE 4
#define FREE_NEST 5
#define UNLOAD_NEST 6
#define LEAVE_NEST 7
#define WAIT_NEST 8
#define DISABLE 9
int stateSource[noS];// = {FREE_SOURCE, FREE_SOURCE, FREE_SOURCE, FREE_SOURCE};//HERE
int stateNest[noN];// = {FREE_NEST, FREE_NEST, FREE_NEST, FREE_NEST};//HERE

int packageSource = 0;
int packageNest = 0;
int contArrivalSource[noS];
int contArrivalNest[noN];
int contLoad[noS];
int contUnload[noN];
long int timeCounter = 0;
int timeCycle = 0;
int timeMinute = 0;
int date = 0;

char fileRobot[] = "./DIRPATH/dd-hh-mm/TAM-COLOR##.txt";
char fileAux[] = "./DIRPATH/dd-hh-mm/TAM-COLOR_AUX##.txt";
#define MY_MASK 0777/home/sim/
#include <errno.h>

void W_initVariables();
void W_reset();
void W_led_ON();
void W_led_OFF();
void W_initialize();
void W_calibrate(int n);

void excuteProcess();
void W_read_dsensor(int print);
int  W_waiting(int n);
void writeFile(int idPlace);
void W_updateNests();
void W_updateSources();
void W_printStates();
void W_updateUtility(int amount);
int  W_speaking(int toWhom);
int  W_listening();
int W_fireWorkers();
void W_recordWorkers();
void W_sortWorkers();
// Robot files
#define COMMUNICATION 5
#define WORKERS 6
#define UTILITY 7
#define VISIT 8

void W_createFiles();
void W_updateFiles();
void W_createDir(int option, int dirBuild);
void W_writeMessage(int speaking, const char *msg);  

char dirPath[] = "./dir-dd-hh-mm";
time_t rawtime;
struct tm * timeinfo;

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  time (&rawtime);
  timeinfo = localtime(&rawtime);
  date = timeinfo->tm_mday+timeinfo->tm_hour+timeinfo->tm_min;
  sprintf(dirPath,"./%d-%d-%d", timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
  //printf("\n dir %s", dirPath);
  
  W_reset();
  
  wb_robot_step(TIME_STEP);
  W_initialize();
  
  excuteProcess();
  wb_robot_cleanup();
  
  return 0;
}

void excuteProcess(){
  int i;  
  while (wb_robot_step(TIME_STEP) != -1) {
    timeCounter++;
    timeCycle++;
    W_read_dsensor(0);
    W_updateNests();
    W_updateSources();
    W_listening();
    // W_printStates();
    if (timeCycle == 78) {//448 -> 1 min
      timeCycle = 0;
      timeMinute++;
      W_fireWorkers();
      printf("\n %s cycle complete with ", robotName);
      for (i = 0; i < NEIGHBORS; i++) {
        printf("%d utility %.1f", i, utility[i]);
      }
      printf("\n");
      if (timeMinute > MINUTES_EMPTY) {
    // cleaning Busy registers
        for (i = 0; i < nRobots; i++){
          listBusy[i] = 0; 
        }
        lastVisitor = 0;
        timeMinute = 0;
        W_speaking(M2NEST); //Synchronize with neighbors
      }
    }  
  }  
}  

void W_reset(){
  int i;
  memset(contArrivalSource, 0, noS*sizeof(contArrivalSource));
  memset(contArrivalNest, 0, noN*sizeof(contArrivalNest));
  memset(contLoad, 0, noS*sizeof(contLoad));
  memset(contUnload, 0, noN*sizeof(contUnload));
  // LEDS
  char textLed[] = "Wled00";
  for (i=0; i<NB_TAM; i++){
    sprintf(textLed, "Wled%d", i);
    led[i] = wb_robot_get_device(textLed);
  }
  wb_robot_step(TIME_STEP);
  W_led_ON();
  W_waiting(10);
  W_led_OFF();
  W_waiting(2);
  // SENSORS
  char textSensora[] = "Wads00";
  char textSensorb[] = "Wbds00";
  for (i=0; i<NB_TAM; i++){
    sprintf(textSensora, "Wads%d", i);
    ds1[i] = wb_robot_get_device(textSensora);
    sprintf(textSensorb, "Wbds%d", i);
    ds2[i] = wb_robot_get_device(textSensorb);
  } 
  wb_robot_step(TIME_STEP);  
  for (i=0; i< NB_TAM; i++){
    wb_distance_sensor_enable(ds1[i], TIME_STEP);
    wb_distance_sensor_enable(ds2[i], TIME_STEP);
  }
  W_calibrate(10);
  // COMMUNICATION
  receiver = wb_robot_get_device("receiver");
  emitter = wb_robot_get_device("emitter");
  wb_receiver_enable(receiver,TIME_STEP);
  // randomSeed by the code of the TAM
  strcpy(robotName, wb_robot_get_name());
  if (strcmp("tamRed", robotName) == 0) {
    //printf("\n I am TAM with ground color RED");
    pDisable = pDisableNestRed;
    codeTam = RED;
  }
  if (strcmp("tamGrey", robotName) == 0) {
    //printf("\n I am TAM with ground color GREY");
    pDisable = pDisableNestGrey;
    codeTam = GREY;
  }
  if (strcmp("tamBlue", robotName) == 0) {
    //printf("\n I am TAM with ground color BLUE");  
    pDisable = pDisableNestBlue;
    codeTam = BLUE;
  }  
  // FILES REGISTERS
  if (flagFiles) {W_createFiles();}
  
  strcpy(robotName, wb_robot_get_name());
  srand(codeTam*100+date);
  wb_robot_step(TIME_STEP);
 //* printf("\n %s is ready to work!", robotName);
}

void W_calibrate(int n){
  int i, it;
  //printf("\n Calibration begun...");
  for (i=0; i<NB_TAM; i++){
    ds_offset1[i] = 0;
    ds_offset2[i] = 0;
  }
  for (it=0; it<n; it++){
    for (i=0; i<NB_TAM; i++){
      ds_offset1[i] += (int) wb_distance_sensor_get_value(ds1[i]);
      ds_offset2[i] += (int) wb_distance_sensor_get_value(ds2[i]);
    }
  }
 //* printf("\n Sensor offset ");
  for (i=0; i<NB_TAM; i++){
    ds_offset1[i] /= n;
    ds_offset2[i] /= n;
   //* printf(" a %d b %d ", ds_offset1[i], ds_offset2[i]);
  }
 //* printf("\n Calibration is done.");
 //* printf("\n");
}

void W_initialize(){
  int i;
  float p = 0.0;
  for (i= 0; i<noS; i++){
    wb_led_set(led[sensorSource[i]], 1);
  }
  for (i= 0; i<noS; i++){
    p = ((float)rand())/RAND_MAX;
    if (pDisable > p) {
      stateSource[i] = DISABLE;
     //* printf("\n source %d in %s go to rest with p %.2f", i, robotName, p);
    } else {
      stateSource[i] = FREE_SOURCE;
     //* printf("\n source %d in %s go for service with p %.2f", i, robotName, p);
    } 
  }
  for (i=0; i<noN; i++){
    stateNest[i] = FREE_NEST;
  }
  wb_robot_step(TIME_STEP);
}

void W_updateNests(){
  int n;
    // FSM for nests!
    for (n=0; n<noN; n++){
      switch (stateNest[n]){
        case DISABLE:
          wb_led_set(led[sensorNest[n]], 0);
          break;
        case FREE_NEST:
          wb_led_set(led[sensorNest[n]], 1);
          if ((ds_value1[sensorNest[n]]>THRESHOLD_DIST) || (ds_value2[sensorNest[n]]>THRESHOLD_DIST)){ 
            contArrivalNest[n]++;
            //juan timeMinute = 0;
            if (contArrivalNest[n]>TIME4ARRIVAL){
              contArrivalNest[n] = 0;
              stateNest[n] = UNLOAD_NEST; 
            }
          } else {
            contArrivalNest[n] = 0;
          }
          break;
        case UNLOAD_NEST:
          wb_led_set(led[sensorNest[n]], 2);
          if ((ds_value1[sensorNest[n]]>THRESHOLD_DIST) || (ds_value2[sensorNest[n]]>THRESHOLD_DIST)) {
            contUnload[n]++;
            if (contUnload[n]>TIME4NEST){
              contUnload[n] = 0;
              packageNest++;
              writeFile(sensorNest[n]);
              stateNest[n] = LEAVE_NEST; 
            }
          } else {
            contUnload[n] = 0;
            stateNest[n] = FREE_NEST;
          }
          break;  
        case LEAVE_NEST:
          wb_led_set(led[sensorNest[n]],0);
          if ((ds_value1[sensorNest[n]]<THRESHOLD_AWAY) || (ds_value2[sensorNest[n]]<THRESHOLD_AWAY)) {
            W_waiting(5);
            stateNest[n] = FREE_NEST;
          } 
          break;
      }
    }
}

void W_updateSources(){
  int s;
  float p;
    // FSM for sources!
    for (s=0; s<noS; s++){
      switch (stateSource[s]){
        case DISABLE:
         if (timeCounter%470 == 0){
            p = ((float)rand())/RAND_MAX;
            if (pDisable < p) {
              stateSource[s] = FREE_SOURCE;
            }
          } else {
            wb_led_set(led[sensorSource[s]], 0);           
          }
          break;
        case FREE_SOURCE:
          wb_led_set(led[sensorSource[s]], 1);
          if ((ds_value1[sensorSource[s]]>THRESHOLD_DIST) || (ds_value2[sensorSource[s]]>THRESHOLD_DIST)){
            contArrivalSource[s]++;
            if (contArrivalSource[s]>TIME4ARRIVAL){
              contArrivalSource[s] = 0;
              stateSource[s] = LOAD_SOURCE; 
            }
          } else {
            contArrivalSource[s] = 0;
          }
          break;
        case LOAD_SOURCE:
          wb_led_set(led[sensorSource[s]], 2);
          if ((ds_value1[sensorSource[s]]>THRESHOLD_DIST) || (ds_value2[sensorSource[s]]>THRESHOLD_DIST)){ 
            contLoad[s]++;
            if (contLoad[s]>TIME4LOAD){
              contLoad[s] = 0;
              packageSource++;
              writeFile(sensorSource[s]);
              stateSource[s] = LEAVE_SOURCE; 
            }
          } else {
            contLoad[s] = 0;
            stateSource[s] = FREE_SOURCE;
          }
          break;  
        case LEAVE_SOURCE:
          wb_led_set(led[sensorSource[s]],0);
          if ((ds_value1[sensorSource[s]]<THRESHOLD_AWAY) || (ds_value2[sensorSource[s]]<THRESHOLD_AWAY)){
            W_waiting(5);
            p = ((float)rand())/RAND_MAX;
            if (pDisable > p) {
              stateSource[s] = DISABLE;
            } else {
              stateSource[s] = FREE_SOURCE;
            }  
          } 
          break;
      }
    }
}

void W_printStates(){
  int i;
  printf("\n We sources ares on ");
  for (i=0; i<noS; i++){
    printf("\n %d is %d", i, stateSource[i]);  
  }
  printf("\n We nests ares on ");
  for (i=0; i<noN; i++){
    printf("\n %d is %d", i, stateNest[i]);  
  }
}

void W_read_dsensor(int print){
  int i, it, samples = 1;//samples = 3
  for (i=0; i<NB_TAM; i++){
    ds_value1[i] = 0;
    ds_value2[i] = 0;
  }
  for (it=0; it<samples; it++){
    for (i=0; i<NB_TAM; i++){
      ds_value1[i] += (int) wb_distance_sensor_get_value(ds1[i]);
      ds_value2[i] += (int) wb_distance_sensor_get_value(ds2[i]); 
    }
  }
  if (print) {
    printf("\n Sensor values ");
    for (i=0; i<NB_TAM; i++){
      ds_value1[i] /= samples;
      printf("-a %d ", ds_value1[i]);
      ds_value2[i] /= samples;
      printf("-b %d ", ds_value2[i]);
    }
  }    
}

void W_createFiles(){
  W_createDir(COMMUNICATION, 1);
  FILE *fcom = fopen(fileRobot, "w");
  if (fcom == NULL) {
    printf("Error opening file com\n");
    printf("\n");
    exit(1);
  }
  fclose(fcom);
  
  W_createDir(WORKERS, 1);
  FILE *fworker = fopen(fileRobot, "w");
  if (fworker == NULL) {
    printf("Error opening file robot\n");
    printf("\n");
    exit(1);
  }
  fclose(fworker);
  
  W_createDir(UTILITY, 1);
  FILE *futi = fopen(fileRobot, "w");
  if (futi == NULL) {
    printf("Error opening file utility\n");
    printf("\n");
    exit(1);
  }
  fclose(futi);

  W_createDir(VISIT, 1);
  FILE *fvis1 = fopen(fileRobot, "w");
  if (fvis1 == NULL) {
    printf("Error opening file visit\n");
    printf("\n");
    exit(1);
  }
  FILE *fvis2 = fopen(fileAux, "w");
  if (fvis2 == NULL) {
    printf("Error opening file visits-aux\n");
    printf("\n");
    exit(1);
  }
  int row;
  for (row=0; row<NB_TAM; row++){
    fprintf(fvis1, "Place%d %3d\n", row, 0);
    fprintf(fvis2, "Place%d %3d\n", row, 0);
  }
  fclose(fvis1);
  fclose(fvis2);
}

void W_createDir(int option, int dirBuild){
  switch(option){
    case COMMUNICATION:
      sprintf(fileRobot, "%s-COM", dirPath); 
      //printf("\n fileRobot %s", fileRobot);
      //printf("\n"); 
      break;
    case WORKERS:
      sprintf(fileRobot, "%s-PER", dirPath);
      //printf("\n fileRobot %s", fileRobot);
      //printf("\n"); 
      break;
    case UTILITY:
      sprintf(fileRobot, "%s-PER", dirPath); 
      //printf("\n fileRobot %s", fileRobot);
      //printf("\n"); 
      break;
    case VISIT:
      sprintf(fileRobot, "%s-VIS", dirPath); 
      //printf("\n fileRobot %s", fileRobot);
      //printf("\n"); 
      break;
   }
   
   //if (dir){ mkdir(fileRobot,0700);} 
   if (dirBuild){ CreateDirectory(fileRobot, NULL);} 
    
   switch(option){
     case COMMUNICATION:
       sprintf(fileRobot, "%s/Node%d-COM.txt", fileRobot, codeTam);
       //printf("\n fileRobot %s", fileRobot);
       //printf("\n"); 
       break;       
     case WORKERS:
       sprintf(fileRobot, "%s/Node%d-WOR.txt", fileRobot, codeTam);
       //printf("\n fileRobot %s", fileRobot);
       //printf("\n");
       break;       
     case UTILITY:
       sprintf(fileRobot, "%s/Node%d-UTI.txt", fileRobot, codeTam);
       //printf("\n fileRobot %s", fileRobot);
       //printf("\n"); 
       break;
     case VISIT:
       sprintf(fileAux, "%s/Node%d-auxV.txt", fileRobot, codeTam);
       //printf("\n fileRobot auxiliar %s", fileAux);
       //printf("\n"); 
       sprintf(fileRobot, "%s/Node%d-VIS.txt", fileRobot, codeTam);
       //printf("\n fileRobot %s", fileRobot);
       //printf("\n"); 
       break;
     
  }
  if (dirBuild){
    //printf("\n %s is accessing to %s", robotName, fileRobot);
  }  
}

int W_waiting(int n){
  while(n>0) {
    n--;
    wb_robot_step(TIME_STEP);    
  }
  return 1;
}

void W_led_ON(){
  int i;
  for (i=0; i<NB_TAM; i++){
    wb_led_set(led[i], 1);
  }  
  wb_robot_step(TIME_STEP);
}

void W_led_OFF(){
  int i;
  for (i=0; i<NB_TAM; i++){
    wb_led_set(led[i], 0);
  }  
  wb_robot_step(TIME_STEP);
}

void W_updateFiles(){
  if (flagFiles) {
    W_createDir(VISIT, 0);
    FILE *fw=fopen(fileAux,"w");
    if (fw == NULL){
      printf("Error opening auxiliar file\n");
      exit(1);
    }
    FILE *fr=fopen(fileRobot,"r");
    if (fr == NULL ){
      printf("Error opening file 1 - update \n");
      exit(1);
    }
    rewind(fr);
    rewind(fw);
    int row, aux;
    char textPS[] = "Place00";
    
    for (row=0; row<NB_TAM; row++){
      fscanf(fr, "%s %3d", textPS, &aux);
      fprintf(fw, "Place%d %3d ", row, aux);
      fprintf(fw, "\n");
    }
    fclose(fw);
    fclose(fr);
    //printf("\n Final update");
  }
}

void writeFile(int idPlace){
  if (flagFiles) {
    W_createDir(VISIT, 0);
    FILE *fw=fopen(fileRobot,"w");
    if (fw == NULL){
      printf("Error opening file 1 - writing\n");
      exit(1);
    }
    FILE *fr=fopen(fileAux,"r");
    if (fr == NULL ){
      printf("Error opening auxiliar file \n");
      exit(1);
    }
    rewind(fr);
    rewind(fw);
    int row, aux;
    char textPS[] = "Place00";
  
    for (row=0; row<NB_TAM; row++){
      if (row==idPlace){
        fscanf(fr, "%s %3d", textPS, &aux);
        //printf("\n Value to be modified %d", aux);
        fprintf(fw, "Place%d %3d ", row, ++aux);
      } else {
        fscanf(fr, "%s %3d", textPS, &aux);
        fprintf(fw, "Place%d %3d ", row, aux);
      }
      fprintf(fw, "\n");
    }
    fclose(fw);
    fclose(fr);
    W_updateFiles();
  }
}

void W_updateUtility(int amount) {
  int i;
  utility[codeTam] -= amount;
  if (flagFiles) {
    W_createDir(UTILITY, 0);
    FILE *fw2 = fopen(fileRobot, "a");
    if (fw2 == NULL) {
      printf("Error opening file utilities \n");
      exit(1);
    }
    //C printf("\n %s is updating utilities %g, %g, %g", robotName, utility[0], utility[1], utility[2]);
    //C printf("\n");
    for (i = 0; i < NEIGHBORS; i++) {
      fprintf(fw2, "%d, ", (int) utility[i]);
    }
    fprintf(fw2,"\n");
    fclose(fw2);
  }  
} 

void W_writeMessage(int speaking, const char *msg) {
  if (flagFiles) {
      // File for decisions
    W_createDir(COMMUNICATION, 0);
    //printf("\n %s is registering its messages in %s", robotName, fileRobot);
    //printf("\n");    
    FILE *file = fopen(fileRobot, "a+");
    if (file == NULL) {
      printf("Error opening file of communications\n");
      printf("\n");
      exit(1);
    }
    //printf("\n %s is updating with %s", robotName, msg);
    if (speaking) {
      fprintf(file, "\n speaking, %s", msg);
      //printf("\n %s is updating with %s by speaking", robotName, msg);
    } else {
      fprintf(file, "\n listening, %s", msg);
      //printf("\n %s is updating with %s by listening", robotName, msg);
    }
    fclose(file);
  }
}

int W_speaking(int toWhom){ //ok-
  if (flagCom == 0) { return 0;}

  char message[30];
  int i;
  
  // wb_emitter_set_channel(emitter, WB_CHANNEL_BROADCAST);
  if (toWhom == M2NEST) { 
    // reporting just to have the same number of lines
    sprintf(message, "T2T%dX%.1fZ", codeTam, utility[codeTam]);
    wb_emitter_send(emitter, message, strlen(message)+1);
    W_writeMessage(1, message);
   //printf("\n %s communicates its utility %d, info nests %d, %d, %d", robotName, utility[codeTam], utility[0], utility[1], utility[2]);
   //printf("\n"); 
  } else if (toWhom == M2ROBOT) {
    W_fireWorkers();
  } else if (toWhom == M2WORKERS) {
    //printf("\n %s is introducing the new %d", robotName, lastVisitor);
    for (i = 0; i<nRobots; i++) {
      if ((lastVisitor != 0) && (listWorkers[i] != 0) && (listWorkers[i] != lastVisitor)) {
        sprintf(message, "T2R%dR%dR%d", codeTam, lastVisitor, listWorkers[i]);
        wb_emitter_send(emitter, message, strlen(message)+1);
        W_writeMessage(1, message);
        //printf("\n %s introduces %d to %d", robotName, listWorkers[i], lastVisitor);
        //printf("\n");
      }
    }
  }
  return 1;
}

int W_listening() { 
  int i, j;
  int robot;
  int value;
  const char *data;
  //printf("\n %s is receiving a message %s", robotName);
  while(wb_receiver_get_queue_length(receiver)>0){  
    //printf("\n %s has received a message", robotName);
    data = wb_receiver_get_data(receiver);
    if ((data[0] == 'T') && (data[2] == 'T')) {
      robot = atoi(&data[3]); //Maximum 9 senders (NEST)
      value = atof(&data[5]); //utility value
      utility[robot] = value;
      W_updateUtility(0);
      //printf("\n %s received a message from %d Nest", robotName, robot);
      //printf("\n %s update neighbor %d utility %d", robotName, robot, value);
      wb_receiver_next_packet(receiver);
    } else if ((data[0] == 'R') && (data[2] == 'T')) {     
      // R2T0000T##X999
      robot = atoi(&data[3]); 
      // LEAVE/STAY/AFFIRMATIVE/NEGATIVE 
      int action = atoi(&data[8]);
      // DESTINATARY
      value = atoi(&data[11]); 
      // The message is for this TAM
      if (value == codeTam){
        //printf("\n %s receive %s as message from %d robot doing %d with value %d", robotName, data, robot, action, value);
        //printf("\n");
        switch(action){
          case ROBOT_LEAVING: 
            for (i = 0; i < nRobots; i++){
              if (robot == listWorkers[i]){
                W_writeMessage(0, data);
                W_updateUtility(-1);                            
                listWorkers[i] = 0;
                W_sortWorkers();
                if (flagOnebyOne == 1) { flagWaitingDeparture = 0;} // flag deactivated, call other
                printf("\n %s removed from its list %d", robotName, robot);
                printf("\n");               
                break;
              }
            }
          break;          
          case ROBOT_ARRIVING:
            for (i = 0; i < nRobots; i++) {
              if (listWorkers[i] == 0) {
                W_writeMessage(0, data);
                W_updateUtility(1);
                listWorkers[i] = robot;
                lastVisitor = robot;
                printf("\n %s add to its list %d", robotName, robot);
                printf("\n");
                W_speaking(M2WORKERS); 
                break; //only add it once
              }
            }
          break;
          case ROBOT_AFFIRMATIVE:
            for (i = 0; i < nRobots; i++) {
              if (listWorkers[i] == robot) {
                W_writeMessage(0, data);
                if (flagOnebyOne == 1) { flagWaitingDeparture = 1;} // flag activated by acceptation
                break;
              }
            }
            break;
          case ROBOT_NEGATIVE:
            for (i = 0; i < nRobots; i++) {
              if (listWorkers[i] == robot) {
                W_writeMessage(0, data);
                flagWaitingDeparture = 0; // flag deactivate by negative
                printf("\n %s received from %d a negative answer", robotName, robot);
                printf("\n");
                for (j = 0; j < nRobots; j++) {
                  if (listBusy[j] == 0) {
                    listBusy[j] = robot;
                    break;
                  }
                }
                W_fireWorkers();
                break;              
              }            
            }
            break;
          case ROBOT_UPDATING:          
            for (i = 0; i < nRobots; i++){
              if (robot == listWorkers[i]){
                //printf("\n %s has received a time of %d from %d", robotName, value, robot);
                //printf("\n");
              }
            } 
            break;
        }   
      }  
      wb_receiver_next_packet(receiver);
    } else {  
      //printf("\n %s receive %s message for other", robotName, data);
      wb_receiver_next_packet(receiver);
    }  
  }
  return 1;
}

void W_sortWorkers(){ 
  int j;
  for (j = 0; j < nRobots-1; j++) {
    if (listWorkers[j] == 0) {
      listWorkers[j] = listWorkers[j+1];
      listWorkers[j+1] = 0;
    }  
  }
  printf("\n %s changing the list of workers from", robotName);
  for (j = 0; j < nRobots; j++) {
    printf("%d ", listWorkers[j]);
  }
  printf("\n");
  W_recordWorkers();
} 

void W_recordWorkers(){
  int i;
  if (flagFiles) {
    W_createDir(WORKERS, 0);
    FILE *fw1 = fopen(fileRobot,"a");
    if (fw1 == NULL){
      printf("Error opening file workers\n");
      exit(1);
    }
    for (i = 0; i < nRobots; i++){
      fprintf(fw1, "%d, ", listWorkers[i]);
    }
    fprintf(fw1,"\n");
    fclose(fw1);
  }
}

int W_fireWorkers(){
  place2Go = codeTam;
  float maxDif = -1;
  float dif;
  int i, j, k;
  int flagBusy = 0;
  char message[30];
  // Find a neighbor with better utility
  for (i = 0; i<NEIGHBORS; i++) {
    if ((i != codeTam) && (utility[i] > utility[codeTam])) {
      dif = utility[i]-utility[codeTam];
      if (dif > maxDif) {
        maxDif = dif;
        place2Go = i;
      }
    }  
  }
  //Rounding values
  if ((flagWaitingDeparture == 1) && (flagOnebyOne == 1)){
    return 0;
  } else if (maxDif > 1.0) {//>=
    if (flagOnebyOne == 1) {
      maxDif = 1;
    } else {
      maxDif /= 2;   // coop * [skj(rk) - sij(ri)] / 2 
      if (maxDif > 1) {
        maxDif = round(maxDif);
      } else {
        maxDif = ceil(maxDif);
      }
    }
    //C printf("\n %s is needing some %d robot goes out to %d", robotName, (int) maxDif, place2Go);
    //C printf("\n");
    if (place2Go != codeTam) {
      i = 0; //registered workers
      j = 0; //fired workers
      for (i = 0; i < nRobots; i++) { 
        flagBusy = 0;  
        // check each registered worker if there is room to fire
        if ((listWorkers[i] != 0) && (j < (int) maxDif)) { 
          for (k = 0; k < nRobots; k++) { 
            // a registered worker can be busy
            if ((listWorkers[i] == listBusy[k]) && (listWorkers[i] != lastVisitor)){ 
              flagBusy = 1;
              break;
            }     
          }
          // This workers is registered and free
        if (flagBusy == 0) {
          robotLeaving = listWorkers[i];
          j++; // increase fired workers
          //all printf("\n %s has chosen %d to leave toward %d", robotName, robotLeaving, place2Go);
          //all printf("\n %s also known as %d utilities values %g, %g, %g", robotName, codeTam, utility[0], utility[1], utility[2]);
          //all printf("\n");
          if (flagOnebyOne == 1) { flagWaitingDeparture = 1;} // changes only when worker says NEGATIVE
            sprintf(message, "T2R%dR%dT%dX%d", codeTam, robotLeaving, LEAVE, place2Go);
            wb_emitter_send(emitter, message, strlen(message)+1);
            W_writeMessage(1, message);
          }          
        }  
      }
    }
  }
  return 0;  
}
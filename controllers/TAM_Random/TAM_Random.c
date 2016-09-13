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

#define TIME_STEP 128
#define TIME4LOAD 75 // Time to load or unload a package
#define TIME4ARRIVAL 30  // Time to robot confirm arrival

// This is the cost to get each object 75, 150, 350, 700, 1000
#define TIME4NEST 75
#define MINUTES_EMPTY 2
float pDisableNestRed = 0.00;
float pDisableNestGrey = 0.33;
float pDisableNestBlue = 0.66;
float pDisable = 0.0;
#define nRobots 4
int listWorkers[] = {0,0,0,0}; // number of robots
int flagFiles = 0;

// Communication flags
int flagCom = 1;                //to enable or disable communications 
WbDeviceTag receiver;
WbDeviceTag emitter;
#define M2ROBOT 1
#define LEAVE 11
#define COME 12
#define ROBOT_LEAVING 31
#define ROBOT_ARRIVING 32
#define ROBOT_UPDATING 33
#define M2NEST 2

char robotName[8];
int codeTam = 0;
#define NEIGHBORS 3
int utility[] = {nRobots,nRobots,nRobots};
 
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
int timeMinute = 0;
int date = 0;
int resources[] = {0, 0, 0};

char dir[] = "dd-hh-mm";//"/home/sim/dd-hh-mm/";
char file[] = "dd-hh-mm/visit_vector.txt";
char fileAux[] = "dd-hh-mm/visit_vector_aux.txt";
#define MY_MASK 0777/home/sim/
#include <errno.h>

void init_variables();
void W_reset();
void W_led_ON();
void W_led_OFF();
void W_initialize();
void W_calibrate(int n);
void W_read_dsensor();
int W_waiting(int n);
void createFile();
void updateFiles();
void writeFile(int idPlace);
void W_updateNests();
void W_updateSources();
int W_speaking(int toWhom);
int listening();
void printStates();
void updateUtility(int amount);

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime(&rawtime);
  date = timeinfo->tm_mday+timeinfo->tm_hour+timeinfo->tm_min;
  sprintf(dir,"%d-%d-%d",timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
  //printf("\n dir %s", dir);

  init_variables(); 
  W_reset();
  W_led_ON();
  W_waiting(10);
  W_led_OFF();
  W_waiting(5);
  W_calibrate(10);
  wb_robot_step(TIME_STEP);
  W_initialize();
  
  /*
  int n;
  for(n=0; n<noN; n++){
    printf("\n Sensor nest %d state %d", n, stateNest[n]);
    printf("\n");
  }  
  */
  printf("\n Leds ready");
  
  if (flagFiles) { createFile();}
  
  while (wb_robot_step(TIME_STEP) != -1) {
    timeCounter++;
    W_read_dsensor();
    W_updateNests();
    W_updateSources();
    listening();
    //printStates();
    if (timeCounter%448 == 0) {
      timeMinute++;
      if (timeMinute > MINUTES_EMPTY) {
        printf("\n %s is gonna speak", robotName);
        timeMinute = 0;
        W_speaking(M2NEST); // Call of Duty
      }
    }  
  }  
  wb_robot_cleanup();
  
  return 0;
}


void printStates(){
  int i;
  //-- printf("\n We sources ares on ");
  for (i=0; i<noS; i++){
  //--   printf("\n %d is %d", i, stateSource[i]);  
  }
  printf("\n We nests ares on ");
  for (i=0; i<noN; i++){
  //--   printf("\n %d is %d", i, stateNest[i]);  
  }
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
            timeMinute = 0;
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
            while(!W_waiting(5));
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
              printf("\n source %d in %s go for service", s, robotName);
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
            while(!W_waiting(5));
            p = ((float)rand())/RAND_MAX;
            if (pDisable > p) {
              stateSource[s] = DISABLE;
            } else {
              stateSource[s] = FREE_SOURCE;
              printf("\n source %d in %s go for service", s, robotName);
            }  
          } 
          break;
      }
    }
}

void init_variables(){
  memset(contArrivalSource, 0, noS*sizeof(contArrivalSource));
  memset(contArrivalNest, 0, noN*sizeof(contArrivalNest));
  memset(contLoad, 0, noS*sizeof(contLoad));
  memset(contUnload, 0, noN*sizeof(contUnload));
  wb_robot_step(TIME_STEP);
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
      printf("\n source %d in %s go to rest with p %.2f", i, robotName, p);
    } else {
      stateSource[i] = FREE_SOURCE;
      printf("\n source %d in %s go for service with p %.2f", i, robotName, p);
    } 
  }
  for (i=0; i<noN; i++){
    stateNest[i] = FREE_NEST;
  }

  wb_robot_step(TIME_STEP);
}

void W_read_dsensor(){
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
   
  /*printf("\n Sensor values ");
  for (i=0; i<NB_TAM; i++){
    ds_value[i] /= samples;
    printf(" %d ", ds_value[i]);
  }*/
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
  printf("\n Sensor offset ");
  for (i=0; i<NB_TAM; i++){
    ds_offset1[i] /= n;
    ds_offset2[i] /= n;
    printf(" a %d b %d ", ds_offset1[i], ds_offset2[i]);
  }
  printf("\n Calibration is done.");
  printf("\n");
}

void W_reset(){
  int i;
 
  char textLed[] = "Wled00";
  for (i=0; i<NB_TAM; i++){
    sprintf(textLed, "Wled%d", i);
    led[i] = wb_robot_get_device(textLed);
  }
  
  char textSensora[] = "Wads00";
  char textSensorb[] = "Wbds00";
  for (i=0; i<NB_TAM; i++){
    sprintf(textSensora, "Wads%d", i);
    ds1[i] = wb_robot_get_device(textSensora);
    sprintf(textSensorb, "Wbds%d", i);
    ds2[i] = wb_robot_get_device(textSensorb);
  } 
  // communication module
  receiver = wb_robot_get_device("receiver");
  emitter = wb_robot_get_device("emitter");
  wb_receiver_enable(receiver,TIME_STEP);
   
  wb_robot_step(TIME_STEP);
  
  for (i=0; i< NB_TAM; i++){
    wb_distance_sensor_enable(ds1[i], TIME_STEP);
    wb_distance_sensor_enable(ds2[i], TIME_STEP);
  }
  //printf("\n World reset");
  // randomSeed by the code of the TAM
  wb_robot_step(TIME_STEP);
  
  //printf("\n TAM name is %s", wb_robot_get_name());
  strcpy(robotName, wb_robot_get_name());

  if (strcmp("tamRed", robotName) == 0) {
    //printf("\n I am TAM with ground color RED");
    pDisable = pDisableNestRed;
    codeTam = 0;
  }
  if (strcmp("tamGrey", robotName) == 0) {
    //printf("\n I am TAM with ground color GREY");
    pDisable = pDisableNestGrey;
    codeTam = 1;
  }
  if (strcmp("tamBlue", robotName) == 0) {
    //printf("\n I am TAM with ground color BLUE");  
    pDisable = pDisableNestBlue;
    codeTam = 2;
  }  
  strcpy(robotName, wb_robot_get_name());
  srand(codeTam*100+date);
  wb_robot_step(TIME_STEP);
 
  printf("\n %s is ready to work!", robotName);
}

int W_waiting(int n){
  int i;
  for (i=0; i<n; i++){
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


void createFile(){
  sprintf(file,"%s_visits_vector%d.txt", dir, codeTam);
  printf("\n file %s",file);

  FILE *fw1 = fopen(file,"w");
  if(fw1 == NULL){
      printf("Error opening file 1\n");
      exit(1);
  }
  sprintf(fileAux, "%s_visit_vector%d_aux.txt", dir, codeTam);

   FILE *fw2 = fopen(fileAux,"w");
  if(fw2 == NULL){
      printf("Error opening auxiliar file \n");
      exit(1);
  }

  int row;
  for (row=0; row<NB_TAM; row++){
    fprintf(fw1, "Place%d %3d\n", row, 0);
    fprintf(fw2, "Place%d %3d\n", row, 0);
  }
  
  fclose(fw1);
  fclose(fw2);
  printf("\n Files created!!");
}

void updateFiles(){
  if (flagFiles) {
  FILE *fw=fopen(fileAux,"w");
  if (fw == NULL){
      printf("Error opening auxiliar file\n");
      exit(1);
  }
  FILE *fr=fopen(file,"r");
  if(fr == NULL ){
      printf("Error opening file 1 \n");
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
  FILE *fw=fopen(file,"w");
  if (fw == NULL){
      printf("Error opening file 1\n");
      exit(1);
  }
  FILE *fr=fopen(fileAux,"r");
  if(fr == NULL ){
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
  updateFiles();
  }
}

void updateUtility(int amount) {
  int i;
  utility[codeTam]-=amount;
  for (i = 0; i < NEIGHBORS; i++) {
    if ((i != codeTam) && (utility[codeTam]<utility[i])) {  
      W_speaking(M2ROBOT);
      break;
    }
  }
}

int W_speaking(int toWhom){ //ok-
  if (flagCom == 0) { return 0;}

  char message[30];
  int i, place2Go = codeTam, dif, maxDif = -1;
  // wb_emitter_set_channel(emitter, WB_CHANNEL_BROADCAST);
  if (toWhom == M2NEST) { // reporting just to have the same number of lines
    sprintf(message, "T2T%dX%d", codeTam, utility[codeTam]);
    wb_emitter_send(emitter, message, strlen(message)+1);
    printf("\n %s communicates its utility %d, info nests %d, %d, %d", robotName, utility[codeTam], resources[0], resources[1], resources[2]);
    printf("\n"); 
  } else if (toWhom == M2ROBOT) {
    for (i=0; i<NEIGHBORS; i++) {
      if ((i != codeTam) && (utility[i] > utility[codeTam])) {
        dif = utility[i]-utility[codeTam];
        if (dif > maxDif) {
          maxDif = dif;
          place2Go = i;
        }
      }  
    }
    if (place2Go != codeTam) {
      sprintf(message, "T2R%dT%dX%d", codeTam, LEAVE, place2Go);
      wb_emitter_send(emitter, message, strlen(message)+1);
      printf("\n %s communicates to its robots", robotName);
      printf("\n");      
    } else {
      //sprintf(message, "T2R%dT%dX%d", codeTam, COME, codeTam);
      printf("\n %s has no neighbors needing", robotName);
      printf("\n");
    }  
  }
  wb_robot_step(32);
  return 1;
}

int listening() { 
  int i;
  //printf("\n %s is receiving a message %s", robotName);
  while(wb_receiver_get_queue_length(receiver)>0){  
    //printf("\n %s has received a message", robotName);
    const char *data = wb_receiver_get_data(receiver);
    if (data[0] == 'T') {
      if (data[2] == 'T') {
        int sender = atoi(&data[3]); //Maximum 9 senders (NEST)
        int value = atoi(&data[5]); //utility value
        printf("\n %s received a message from %d Nest", robotName, sender);
        printf("\n %s update neighbor %d utility %d", robotName, sender, value);
        utility[sender] = value; 
      }
      wb_receiver_next_packet(receiver);
    } else if (data[0] == 'R') {
      if (data[2] == 'T') {     
        //R2T0000T##X999
        int robot = atoi(&data[3]); 
        int action = atoi(&data[8]);
        int value = atoi(&data[11]);
        printf("\n %s receive %s as message from %d robot", robotName, data, robot);
        if (value == codeTam) {
          // The message is for this TAM
          if (action == ROBOT_LEAVING) {
            for (i = 0; i < nRobots; i++){
              if (robot == listWorkers[i]){
                // proceed to listen the information
                listWorkers[i] = 0;
                printf("\n %s removed from its list %d", robotName, robot);
                updateUtility(-1);
              }
            } 
          } else if (action == ROBOT_ARRIVING) {
            for (i = 0; i < nRobots; i++) {
              if (listWorkers[i] == 0) {
                listWorkers[i] = robot;
                printf("\n %s add to its list %d", robotName, robot);
                updateUtility(1);
                break;
              }
            }
          } else {          
            printf("\n %s has received %d from %d", robotName, value, robot);
            printf("\n");
          }  
        }  
      }
      wb_receiver_next_packet(receiver);
    }   
    //wb_receiver_next_packet(receiver);
  }
  return 1;
}
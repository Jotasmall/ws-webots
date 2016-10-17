#include "registers.h"
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
#define IMAGE 201
// Robot files
#define FSM 0
#define ESTIMATIONS 1
#define LIFE 2
#define DECISIONS 3
#define PERFORMANCE 4 
#define COMMUNICATION 5
#define M2ROBOT 1

void updateEstimations(int task, int value, int cache, int timeImage, int timeMeasured, int timeListened, struct robotState *botState, struct modelParam *botParam, struct robotEstimations *botEst, struct robotDevices *botDevices, struct flags4Files *botFlags){ //ok-
  //0 means no listen nothing 1 forget all for new data
  int codeTask = task; 
  if (botParam->flagMomento != 1) { botParam->beta = 0;}
  if (botDevices->flagListened == 0) {
    value = timeMeasured;
    wb_robot_step(32);
    //printf("\n for state %d time measured is %d", codeTask, value);
  } 

  switch(task){
    case PICK_SOURCE:
      botEst->estPickS = (botEst->estPickS * (100 - botParam->alpha) + value * botParam->alpha + (botEst->estPickS - value) * botParam->beta) / 100;
      break;
    case DROP_NEST:
      botEst->estDropN = (botEst->estDropN * (100 - botParam->alpha) + value * botParam->alpha + (botEst->estDropN - value) * botParam->beta ) / 100;
      break;
    case TRAVEL2GREY:
      botEst->estTravelGrey = (botEst->estTravelGrey  * (100 - botParam->alpha) + value * botParam->alpha + (botEst->estTravelGrey  - value) * botParam->beta ) / 100;
      break;
    case TRAVEL2BLUE:
      botEst->estTravelBlue = (botEst->estTravelBlue  * (100 - botParam->alpha) + value * botParam->alpha + (botEst->estTravelBlue  - value) * botParam->beta ) / 100;
      break;
    case TRAVEL2RED:
      botEst->estTravelRed = (botEst->estTravelRed  * (100 - botParam->alpha) + value * botParam->alpha + (botEst->estTravelRed  - value) * botParam->beta ) / 100;
      break;
  }
  cache = 0; // comment when working with shapes
  
  updateBitacora(0, ESTIMATIONS, cache, timeMeasured, timeListened, botEst, botFlags, botDevices, botState);
  if (codeTask != IMAGE) { 
    updateBitacora(codeTask, FSM, cache, timeMeasured, timeListened, botEst, botFlags, botDevices, botState); 
    if (botDevices->flagListened) {
      botDevices->flagListened = 0;
    } else {
      printf("\n Everybody listen, I am %d, my %d cost me %d", botState->botNumber, codeTask, value);
      printf("\n");
      speaking(botDevices, botState->botNumber, M2ROBOT, codeTask, value, cache, botFlags);
    }  
  } 
  botEst->lastImage = timeImage;
  timeImage = 0;  
  timeMeasured = 0;
  timeListened = 0;
  wb_robot_step(32);
}

void updateBitacora(int codeTask, int estimations, int cache, int timeMeasured, int timeListened, struct robotEstimations *botEst, struct flags4Files *botFlags, struct robotDevices *botDevices, struct robotState *botState){ //ok-
  if (estimations == ESTIMATIONS) { 
    if (botFlags->flagFilesEST) { 
      createDir(ESTIMATIONS, 0, botFlags); 
      //printf("\n %s is estimating times in %s", botState->botNumber, fileRobot);
      //printf("\n");
      FILE *fbot = fopen(botFlags->fileRobot, "a+");
      if (fbot==NULL) {
        printf("Error opening file of estimations bot\n");
        printf("\n");
        exit(1);
      }
      
      if (botDevices->flagListened == 1) {
        fprintf(fbot, "Update after heard for cache %d, %d, %d, %d, %d, %d \n", 
                   botEst->estPickS, botEst->estDropN, botEst->estTravelGrey, 
                   botEst->estTravelBlue, botEst->estTravelRed, botEst->lastImage);
      } else {
        fprintf(fbot, "Update after finish for cache %d, %d, %d, %d, %d, %d \n", 
                   botEst->estPickS, botEst->estDropN, botEst->estTravelGrey, 
                   botEst->estTravelBlue, botEst->estTravelRed, botEst->lastImage);
      }             
      fclose(fbot); //-- JUAN EDIT FILES
      }
  } else {    
    if (botFlags->flagFilesFSM) {
      createDir(FSM, 0, botFlags); 
      //printf("\n %d is on state machine times in %s", botState->botNumber, fileRobot);
      //printf("\n");
      FILE *fbot = fopen(botFlags->fileRobot, "a+");    
      if (fbot==NULL) {
        printf("Error opening file bot\n");
        printf("\n");
        exit(1);
      }
      
      char stringState[] = "SEARCHING SOMETHING";
      int innerState = botState->currentState;
      if (botDevices->flagListened) { innerState = codeTask;}
      
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
      if (botDevices->flagListened) {
        fprintf(fbot,"Listened %s, 0, %d\n", stringState, timeListened);
      } else {
        fprintf(fbot,"Executed %s, %d, %d\n", stringState, codeTask, timeMeasured);
      }
      fclose(fbot);  //-- JUAN EDIT FILES           
    }                
  } 
}

/*
void cronometer(int task, int cache, int *suggestedState, int *timeImage, int *timeMeasured, struct robotDevices *botDevices, struct robotState *botState, struct flags4Files *botFlags){//ok-
  
  if (task == IMAGE) { 
    (*timeImage)++;
    //printf("\n Time images %d", *timeImage);
    //printf("\n");
  } else {  
    (*timeMeasured)++;
  }
  //printf("\n %s is listening", robotName);
  //printf("\n");
//juan  listening(botDevices.receiver, floorColor, botNumber, listFriends, &stateUML, &suggestedState, &botFlagFiles); //--JUAN EDIT FILES
  listening(botDevices->receiver, botState->floorColor, botState->botNumber, botState->listFriends, &botState->currentState, suggestedState, botFlags); //--JUAN EDIT FILES

  if (botFlags->flagFilesLIFE) {
    createDir(LIFE, 0, botFlags);
    //printf("\n %s is updating in %s", robotName, fileRobot);
    //printf("\n");
    FILE *flife = fopen(botFlags->fileRobot,"a+");
    if (task == IMAGE) { 
      fprintf(flife, "image, %d \n", *timeImage);
    } else {  
     fprintf(flife, "state %d, %d\n", botState->currentState, *timeMeasured);
    }
    fclose(flife); //-- JUAN EDIT FILES 
  } 
}
*/
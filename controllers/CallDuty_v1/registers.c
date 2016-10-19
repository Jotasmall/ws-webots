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

void updateEstimations(int task, int cache){ //ok-
  int value = bot.timeMeasured;
  int codeTask = bot.currentState; 
  if (bot.flagMomento != 1) { bot.beta = 0;}
  if (bot.flagListened) {
    value = bot.timeListened;
    wb_robot_step(32);
    //printf("\n for state %d time measured is %d", codeTask, value);
  } 

  switch(task){
  case PICK_SOURCE:
    bot.estPickS = (bot.estPickS * (100 - bot.alpha) + value * bot.alpha + (bot.estPickS - value) * bot.beta) / 100;
    break;
  case DROP_NEST:
    bot.estDropN = (bot.estDropN * (100 - bot.alpha) + value * bot.alpha + (bot.estDropN - value) * bot.beta) / 100;
    break;
  case TRAVEL2GREY:
    bot.estTravelGrey = (bot.estTravelGrey  * (100 - bot.alpha) + value * bot.alpha + (bot.estTravelGrey  - value) * bot.beta) / 100;
    break;
  case TRAVEL2BLUE:
    bot.estTravelBlue = (bot.estTravelBlue  * (100 - bot.alpha) + value * bot.alpha + (bot.estTravelBlue  - value) * bot.beta) / 100;
    break;
  case TRAVEL2RED:
    bot.estTravelRed = (bot.estTravelRed  * (100 - bot.alpha) + value * bot.alpha + (bot.estTravelRed  - value) * bot.beta) / 100;
    break;
  }
  cache = 0; // comment when working with shapes
  
  updateBitacora(0, ESTIMATIONS, cache);
  if (codeTask != IMAGE) { 
    updateBitacora(codeTask, FSM, cache); 
    if (bot.flagListened) {
      bot.flagListened = 0;
    } else {
      printf("\n Everybody listen, I am %d, my %d cost me %d", bot.botNumber, codeTask, value);
      printf("\n");
      speaking(M2ROBOT, codeTask, value, cache);
    }  
  } 
  bot.lastImage = bot.timeImage;
  bot.timeImage = 0;  
  bot.timeMeasured = 0;
  bot.timeListened = 0;
  wb_robot_step(32);
}

void updateBitacora(int codeTask, int estimations, int cache){ //ok-
  if (estimations == ESTIMATIONS) { 
    if (bot.flagFilesEST) { 
      createDir(ESTIMATIONS, 0); 
      //printf("\n %s is estimating times in %s", bot.botNumber, fileRobot);
      //printf("\n");
      FILE *fbot = fopen(bot.fileRobot, "a+");
      if (fbot==NULL) {
        printf("Error opening file of estimations bot\n");
        printf("\n");
        exit(1);
      }
      if (bot.flagListened == 1) {
       fprintf(fbot, "Update after heard for cache %d, %d, %d, %d, %d, %d \n", 
                     bot.estPickS, bot.estDropN, bot.estTravelGrey, 
                     bot.estTravelBlue, bot.estTravelRed, bot.lastImage);
     } else {
       fprintf(fbot, "Update after finish for cache %d, %d, %d, %d, %d, %d \n", 
                     bot.estPickS, bot.estDropN, bot.estTravelGrey, 
                     bot.estTravelBlue, bot.estTravelRed, bot.lastImage);
     }     
     fclose(fbot); //-- JUAN EDIT FILES
    }
  } else {  
  if (bot.flagFilesFSM) {
    createDir(FSM, 0); 
    //printf("\n %d is on state machine times in %s", bot.botNumber, fileRobot);
    //printf("\n");
    FILE *fbot = fopen(bot.fileRobot, "a+");  
    if (fbot==NULL) {
      printf("Error opening file bot\n");
      printf("\n");
      exit(1);
    }
    char stringState[] = "SEARCHING SOMETHING";
    int innerState = bot.currentState;
    if (bot.flagListened) { innerState = codeTask;}
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
    if (bot.flagListened) {
      fprintf(fbot,"Listened %s, 0, %d\n", stringState, bot.timeListened);
    } else {
      fprintf(fbot,"Executed %s, %d, %d\n", stringState, codeTask, bot.timeMeasured);
    }
    fclose(fbot);  //-- JUAN EDIT FILES     
    }      
  } 
}

void cronometer(int task, int cache){//ok-
  
  if (task == IMAGE) { 
    bot.timeImage++;
    //printf("\n Time images %d", bot.timeImage);
    //printf("\n");
  } else {  
    bot.timeMeasured++;
  }
  //printf("\n %s is listening", robotName);
  //printf("\n");
  listening(bot); //--JUAN EDIT FILES

  if (bot.flagFilesLIFE) {
    createDir(LIFE, 0);
    //printf("\n %s is updating in %s", robotName, fileRobot);
    //printf("\n");
    FILE *flife = fopen(bot.fileRobot,"a+");
    if (task == IMAGE) { 
      fprintf(flife, "image, %d \n", bot.timeImage);
    } else {  
      fprintf(flife, "state %d, %d\n", bot.currentState, bot.timeMeasured);
    }
    fclose(flife); //-- JUAN EDIT FILES 
  } 
}

void countObjects(int nbRegions){
  switch(bot.currentState){
  case DROP_NEST:
    bot.nDrop[bot.floorColor]++; break;
  case PICK_SOURCE:
    bot.nPick[bot.floorColor]++; break;
  }
  printf("\n We have %d objects picked", bot.nPick[bot.floorColor]);
  printf("\n");
  if (bot.flagFilesPER) {
    createDir(PERFORMANCE, 0);
    //printf("\n %s is counting objects in %s", robotName, fileRobot);
    //printf("\n");
    FILE *fper = fopen(bot.fileRobot, "a+");
    int i;
    for (i=0; i<nbRegions; i++){ 
      fprintf(fper, "%d, %d, ", bot.nPick[i], bot.nDrop[i]);
    }
    fprintf(fper, "\n");  
    fclose(fper); //-- JUAN EDIT FILES
  }  
}

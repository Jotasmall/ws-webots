#include "readWriteFiles.h"
// files 
#define FSM 0
#define ESTIMATIONS 1
#define LIFE 2
#define DECISIONS 3
#define PERFORMANCE 4 
#define COMMUNICATION 5
// tasks
#define PICKING 0
#define DROPPING 1
#define NEST 5
#define SOURCE 10
#define IMAGE 201
// To write decisions
#define TRAVELING_AGREE 0
#define TRAVELING_LEVY 1
#define TRAVELING_CALL 2

void createDir(int option, int flagBuild){
  switch(option){
  case FSM:
    sprintf(bot.fileRobot, "%s-FSM", bot.dirPath);
    break;
  case ESTIMATIONS:
    sprintf(bot.fileRobot, "%s-EST", bot.dirPath); 
    break;
  case LIFE:
    sprintf(bot.fileRobot, "%s-REC", bot.dirPath);
    break;
  case DECISIONS:
    sprintf(bot.fileRobot, "%s-DM", bot.dirPath); 
    break;
  case PERFORMANCE:
    sprintf(bot.fileRobot, "%s-OBJ", bot.dirPath);
    break;
  case COMMUNICATION:
    sprintf(bot.fileRobot, "%s-COM", bot.dirPath);
    break;
 }
 //printf("\n fileRobot %s", fileRobot);
 //printf("\n");
 
 //if (flagBuild){ mkdir(fileRobot,0700);} //LINUX
 if (flagBuild){ CreateDirectory(bot.fileRobot, NULL);}  //Windows
  
 strcat(bot.fileRobot, "/");
 strcat(bot.fileRobot, wb_robot_get_name()); 
 
 switch(option){
   case FSM:
     strcat(bot.fileRobot, "-FSM.txt");
     break;   
   case ESTIMATIONS:
     strcat(bot.fileRobot, "-EST.txt");
     break;   
   case LIFE:
     strcat(bot.fileRobot, "-REC.txt");
     break;
   case DECISIONS:
     strcat(bot.fileRobot, "-DM.txt"); 
   break;
   case PERFORMANCE:
     strcat(bot.fileRobot, "-OBJ.txt"); 
     break;
   case COMMUNICATION:
     strcat(bot.fileRobot, "-COM.txt");
     break;
  }
  //printf("\n fileRobot %s", fileRobot);
  //printf("\n"); 
 
  if (flagBuild){
  //printf("\n %s is accessing to %s", robotName, fileRobot);
  }  
}

void createFiles(){ //ok
  // File for each cycle of FSM
  if (bot.flagFilesFSM) {
    createDir(FSM, 1); 
    FILE *ffsm = fopen(bot.fileRobot, "w");
    if (ffsm == NULL) {
      printf("Error opening file bot fsm \n");
      printf("\n");
      exit(1);
    }
    fprintf(ffsm, "StateUML, Suceess/Fail, bot.timeMeasured \n");
    fclose(ffsm);
  }
  // File for evolution of estimations
  if (bot.flagFilesEST) {
    createDir(ESTIMATIONS, 1); 
    FILE *fest = fopen(bot.fileRobot, "w");
    if (fest == NULL) {
      printf("Error opening estimation file \n");
      printf("\n");
      exit(1);
    }
    fprintf(fest,"who, tStore, tHarvest, tPickS, tDropC, "
                 "tePickC, tDropN, WCacheDrop, WCachePick, "
                 "tImage \n");
    fclose(fest);
  }  
  // File for record entire life
  if (bot.flagFilesLIFE) {
    createDir(LIFE, 1); 
    FILE *flife = fopen(bot.fileRobot, "w");
    if (flife == NULL) {
      printf("Error opening estimation file \n");
      printf("\n");
      exit(1);
    }
    fprintf(flife,"what, time \n");
    fclose(flife);
  }  
  // File for decisions
  if (bot.flagFilesDM) {
    createDir(DECISIONS, 1); 
    FILE *fpart = fopen(bot.fileRobot, "w");
    if (fpart == NULL) {
      printf("Error opening partition file \n");
      printf("\n");
      exit(1);
    }
    fprintf(fpart, "Case, decision \n");
    fclose(fpart);
  }  
  // File for performance 
  if (bot.flagFilesPER) {
    createDir(PERFORMANCE, 1);
    FILE *fper = fopen(bot.fileRobot, "w");
    if (fper == NULL) {
      printf("Error opening performance file \n");
      printf("\n");
      exit(1);
    }
    fprintf(fper, "PICK, DROP, HARVEST, STORE\n");
    fclose(fper);
  }  
  // File for communications
  if (bot.flagFilesCOM) {
    createDir(COMMUNICATION, 1);
    FILE *fcom = fopen(bot.fileRobot, "w");
    if (fcom == NULL) {
      printf("Error opening communication file \n");
      printf("\n");
      exit(1);
    }
    fprintf(fcom, "who, message");
    fclose(fcom);
  }
}

void writeDecision(float boundP, float realP, int mechanism, int flagTravel){ //ok-
  if (bot.flagFilesDM) {
    // File for decisions
    createDir(DECISIONS, 0);
    //printf("\n %s is registering its decision in %s", robotName, fileRobot);
    //printf("\n");	
    FILE *file = fopen(bot.fileRobot, "a+");
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

void writeMessage(const char *msg) {
  if (bot.flagFilesCOM) {
    // File for decisions
    createDir(COMMUNICATION, 0);
    //printf("\n %d is registering its messages in %s", bot.botNumber, msg);
    //printf("\n");	
    FILE *file = fopen(bot.fileRobot, "a+");
    if (file == NULL) {
      printf("Error opening file of communications\n");
      printf("\n");
      exit(1);
    }
    //printf("\n %s is updating with %s", robotName, msg);
    fprintf(file, "\n listening, %s", msg);
    //printf("\n %s is updating with %s by listening", robotName, msg);
    fclose(file);
  }
}
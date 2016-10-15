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

//void createDir(int option, int flagBuild, char *fileRobot, char *dirPath){
void createDir(int option, int flagBuild, struct flags4Files *botFlags){
  switch(option){
    case FSM:
      sprintf(botFlags->fileRobot, "%s-FSM", botFlags->dirPath);
      break;
    case ESTIMATIONS:
      sprintf(botFlags->fileRobot, "%s-EST", botFlags->dirPath); 
      break;
    case LIFE:
      sprintf(botFlags->fileRobot, "%s-REC", botFlags->dirPath);
      break;
    case DECISIONS:
      sprintf(botFlags->fileRobot, "%s-DM", botFlags->dirPath); 
      break;
    case PERFORMANCE:
      sprintf(botFlags->fileRobot, "%s-OBJ", botFlags->dirPath);
      break;
    case COMMUNICATION:
      sprintf(botFlags->fileRobot, "%s-COM", botFlags->dirPath);
      break;
   }
   //printf("\n fileRobot %s", fileRobot);
   //printf("\n");
   
   
   //if (flagBuild){ mkdir(fileRobot,0700);} //LINUX
   if (flagBuild){ CreateDirectory(botFlags->fileRobot, NULL);}  //Windows
    
   strcat(botFlags->fileRobot, "/");
   strcat(botFlags->fileRobot, wb_robot_get_name()); 
   
   switch(option){
     case FSM:
       strcat(botFlags->fileRobot, "-FSM.txt");
       break;       
     case ESTIMATIONS:
       strcat(botFlags->fileRobot, "-EST.txt");
       break;       
     case LIFE:
       strcat(botFlags->fileRobot, "-REC.txt");
       break;
     case DECISIONS:
       strcat(botFlags->fileRobot, "-DM.txt"); 
       break;
     case PERFORMANCE:
       strcat(botFlags->fileRobot, "-OBJ.txt"); 
       break;
     case COMMUNICATION:
       strcat(botFlags->fileRobot, "-COM.txt");
       break;
  }
  //printf("\n fileRobot %s", fileRobot);
  //printf("\n"); 
   
  if (flagBuild){
    //printf("\n %s is accessing to %s", robotName, fileRobot);
  }  
}

void createFiles(struct flags4Files *botFlags){ //ok
  // File for each cycle of FSM
  if (botFlags->flagFilesFSM) {
    createDir(FSM, 1, botFlags); 
    FILE *ffsm = fopen(botFlags->fileRobot, "w");
    if (ffsm == NULL) {
      printf("Error opening file bot fsm \n");
      printf("\n");
      exit(1);
    }
    fprintf(ffsm, "StateUML, Suceess/Fail, timeMeasured \n");
    fclose(ffsm);
  }
  // File for evolution of estimations
  if (botFlags->flagFilesEST) {
    createDir(ESTIMATIONS, 1, botFlags); 
    FILE *fest = fopen(botFlags->fileRobot, "w");
    if (fest == NULL) {
      printf("Error opening estimation file \n");
      printf("\n");
      exit(1);
    }
    fprintf(fest,"who, tStore, tHarvest, tPickS, tDropC,"
                 " tePickC, tDropN, WCacheDrop, WCachePick,"
                  "tImage \n");
    fclose(fest);
  }  
  // File for record entire life
  if (botFlags->flagFilesLIFE) {
    createDir(LIFE, 1, botFlags); 
    FILE *flife = fopen(botFlags->fileRobot, "w");
    if (flife == NULL) {
      printf("Error opening estimation file \n");
      printf("\n");
      exit(1);
    }
    fprintf(flife,"what, time \n");
    fclose(flife);
  }  
  // File for decisions
  if (botFlags->flagFilesDM) {
    createDir(DECISIONS, 1, botFlags); 
    FILE *fpart = fopen(botFlags->fileRobot, "w");
    if (fpart == NULL) {
      printf("Error opening partition file \n");
      printf("\n");
      exit(1);
    }
    fprintf(fpart, "Case, decision \n");
    fclose(fpart);
  }  
  // File for performance 
  if (botFlags->flagFilesPER) {
    createDir(PERFORMANCE, 1, botFlags);
    FILE *fper = fopen(botFlags->fileRobot, "w");
    if (fper == NULL) {
      printf("Error opening performance file \n");
      printf("\n");
      exit(1);
    }
    fprintf(fper, "PICK, DROP, HARVEST, STORE\n");
    fclose(fper);
  }  
  // File for communications
  if (botFlags->flagFilesCOM) {
    createDir(COMMUNICATION, 1, botFlags);
    FILE *fcom = fopen(botFlags->fileRobot, "w");
    if (fcom == NULL) {
      printf("Error opening communication file \n");
      printf("\n");
      exit(1);
    }
    fprintf(fcom, "who, message");
    fclose(fcom);
  }
}

//void writeDecision(float boundP, float realP, int mechanism, int flagTravel, struct flags4Files *botFlags, char *fileRobot, char *dirPath ){ //ok-
void writeDecision(float boundP, float realP, int mechanism, int flagTravel, struct flags4Files *botFlags){ //ok-
  if (botFlags->flagFilesDM) {
    // File for decisions
    createDir(DECISIONS, 0, botFlags);
    //printf("\n %s is registering its decision in %s", robotName, fileRobot);
    //printf("\n");	
    FILE *file = fopen(botFlags->fileRobot, "a+");
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


//void writeMessage(int speaking, const char *msg, struct flags4Files *botFlagFiles, char *fileRobot, char *dirPath) {
void writeMessage(int speaking, const char *msg, struct flags4Files *botFlags) {
  if (botFlags->flagFilesCOM) {
      // File for decisions
    createDir(COMMUNICATION, 0, botFlags);
    //printf("\n %s is registering its messages in %s", robotName, fileRobot);
    //printf("\n");	
    FILE *file = fopen(botFlags->fileRobot, "a+");
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
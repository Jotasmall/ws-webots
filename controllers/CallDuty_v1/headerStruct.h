#ifndef SOME_HEADER_GUARD_WITH_UNIQUE_NAME
#define SOME_HEADER_GUARD_WITH_UNIQUE_NAME

#define NB_DIST_SENS 8
#define NB_LEDS 10

typedef struct{
  // model parameters
  int alpha;       	//in percentage
  int flagMomento;  //to enable soft changes
  int beta;         //soft adaptations
  int gammaUCB;     //in UCB-model 100/1000-Explote/Explore
  float greedy;     //in e-Greedy 0.01/0.11-Explote/Explore
  float sParam;     //2011 is 2.5 //2013 is 6/1-Explote/Explore  
  // internal 	
  int botNumber;           //ok - *main
  int currentState;        //ok - initVariables *main
  int suggestedState;      //ok - initVariables *main
  int colorDestination;   //ok - initVariables *main
  int flagLoad;            //ok - initVariables *main 
  int floorColor;          //ok - *main
  int *listFriends;        //ok - *main
  // estimations
  int estPickS;       //ok *initBot
  int estDropN;       //ok *initBot   
  int estTravelGrey;  //ok *initBot 
  int estTravelRed;   //ok *initBot 
  int estTravelBlue;  //ok *initBot
  int lastImage;      //ok *initBot 
  int nPick[3];       //ok *initBot
  int nDrop[3];       //ok *initBot
  int timeMeasured;   //ok *initBot
  int timeListened;   //ok *initBot
  int timeImage;      //ok *initBot
  // devices
  WbDeviceTag cam;                    //ok - reset *initBot
  unsigned short width;               //ok - reset *initBot
  unsigned short height;              //ok - reset *initBot
  const unsigned char *image;         //ok - *main 
  WbDeviceTag receiver;               //ok - reset *initBot   
  WbDeviceTag emitter;                //ok - reset *initBot
  int flagCom;                        //ok - *main
  int flagListened;                   //ok - *main
  int flagCommanded;                  //ok - *main 
  WbDeviceTag sensors[NB_DIST_SENS];  //ok - reset *initBot
  int ps_value[NB_DIST_SENS];         //ok - reset *initBot  
  int ps_offset[NB_DIST_SENS];        //ok - calibrate *initBot       
  WbDeviceTag leds[NB_LEDS];          //ok - reset *initBot
  // flags
  int flagFilesFSM;  //ok
  int flagFilesEST;  //ok
  int flagFilesLIFE; //ok 
  int flagFilesDM;   //ok
  int flagFilesPER;  //ok
  int flagFilesCOM;  //ok
  char *fileRobot;   //ok
  char *dirPath;     //ok 
  // image
  int shapeLooking;  //ok
  int shapeSeen;  
  int colorSeeking;  //ok
  int lineColor;
  int pointA;
  int pointB;
  int nComp;  
}robot;

struct message{
 int source;
 int destination;
 char *msg;
};
	
#endif
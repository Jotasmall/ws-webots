#ifndef SOME_HEADER_GUARD_WITH_UNIQUE_NAME
#define SOME_HEADER_GUARD_WITH_UNIQUE_NAME

#define NB_DIST_SENS 8
#define NB_LEDS 10

struct robotDevices{ 
  int flagCom;
  int flagListened;
  int ps_value[NB_DIST_SENS];
  int ps_offset[NB_DIST_SENS];
  unsigned short width;
  unsigned short height;
  WbDeviceTag cam;
  WbDeviceTag receiver;
  WbDeviceTag emitter;
  WbDeviceTag sensors[NB_DIST_SENS];  
  WbDeviceTag leds[NB_LEDS];
};

struct robotState{
  int robotNumber;
  int currentState;
  int flagLoad;
  int floorColor;
  int *listFriends;
  int travelDestination;
};

struct robotEstimations{
  int estPickS;
  int estDropN;
  int estTravelGrey;
  int estTravelRed;
  int estTravelBlue;
  int lastImage;
  int nPick[3];
  int nDrop[3];
};

struct message{
 int source;
 int destination;
 char *msg;
};

struct flags4Files{
	int flagFilesFSM;
	int flagFilesEST;
	int flagFilesLIFE;
	int flagFilesDM;
	int flagFilesPER;
	int flagFilesCOM;
	char *fileRobot;
    char *dirPath;
};
	
#endif
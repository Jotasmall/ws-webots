#ifndef SOME_HEADER_GUARD_WITH_UNIQUE_NAME
#define SOME_HEADER_GUARD_WITH_UNIQUE_NAME

struct robotState{
	int robotNumber;
	int currentState;
	int floorColor;
	int travelDestination;
	int nPick[3];
	int nDrop[3];
};

struct robotCom{ 
  int flagCom;
  int flagListened;
  WbDeviceTag receiver;
  WbDeviceTag emitter;
};

struct robotImage{
  int color;
  int shapeLooking;
};

struct robotEstimations{
	int estPickS;
	int estDropN;
	int estTravelGrey;
	int estTravelRed;
	int estTravelBlue;
	int lastImage;
};

struct a{
  int i;
  int j;
};

#endif
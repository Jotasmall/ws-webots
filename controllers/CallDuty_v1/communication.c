#include "communication.h"

#define NROBOTS 10

#define RED 0
#define TRAVEL2RED 104
#define GREY 1
#define TRAVEL2GREY 105
#define BLUE 2
#define TRAVEL2BLUE 106

#define LEAVE 11
#define COME 12
#define ROBOT_LEAVING 31
#define ROBOT_ARRIVING 32
#define ROBOT_UPDATING 33
#define ROBOT_NEGATIVE 34
// To write decisions
#define TRAVELING_AGREE 0
#define TRAVELING_LEVY 1
#define TRAVELING_CALL 2
// destinations
#define M2ROBOT 1
#define M2NEST 2


int listening(){ //ok-
  int i;
  while(wb_receiver_get_queue_length(bot.receiver)>0){  
  const char *data = wb_receiver_get_data(bot.receiver);
  if (data[0] == 'U') {
    //printf("\n %d will update its state of partitioning %d", bot.botNumber, flagTravel);
    writeDecision(0, 0, TRAVELING_CALL, 0);
    wb_receiver_next_packet(bot.receiver);
  } else if ((data[0] == 'T') && (data[2] == 'R')) {
    // "T2R0R0000T77X9"
    int place = atoi(&data[3]);
    //printf("\n %d has received %s message from %d", bot.botNumber, data, place);
    //printf("\n");
    if (place == bot.floorColor) {
      int destinatary = atoi(&data[5]);
      if (destinatary == bot.botNumber){ 
        //l printf("\n %d is listening its nest location %d to say %s", bot.botNumber, place, data);
        //l printf("\n");
        writeMessage(0, data);
        int newFriend = atoi(&data[10]);
        int suggestedDestination = atoi(&data[13]);
        if (newFriend == LEAVE) {
          //l printf("\n Nest %d is asking for %d to leave and go %d", place, bot.botNumber, suggestedDestination);
          //l printf("\n");
        if (bot.botNumber != 2701) {    
          switch(suggestedDestination){
          case RED:
		    if (bot.floorColor != RED) {
              printf("\n %d do not wanna go RED", bot.botNumber);
              bot.currentState = TRAVEL2RED;
              bot.suggestedState = TRAVEL2RED;
			}  
            break;
          case GREY:
            if (bot.floorColor != GREY) {
			  printf("\n %d do not wanna go GREY", bot.botNumber);
              bot.currentState = TRAVEL2GREY;
              bot.suggestedState = TRAVEL2GREY;
			}  
            break;
          case BLUE:
            if (bot.floorColor != BLUE) {
			  printf("\n %d do not wanna go BLUE", bot.botNumber);
              bot.currentState = TRAVEL2BLUE;
              bot.suggestedState = TRAVEL2BLUE;
			}  
            break;
          }
		  bot.flagCommanded = 1;
          printf("\n");  
		  wb_robot_step(32);
        } else {
          if ((bot.flagLoad) || (bot.botNumber == 2701)) {
            speaking(M2NEST, ROBOT_NEGATIVE, 0, 0);
		  }	
		}
      } else if (newFriend == COME) {
        printf("\n Nest %d is asking for %d to arrive", place, bot.botNumber);
        printf("\n");
      } else {
        int flagNew = 1, pos = NROBOTS; 
        for (i = 0; i < NROBOTS; i++) { 
          if (newFriend == bot.listFriends[i]) {
            flagNew = 0; // it already exists
            break;
          }
          if (bot.listFriends[i] == 0) {
            if (pos > i) {
              pos = i; // to fill the empty spaces
            }
          }
        }
        if (flagNew) {
          bot.listFriends[pos] = newFriend;
          //printf("\n %s added to its friends %d", bot.botNumber, newFriend);
          //printf("\n");
        }
      }  
    }  
  }
  wb_receiver_next_packet(bot.receiver);
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
    for (i = 0; i < NROBOTS; i++) {
      if (name == bot.listFriends[i]) {
        writeMessage(0, data);
        if (codeReceived == ROBOT_LEAVING) {
          printf("\n %d bye bye %d", bot.botNumber, name);
          printf("\n");
          bot.listFriends[i] = 0;
        } else {
          // proceed to listen the information
          bot.timeListened = atoi(&data[12]);
          /*
            Robot codes 
            301 TRIANGLE   100 timePickSource   106 timeStore
            302 BOX    101 timepickCache    107 timeHarvest
            303 CIRCLE   102 timeDropCache    201 bot.timeImage
            304 ALL    103 timeDropNest   202 timeCache
            305 NOTHING  104 timeTravel2Nest  777 waiting
            306 ROBOT    105 timeTravel2Source
          */
          if ((codeReceived >= 100) && (codeReceived <= 107)){	
            //printf("\n Thanks %d buddy, %s will consider your estimations for %d of time %d", name, bot.botNumber, codeReceived, bot.timeListened);
            //printf("\n");
            bot.flagListened = 1;
            wb_robot_step(32); // to update global values
            updateEstimations(codeReceived, 0);
          }
        }
        break;
      } /*else {
        printf("\n %s receive a message from %d, but it is not for him %s", bot.botNumber, name, data);
        printf("\n");
      }*/
    }
    wb_receiver_next_packet(bot.receiver);
    } else {
     //printf("\n %s receive a message no for him %s", bot.botNumber, data);
      //printf("\n");
      wb_receiver_next_packet(bot.receiver);
    }
  }	
  return 1;
}

int speaking(int toWhom, int codeTask, int time, int cache){ //ok-
  if (bot.flagCom == 0) { return 0;}
  char message[30];

  // wb_emitter_set_channel(emitter, WB_CHANNEL_BROADCAST);
  if (toWhom == M2ROBOT) {
    if (time == -1) { // reporting just to have the same number of lines
      sprintf(message, "U");
    } else if (toWhom == -1){ 
      sprintf(message, "R2R%dR%d",bot.botNumber, ROBOT_LEAVING);
    } else {
      sprintf(message, "R2R%dC%dT%d",bot.botNumber, codeTask, time);
    }
  } else if (toWhom == M2NEST) {
    if (time == -1) {
      printf("\n %d will update your estimation NEST %d", bot.botNumber, bot.floorColor);
    } else {
      if (codeTask == ROBOT_LEAVING) {
        sprintf(message,"R2T%dT%dX%d",bot.botNumber, ROBOT_LEAVING, bot.floorColor);
        //s printf("\n %d is leaving NEST %d", bot.botNumber, bot.floorColor);
        //s printf(" message %s", message);
        printf("\n");
      } else if (codeTask == ROBOT_ARRIVING) {
        sprintf(message,"R2T%dT%dX%d",bot.botNumber, ROBOT_ARRIVING, bot.floorColor);
        printf("\n %d is arriving into NEST %d", bot.botNumber, bot.floorColor);
        printf(" message %s", message);    
        printf("\n");
      } else if (codeTask == ROBOT_UPDATING) {
        sprintf(message,"R2T%dT%dX%d",bot.botNumber, ROBOT_UPDATING, bot.estPickS + bot.estDropN);
        printf("\n %d is updating NEST %d", bot.botNumber, bot.floorColor);
        printf(" message %s", message);    
        printf("\n");
      } else if (codeTask == ROBOT_NEGATIVE) {
		sprintf(message,"R2T%dT%dX%d",bot.botNumber, ROBOT_NEGATIVE, bot.floorColor);
        printf("\n %d said *negative sir* to nest %d commands", bot.botNumber, bot.floorColor);
        printf(" message %s", message);    
        printf("\n"); 
	  }
    }
  } 
  if (strcmp(message, "U")) {
    //s printf("\n %d updating its record of messages", bot.botNumber);
    writeMessage(1, message);
  }  
  wb_emitter_send(bot.emitter, message, strlen(message)+1);
  wb_robot_step(32);
  return 1;
  
}
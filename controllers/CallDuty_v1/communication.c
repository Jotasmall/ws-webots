#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "communication.h"

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
#define nRobots 10

int listening(WbDeviceTag receiver, int floorColor, int botNumber, int *listFriends, int *stateUML, int *suggestedState){ //ok-
  int i;
  while(wb_receiver_get_queue_length(receiver)>0){  
    const char *data = wb_receiver_get_data(receiver);
    if (data[0] == 'U') {
      //printf("\n %d will update its state of partitioning %d", botNumber, flagTravel);
      //c writeDecision(0, 0, TRAVELING_CALL);
      wb_receiver_next_packet(receiver);
    } else if ((data[0] == 'T') && (data[2] == 'R')) {
      // "T2R0R0000T77X9"
      int place = atoi(&data[3]);
      //printf("\n %d has received %s message from %d", botNumber, data, place);
      //printf("\n");
      if (place == floorColor) {
        int destinatary = atoi(&data[5]);
        if (destinatary == botNumber){ 
          printf("\n %d is listening its nest location %d to say %s", botNumber, place, data);
          printf("\n");
          //c writeMessage(0, data);
          int newFriend = atoi(&data[10]);
          int suggestedDestination = atoi(&data[13]);
          if (newFriend == LEAVE) {
            printf("\n Nest %d is asking for %d to leave and go %d", place, botNumber, suggestedDestination);
            printf("\n");
            if (botNumber != 2701) {          
              switch(suggestedDestination){
                case RED:
                   printf("\n %d do not wanna go RED", botNumber);
                   *stateUML = TRAVEL2RED;
                   *suggestedState = TRAVEL2RED;
                   break;
                case GREY:
                   printf("\n %d do not wanna go GREY", botNumber);
                   *stateUML = TRAVEL2GREY;
                   *suggestedState = TRAVEL2GREY;
                   break;
                case BLUE:
                   printf("\n %d do not wanna go BLUE", botNumber);
                   *stateUML = TRAVEL2BLUE;
                   *suggestedState = TRAVEL2BLUE;
                   break;
              }
              printf("\n");  
            }  
          } else if (newFriend == COME) {
            printf("\n Nest %d is asking for %d to arrive", place, botNumber);
            printf("\n");
          } else {
            int flagNew = 1, pos = nRobots; 
            //-- printf("\n %d was introduced to %d", botNumber, newFriend);
            //-- printf("\n");
            for (i = 0; i < nRobots; i++) { 
              if (newFriend == listFriends[i]) {
                flagNew = 0; // it already exists
                break;
              }
              if (listFriends[i] == 0) {
                if (pos > i) {
                  pos = i; // to fill the empty spaces
                }
              }
            }
            if (flagNew) {
              listFriends[pos] = newFriend;
              //printf("\n %s added to its friends %d", botNumber, newFriend);
              //printf("\n");
            }
          }  
        }    
      }
      wb_receiver_next_packet(receiver);
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
      for (i = 0; i < nRobots; i++) {
        if (name == listFriends[i]) {
          //c writeMessage(0, data);
          if (codeReceived == ROBOT_LEAVING) {
            printf("\n %d bye bye %d", botNumber, name);
            printf("\n");
            listFriends[i] = 0;
          } else {
            // proceed to listen the information
            int timeListened = atoi(&data[12]);
            /* Robot codes 
            301 TRIANGLE     100 timePickSource       106 timeStore
            302 BOX          101 timepickCache        107 timeHarvest
            303 CIRCLE       102 timeDropCache        201 timeImage
            304 ALL          103 timeDropNest         202 timeCache
            305 NOTHING      104 timeTravel2Nest      777 waiting
            306 ROBOT        105 timeTravel2Source
            */
            if ((codeReceived >= 100) && (codeReceived <= 107)){	
              //printf("\n Thanks %d buddy, %s will consider your estimations for %d of time %d", name, botNumber, codeReceived, timeListened);
              //printf("\n");
              //c flagListened = 1;
              wb_robot_step(32); // to update global values
              //updateEstimations(codeReceived, timeListened, 0);
            }
          }
          break;
        } /*else {
          printf("\n %s receive a message from %d, but it is not for him %s", botNumber, name, data);
          printf("\n");
        }*/
      }
      wb_receiver_next_packet(receiver);
    } else {
      //printf("\n %s receive a message no for him %s", botNumber, data);
      //printf("\n");
      wb_receiver_next_packet(receiver);
    }
  }  
  return 1;
}
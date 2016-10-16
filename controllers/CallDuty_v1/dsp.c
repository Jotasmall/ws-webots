#include "dsp.h"
// Colors
#define RED 0
#define GREY 1
#define BLUE 2
#define CYAN 3
#define MAGENTA 4
#define BLACK 6
#define GREEN 7
#define WHITE 8
#define TAM_WALL 9
#define ROBOT_COLOR 10

#define COLOR_THRES 150
#define COMP_COLOR 80
#define ROBOT_THRES 100
#define BLACK_THRES 10
#define LOW_THRES 50

#define PICK_SOURCE 100
#define DROP_NEST 103
#define TRAVEL2RED 104
#define TRAVEL2GREY 105
#define TRAVEL2BLUE 106

#define COMP_LOW 38
#define COMP_LOWDARK 34
#define COMP_HIGH 200

int compareColorPixel(struct robotCamera *botCam, const unsigned char *image, int pixelX, int pixelY, int foreground, struct robotState *botState){ //ok-
  int auxColor = 0;
  //int width = botCam->width;
  int pixelR = wb_camera_image_get_red(botCam->image, botCam->width, pixelX, pixelY);
  int pixelG = wb_camera_image_get_green(botCam->image, botCam->width, pixelX, pixelY);
  int pixelB = wb_camera_image_get_blue(botCam->image, botCam->width, pixelX, pixelY);
  
  if ((foreground == CYAN) && (botState->floorColor == GREY)) { foreground = WHITE;}
  
  switch(foreground){
    case RED:
      auxColor = (pixelR > COLOR_THRES) && (pixelB < 20) && (pixelG < 20); //only red
      break;
    case GREEN:
      auxColor = (pixelG > COLOR_THRES) && (pixelB < LOW_THRES) && (pixelR < LOW_THRES); //only green
      break;  
    case BLUE:
      auxColor = (pixelB > COLOR_THRES) && (pixelR < LOW_THRES) && (pixelG < LOW_THRES); //only blue  
      break;
    case CYAN:     // green+blue
      auxColor = (pixelG > LOW_THRES) && (pixelB > LOW_THRES) && (pixelR < 20);
      break;
    case MAGENTA:  // red+blue
      auxColor = (pixelR > COMP_COLOR) && (pixelB > COMP_COLOR) && (pixelG < LOW_THRES);
      break;
    case BLACK:
      auxColor = (pixelR < BLACK_THRES) && (pixelG < BLACK_THRES) && (pixelB < BLACK_THRES);
      break;
    case ROBOT_COLOR:   
      auxColor = (pixelR < COMP_LOW) && (pixelG < COMP_LOW) && (pixelB < COMP_LOW); 
      auxColor = auxColor && ((pixelR > COMP_LOWDARK) && (pixelG > COMP_LOWDARK) && (pixelB > COMP_LOWDARK));
      auxColor = auxColor || ((pixelR > ROBOT_THRES) && (pixelR < COMP_HIGH) && (pixelG > ROBOT_THRES) && (pixelG < COMP_HIGH) && (pixelB > ROBOT_THRES) &&  (pixelB < COMP_HIGH));
      break;
    case GREY:
      auxColor = (pixelR < COMP_COLOR) && (pixelG < COMP_COLOR) && (pixelB < COMP_COLOR);
      auxColor = auxColor && ((pixelR > BLACK_THRES) && (pixelG > BLACK_THRES) && (pixelB > BLACK_THRES));	  
      break;	
    case WHITE:
      auxColor = (pixelR > COLOR_THRES) && (pixelB > COLOR_THRES) && (pixelG > COLOR_THRES);  
      break;
    case TAM_WALL:
      auxColor = (pixelR < LOW_THRES) && (pixelG < LOW_THRES) && (pixelB < LOW_THRES); 
      break;  
    default:
      auxColor =  0;
  }
  return auxColor;
}

int cont_height_figure(int indexP, int color, struct robotCamera *botCam, struct robotState *botState){ //ok
  int count=0;
  int maxCount = 0;
  int beginY = 0;
  int endX = botCam->width-1;
  int foreground = color;
  int i, j;
  switch (indexP){
    case -22:
      beginY = botCam->height - 5;
      foreground = GREY; break;
    case -21: // checking red-nest ground color
      beginY = botCam->height - 5;
      foreground = RED; break;
    case -20: // checking blue-source ground color
      beginY = botCam->height - 5; 
      foreground = BLUE; break; 
    case -11: // looking for landmark
      foreground = MAGENTA; // nest TAM
      break; 
    case -10: // On levy avoid colors 18 
      foreground = MAGENTA;
      if (botState->currentState == DROP_NEST) { 
        foreground = RED; // source TAM
        if (botState->floorColor == RED) { foreground = BLUE;} // source TAM
      } break; 
    case 100: // checking tam_wall color
      foreground = TAM_WALL; break;
    case 101: // waiting on TAM 
      if ((botState->currentState == TRAVEL2BLUE) || (botState->currentState == TRAVEL2RED) || (botState->currentState == TRAVEL2GREY)){
        foreground = CYAN;
      } else {
        if (foreground != BLACK) { foreground = WHITE;}
      } break;
    case 102: // checking for sources 
      if (botState->currentState == PICK_SOURCE) {
        foreground = RED;
        if (botState->floorColor == RED) {
          foreground = BLUE;
        }
      } break;
    default:  // Normal processing
      if ((indexP >= 0) && (indexP < botCam->width)) { endX = 0;}
  } 

  for (i = 0; i <= endX; i++) {
    if (endX == 0) { i = indexP;} // Only that point
    for (j = beginY; j < botCam->height; j++) {
      count += compareColorPixel(botCam, botCam->image, i, j, foreground, botState);  
    } 
    if (count > maxCount) { maxCount = count;}
    if (beginY != (botCam->height - 5)) { count = 0;}
  } 
  // printf("\n count value %d index %d", maxCount, i);
  return maxCount;
}



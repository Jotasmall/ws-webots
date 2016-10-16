// Colors
#define RED 0
#define GREY 1
#define BLUE 2
#define CYAN 3
#define MAGENTA 4
#define BLACK 6
#include "dsp.h"

#define GREEN 7
#define WHITE 8
#define TAM_WALL 9
#define ROBOT_COLOR 10

#define COLOR_THRES 150
#define COMP_COLOR 80
#define ROBOT_THRES 100
#define BLACK_THRES 10
#define LOW_THRES 50

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


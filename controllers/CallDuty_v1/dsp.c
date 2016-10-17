#include "dsp.h"

#define TIME_STEP 64
#define TURN_CACHE -52
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
// display Colors
#define HEXRED 0xFF0000
#define HEXWHITE 0xFFFFFF
#define HEXBLACK 0x000000
#define HEXYELLOW 0xFFFF00
#define HEXGREEN 0x2AE246
// Color thresholds
#define COLOR_THRES 150
#define COMP_COLOR 80
#define COMP_LOW 38
#define COMP_LOWDARK 34
#define COMP_HIGH 200
#define ROBOT_THRES 100
#define BLACK_THRES 10
#define LOW_THRES 50
// Figures
#define BOX 301
#define TRIANGLE 302
#define CIRCLE 303
#define ALL 304
#define NOTHING 305
#define ROBOT 306
// Activities
#define PICK_SOURCE 100
#define DROP_NEST 103
#define TRAVEL2RED 104
#define TRAVEL2GREY 105
#define TRAVEL2BLUE 106
// Messages
#define M2NEST 2
#define ROBOT_LEAVING 31
#define ROBOT_ARRIVING 32

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

int whereIam(int avoiding, int color, double *speed, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState){ 
  botCam->image = wb_camera_get_image(botDevices->cam);
  wb_robot_step(TIME_STEP);
  int groundDetected = GREY;
  // cronometer(IMAGE, 0); //This is a fast operation
  
  if (cont_height_figure(-20, color, botCam, botState) > 104 ) { 
    groundDetected = BLUE;
  } else if (cont_height_figure(-21, color, botCam, botState) > 104 ) {
    groundDetected = RED;
  } else if (cont_height_figure(-22, color, botCam, botState) > 104) {
    groundDetected = GREY;
  }
  if ((avoiding) && (groundDetected != botState->floorColor)) {
    float p = ((float)rand())/RAND_MAX; //-- JUAN EDITED
    if (p>0.5) {p = 1;} else { p = -1;} //-- JUAN EDITED
    turnSteps((int) p*TURN_CACHE/2, speed);    //-- JUAN EDITED
    run(botState->flagLoad, 5, speed, botDevices);//7
	whereIam(1, color, speed, botCam, botDevices, botState);
	run(botState->flagLoad, 5, speed, botDevices);
	whereIam(1, color, speed, botCam, botDevices, botState);
    //printf("\n Missing my region %s", robotName);
    //printf("\n");
  }    
  return groundDetected;
} 

int whereArrive(int color, double *speed, struct robotDevices *botDevices, struct robotCamera *botCam, struct robotState *botState, struct flags4Files *botFlags){
    // Verify if not robot is close
    if ((readSensors(0, botDevices) == 0)) {//check && (check4Robot() == 0)) {
      botState->floorColor = whereIam(0, color, speed, botCam, botDevices, botState);
      printf("\n %d arrived into a land of color %d", botState->botNumber, botState->floorColor);
      printf("\n");
      speaking(botDevices, botState->botNumber, M2NEST, ROBOT_ARRIVING, 0, 0, botFlags);
    } else {
      waiting(10);
      printf("\n Waiting to have a clear ground");
      printf("\n");
      return whereArrive(color, speed, botDevices, botCam, botState, botFlags);
    }
    return 1;
}


int find_middle(int wrongLine, int colorLine, struct robotCamera *botCam, struct robotState *botState){ //ok 
  int i;
  int aux, index1 = -1, index2 = -1;
  int foreground = colorLine;
  if (wrongLine) { 
    if (foreground == BLUE) {
      foreground = RED;
    } else {
      foreground = BLUE;
    }
  }
  // new world
  for (i = 0; i<botCam->width; i++){
    aux = compareColorPixel(botCam, botCam->image, i, botCam->height-1, foreground, botState);
    if (aux == 1) {
      if (index1 == -1) { // the 1st time see the color
        index1 = i;
      } else { // the final index where the color is seen
        index2 = i;
      }  
    }
  }  
  if (index1 == -1) { return -1;} // followLine
  aux = (index2-index1)/2+index1;
  if (wrongLine) {
    aux = 100;
    printf("\n %d had found a wrong line color", botState->botNumber);
    printf("\n");
  }
  return aux;    
}

int waiting_color(int foreground, int color, struct robotState *botState, struct robotCamera *botCam, struct robotDevices *botDevices) {//ok
  botCam->image = wb_camera_get_image(botDevices->cam);
  waiting(1);
  int count = 0;
  count = cont_height_figure(101, color, botCam, botState);
  int countArriving = 0;
  int flagPrint1=0;
  countArriving = cont_height_figure(102, color, botCam, botState);
  if (flagPrint1) {
    if (count > countArriving) {
      //printf("\n Intensity %d half line", count);
    } else {
      //printf("\n Intensity %d half line", countArriving);  
    }
  
    flagPrint1 = 0;
  } 
  if ((count > 26) || (countArriving > 26)) {
    if (botState->currentState == PICK_SOURCE) {
      //check cronometer(WAITING, 0); //shapeSeen); //when using different shapes
    } 
    //check cronometer(-1, 0);
    return 1; //keep waiting
  }    
  //printf("\n Intensity gets down");
  return 0; //wait no longer
}

int whatIsee(int color, float Eccentricity, float Extent, int squarewidth, int middleAxisH, int middleAxisV, int numImage){
    // 1 Triangle, 2 Box, 3 Circle, 4 Nothing, 5 All, 6 Robot, -1 ReallyNothing
    int shapeFound = -1; // Weka 3rd generation 16feb16
    if (Eccentricity <= 1.2) {
      if (Extent <= 0.889) {
        if (Extent <= 0.711) {
          if (squarewidth <= 11) {
            //printf("\n Circle (4.0/1.0)");
            shapeFound = CIRCLE;
          } else {
            //printf("\n Robot (176.0)");
            shapeFound = ROBOT;
          }
        } else {
          if (Eccentricity <= 0.818) {
            //printf("\n Robot (6.0)");
            shapeFound = ROBOT;
          } else {
            //printf("\n Circle (135.0/1.0)");
            shapeFound = CIRCLE;
          }
        }
      } else {
        //printf("\n Box (132.0)");
        shapeFound = BOX;
      }
    } else {
      if (Extent <= 0.708) { 
        if (Extent <= 0.474) {
          //printf("\n Robot (20.0/1.0)");
          shapeFound = ROBOT;
        } else {
          if (Eccentricity <= 2.455) {
            if (Extent <= 0.613) {
              if (middleAxisH <= 3) {
                //printf("\n No triangle (4.0)");
                shapeFound = NOTHING;
              } else {
                //printf("\n Triangle (99.0/3.0)");
                shapeFound = TRIANGLE;
              } 
            } else {
              if (squarewidth <= 7) {
                if (middleAxisV <= 7) {
                  //printf("\n Triangle (15.0)");
                  shapeFound = TRIANGLE;
                } else {
                  //printf("\n No circle (5.0)");
                  shapeFound = NOTHING;
                }
              } else {
                //printf("\n No triangle (34.0)");
                shapeFound = NOTHING;
              }
            }
          } else {
            //printf("\n No triangle (35.0/1.0)");
            shapeFound = NOTHING;
          }
        }
      } else {
        //printf("\n No circle (105.0)");
        shapeFound = NOTHING;
        if (color == CYAN ){ shapeFound = BOX;}
      }    
    }  
    return shapeFound;
}

int doubleCheck(double *speed, WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, int color, int foreground, int shape, int numImage, int *numberComponents, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState){
  int index = -1;
  int nComp;
  run(botState->flagLoad, 5, speed, botDevices); //forward(5);
  whereIam(1, color, speed, botCam, botDevices, botState);
  run(botState->flagLoad, 5, speed, botDevices);
  whereIam(1, color, speed, botCam, botDevices, botState);
  index = detectImage(displayExtra, shapeSeen, pointA, pointB, color, color, shape, 1, &nComp, botCam, botDevices, botState);
  if ((index == -1) || (index == 100)){
    printf("\n False alarm %d - %d continue searching", index, botState->botNumber);
    printf("\n");
    return -1;
  } 
  //printf("\n Shape %d watched on 2check", figura);
  //printf("\n");
  return index; 
}

int check4Robot(WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, int color, int foreground, int shape, int numImage, int *numberComponents, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState){ //ok
  int nComp, sizeRobot = 0;
  sizeRobot = detectImage(displayExtra, shapeSeen, pointA, pointB, color, ROBOT_COLOR, ROBOT, 0, &nComp, botCam, botDevices, botState);
  if (((sizeRobot > 9) && (nComp > 1)) || ((sizeRobot > 4) && (nComp > 3))) {//4 3
    printf("\n %d sees a robot of height %d components %d", botState->botNumber, sizeRobot, nComp);
    return 1;
  }
  return 0;
}


int detectImage(WbDeviceTag *displayExtra, int *shapeSeen, int *pointA, int *pointB, int color, int foreground, int shape, int numImage, int *numberComponents, struct robotCamera *botCam, struct robotDevices *botDevices, struct robotState *botState){ //ok
  int flagSeen = -1;
  int middleH = -1;
  int aux = 0;
  int newShapeSeen = 0;
  int distMiddle = 0;
  int maxProx = 0;
  int comp = 1;
  int nearest = 100;
  int realComp = 0;
  int i, j, k;
  int left, up;
  int minV = botCam->height, maxV = 0, minH = botCam->width, maxH = 0, area = 0;
  
  int imaComp[botCam->width][botCam->height];
  memset(imaComp, -1, botCam->width*botCam->height*sizeof(int));
  int relations[40];
  memset(relations, 0, 40*sizeof(int));
  int check[20];
  memset(check, 0, 20*sizeof(int));

  botCam->image = wb_camera_get_image(botDevices->cam);
  wb_robot_step(TIME_STEP);
  //check cronometer(IMAGE, 0); // for image processing

  // Segmentation process
  for (i = 0; i < botCam->width; i++) {
    for (j = 0; j < botCam->height; j++) {
      aux = compareColorPixel(botCam, botCam->image, i, j, foreground, botState); 
      if (aux){    
        // Identifying component through a N-neighborhood strategy
        left = i-1; 
        if (left < 0) { left = 0;}      
        up = j-1;
        if (up < 0) { up = 0;}
        // Case 1 - new 
        if ((imaComp[i][up] == -1) && (imaComp[left][j] == -1)){
          imaComp[i][j] = comp;
          relations[comp] = comp;
          comp++;
        } else {
          if (imaComp[left][j] > 0) {
            imaComp[i][j] = imaComp[left][j]; //Case 2
            if (imaComp[i][up] > 0) {
              relations[imaComp[i][up]] = imaComp[left][j];
              imaComp[i][up] = imaComp[left][j]; // save E2 = E1
            }
          } else {
            imaComp[i][j] = imaComp[i][up]; //Case 2
          }
        }
        wb_display_set_color((WbDeviceTag)*displayExtra, HEXWHITE);
      } else {
        wb_display_set_color((WbDeviceTag)*displayExtra, HEXBLACK);
      }
      wb_display_draw_pixel((WbDeviceTag)*displayExtra, i, j);
    }
  }
  // Replacing overlapping in components
  aux = 0;
  for (k = comp; 0 < k; k--) {
    if (relations[k] != k) {
      for (i = 0; i < botCam->width; i++) {
        for (j = 0; j < botCam->height; j++) {
          if (imaComp[i][j] == k) {
            imaComp[i][j] = relations[k];
          }
        }
      }
      comp--;
    } else {
      check[aux]=relations[k];
      aux++;
    }  
  }
  comp = aux;
  //  *numberComponents = comp; 
  //  FILE *fp4 = fopen("image_descritors_4n.csv","a");  
  for (k = 0; k < comp; k++) {
    // reset values to find them in a new component
    minV = botCam->height; maxV = 0; minH = botCam->width; maxH = 0; area = 0; 
    for (i = 0; i < botCam->width; i++) {
      for (j = 0; j < botCam->height; j++) {
        // If the pixel has the same component
        if (imaComp[i][j] == check[k]) {
          area++;
          // Checking boundaries
          if (maxH < i) { maxH = i;}
          if (minH > i) { minH = i;}
          if (maxV < j) { maxV = j;}
          if (minV > j) { minV = j;}
        } 
      }
    }
    if ((numImage == 255) && (color == CYAN)) {
      *pointA = cont_height_figure(minH+1, color, botCam, botState); 
      *pointB = cont_height_figure(maxH-1, color, botCam, botState);
      //printf("\n %s really close and sure it is not a robot, go for the center", robotName);
      //printf("\n");
      return 100;
    }
//    if (((area > 10) && (foreground != CYAN)) || ((foreground == CYAN) && (area > 15))) { 
    if (((area > 10) && (foreground != CYAN)) || ((foreground == CYAN) && (area > 25))) { 
      int squarewidth = maxH-minH+1;
      int squareHeight = maxV-minV+1;  
      // Middle axis width within the square
      int middleAxisH = 0;
      aux = (int)squareHeight/2+minV;
      for (i = minH; i <= maxH; i++) {
        if (imaComp[i][aux] > 0) {
          middleAxisH++;  
        } 
      }
      wb_display_set_color((WbDeviceTag)*displayExtra, HEXRED);
      wb_display_draw_line((WbDeviceTag)*displayExtra, minH, aux, maxH, aux); 
      // Middle axis height within the square
      int middleAxisV = 0;
      aux = (int)squarewidth/2+minH;
      for (i = minV; i <= maxV; i++) {
        if (imaComp[aux][i] > 0) {
          middleAxisV++;
        }
      }
      wb_display_set_color((WbDeviceTag)*displayExtra, HEXRED);
      wb_display_draw_line((WbDeviceTag)*displayExtra, aux, minV, aux, maxV); 
      int x = aux; //middle index horizontal 
      int areaSquare = squarewidth * squareHeight;   
      float extent = (float) area/areaSquare;
      float eccentricity = (float) middleAxisV/middleAxisH;
      // Increase padding of 1 for window of component
      if (minV > 0) { minV--;}
      if (minH > 0) { minH--;}
      wb_display_set_color((WbDeviceTag)*displayExtra, HEXYELLOW);
      wb_display_draw_rectangle((WbDeviceTag)*displayExtra, minH, minV, squarewidth+1, squareHeight+1);
      // return the horizontal position as delta value
      distMiddle = abs(botCam->width/2-x);
      realComp++;
      *numberComponents = realComp;
      // A great enough region
      if ((squarewidth >= 4) && (squareHeight >= 4)) {
        //1 Triangle, 2 Box, 3 Circle, 4 Nothing, 0 ReallyNothing, 5 All, 6 Robot
         newShapeSeen = whatIsee(color, eccentricity, extent, squarewidth, middleAxisH, middleAxisV, numImage);
         if (shape == ROBOT){
           if (newShapeSeen == ROBOT) {
             if ((x > 23) && (x < 29) && (areaSquare > 600)) { waiting(15);} 
             return squareHeight; //only returned when checkRobot is used  
           } 
         } else {
           switch(newShapeSeen){
             case NOTHING:
               //last value to check and nothing was seen clearly
               if ((flagSeen == -1) && (k == comp-1) && (squareHeight < 15)) { 
                 *numberComponents = 1; 
                 return 100;
               } break;
             case TRIANGLE:
             case CIRCLE:
             case BOX:
               if ((shape == ALL) || (shape == newShapeSeen)) { 
                 flagSeen = 1;
               } else if (k == comp-1){
                 return 100;
               }
               //if in k component was seen, then check if it is better
               if (flagSeen == 1) { 
                 if (maxProx < middleAxisH) {
                   nearest = distMiddle;
                   maxProx = middleAxisH;
                   middleH = x;
                   *shapeSeen = newShapeSeen;
                   //printf("\n Robot %s found a new shape %d higher %d and closer to the middle %d", robotName, newShapeSeen, maxProx, middleH);
                 } else if (maxProx == middleAxisH) {
                   if (nearest > distMiddle) {
                     nearest = distMiddle;
                     middleH = x;
                     *shapeSeen = newShapeSeen;
                     //printf("\n Robot %s found a new shape %d just closer to the middle %d", robotName, newShapeSeen, middleH);
                   }
                 }
                 flagSeen = 0;
               }              
               *numberComponents = realComp; 
               break;
           }  
        }
      }
    }  
  }
  *numberComponents = realComp;
  return middleH; 
}      

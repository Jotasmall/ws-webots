#ifndef INITBOT_H
#define INITBOT_H

void initSensors(WbDeviceTag *sensors);
void initLeds(WbDeviceTag *leds);
void initCamera(WbDeviceTag *cam, unsigned short *width, unsigned short *height);
void resetDisplay(WbDeviceTag *displayExtra, unsigned short width, unsigned short height);
void calibrateSensors(WbDeviceTag *sensors, int *ps_offset);

#endif
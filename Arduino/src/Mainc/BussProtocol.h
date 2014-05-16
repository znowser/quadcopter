#ifndef BUSS_PROTOCOL_H
#define BUSS_PROTOCOL_H

#include "sensorDataStruct.h"

//Protocol definitions
#define TEXT_PACKAGE 0x01
#define PS3_CONTROLLER_PACKAGE 0x02
#define EMERGENCY_STOP_PACKAGE 0x03
#define SENSORDATA_PACKAGE 0x04
#define NUM_OF_PACKAGES 4


char* buildSensorPackage(const sensordata &data, char* res, int &len);
void emergencyStopCallback(char *data, int len, void *additional_info);
void ps3DataCallback(char *data, int len, void *additional_info);

#endif

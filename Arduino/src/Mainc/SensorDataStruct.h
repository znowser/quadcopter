#ifndef SENSORDATA_STRUCT
#define SENSORDATA_STRUCT

#include "Arduino.h"
#include "helper_3dmath.h"

/*======== Mapping of hardwarePins ========*/
enum MotorPins { 
  leftfront = 0, rightfront = 1, leftback = 2, rightback = 3}; 
enum CellPins{
  cell1 = 0, cell2 = 1, cell3 = 2};
/*=========================================*/

struct sensordata {
  /*Angles*/
  int motorSpeed[4];
  float cellVoltage[3];
  float angleYaw;
  float anglePitch;
  float angleRoll;
  /*Accelerations*/
  VectorInt16 acc;

  float temperature;
  float pressure;
  float height;

};

#endif

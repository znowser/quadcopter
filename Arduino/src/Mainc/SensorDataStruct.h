#ifndef SENSORDATA_STRUCT
#define SENSORDATA_STRUCT

/*======== Mapping of hardwarePins ========*/
enum MotorPins { 
  leftfront = 10, rightfront = 11, leftback = 12, rightback = 13}; 
enum CellPins{
  cell1 = A0, cell2 = A1, cell3 = A2};

struct sensordata {
  /*Angles*/
  float angleYaw;
  float anglePitch;
  float angleRoll;
  /*Accelerations*/
  float temperature;
  float preasure;
  float height;
};

#endif

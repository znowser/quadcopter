#include "Arduino.h"
#include "Motor.h"
#include "BussProtocol.h"


//Function that send away the whole sensorstruct to the rasp
//Format the package by martins format
char* buildSensorPackage(const sensordata &data, char* res, int &len) {
  len = 0;
  
  len += sprintf(&res[len], "c1:%d!", (int)data.cellVoltage[CELL1]);
  len += sprintf(&res[len], "c2:%d!", (int)data.cellVoltage[CELL2]);
  len += sprintf(&res[len], "c3:%d!", (int)data.cellVoltage[CELL3]);

  len += sprintf(&res[len], "lf:%d!", data.motorSpeed[LF]);
  len += sprintf(&res[len], "rf:%d!", data.motorSpeed[RF]);
  len += sprintf(&res[len], "lb:%d!", data.motorSpeed[LB]);
  len += sprintf(&res[len], "rb:%d!", data.motorSpeed[RB]);

  len += sprintf(&res[len], "temp:%d!", (int)data.temperature);
  len += sprintf(&res[len], "height:%d!", (int)data.height);

  len += sprintf(&res[len], "yaw:%d!", data.angleYaw);
  len += sprintf(&res[len], "pitch:%d!", data.anglePitch);
  len += sprintf(&res[len], "roll:%d!", data.angleRoll);
  return res;
}

void emergencyStopCallback(char *data, int len, void *additional_info) {
  Motor *motor = (Motor*)additional_info;
  //Turn off all motors
  motor[LF].setSpeed(0);
  motor[RF].setSpeed(0);
  motor[LB].setSpeed(0);
  motor[RB].setSpeed(0);
}

void ps3DataCallback(char *data, int len, void *additional_info) {
  sensordata *sensorData = (sensordata*)additional_info;
}

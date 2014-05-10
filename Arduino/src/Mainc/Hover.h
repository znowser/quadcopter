#ifndef HOVER_H
#define HOVER_H

#include "Motor.h"
#include "SensorDataStruct.h"

class Hover {
private:
  Motor *motors;
  sensordata sensor;
  float refHeight;
  unsigned long time;
  float old_lfmh;
  float old_rfmh;
  float old_lfmv;
  float old_rfmv;
  float old_lfma;
  float old_rfma;
  float old_cbh;
  float old_cbv;
  float old_cba;
  float old_errorHeight;
  float speed;
public:
  Hover(Motor *motors, sensordata &sensor, float refHeight);
  void Regulate(void);
};

#endif

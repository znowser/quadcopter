#ifndef HOVER_H
#define HOVER_H

#include "Motor.h"
#include "SensorDataStruct.h"

class Hover {
private:
  Motor *motors;
  sensordata *sensor;
  float refAltitude;
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
  float old_errorAltitude;
  float speed_lf;
  float speed_rf;
  float speed_lb;
  float speed_rb;
public:
  Hover() {};
  Hover(Motor *motors, sensordata *sensor, float refAltitude);
  void Regulate(void);
  void init(Motor *motors, sensordata *sensor, float refAltitude);
};

#endif

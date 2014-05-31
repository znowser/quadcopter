#ifndef HOVER_H
#define HOVER_H

#include "Motor.h"
#include "SensorDataStruct.h"

#define SEC 1000000

#define axisX 0
#define axisY 1
#define axisZ 2
#define axisRo 3
#define axisPi 4
#define axisYa 5

#define CALIBRATION_CNT 256
#define SAMPLE_CNT 256

#define MAX_RUNTIME 20      // Number of second the test should last
#define SPEED_UP_LIM 10     // Number of +1 speed increase/second (speed decreases rest of time)
#define START_SPEED 25      // Motor start speed

class Hover {

private:
  /* Hardware */
  Motor *motors;
  sensordata *sensor;
  /* Time */
  unsigned long startTime, speedUpTime;
  int speedUpCnt;
  /* Calibration */
  int calCnt, sampleCnt;  
  //long a[3][2], v[3][2], p[3][2], pRef[3];
  long smp[6], sstate[6];
  /* PD */
  int e[6], eOld[6], u[6];
  float K[6], Td[6];
  /* MOTOR EFFECT */
  int speed[4];

public:
  Hover(Motor *motors, sensordata *sensor, float refAltitude);
  void init(Motor *motors, sensordata *sensor, float refAltitude);
  bool Calibrate(void);
  void Regulate(void);

};

#endif

#ifndef HOVER_H
#define HOVER_H

#include "Motor.h"
#include "SensorDataStruct.h"

#define ROLL 0
#define PITCH 1
#define YAW 2
#define X 0
#define Y 1
#define Z 2
#define axisX 0
#define axisY 1
#define axisZ 2
#define axisRo 3
#define axisPi 4
#define axisYa 5

class Hover {
private:

  /* Debug */
  int debug_maxMotorEffect;

  /* Hardware */
  Motor *motors;
  sensordata *sensor;

  /* last call and last motor update */
  unsigned long timestamp, timestampCurrent, timestampMotor, timestampPrint;
  unsigned long dt;

  /* Calibration */
  int calCnt;
  int sstate[3];
  int min, max;

  int sampleCnt, sampleSize;
  //float rawToSI;
  int acc[3];
  int a[3][2], v[3][2], p[3][2];
  int pRef[3];

  /* PID */
  int Ts;
  int Ti[6], Td[6], K[6], I[6], e[6], eOld[6], u[6];

  /* Internal speed when debugging without real engines */
  int speed[4];

public:
  Hover(Motor *motors, sensordata *sensor, float refAltitude);
  void init(Motor *motors, sensordata *sensor, float refAltitude);
  bool Calibrate(void);
  void Regulate(void);
};

#endif

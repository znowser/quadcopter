#ifndef HOVER_H
#define HOVER_H

#include "Motor.h"
#include "SensorDataStruct.h"

#define motorDistanceCentre 0.4

/* OBSERVER!
* How to run this regulator:
* 1. Run init
* 2. Run Calibrate until it returns true
* 3. Run Regulate as many times regulation is needed
*/


class Hover {
private:

  enum Angle { ROLL = 0, PITCH = 1, YAW = 2 };
  enum Position { X = 0, Y = 1, Z = 2 };
  enum Reg { axisX = 0, axisY = 1, axisZ = 2, axisRo = 3, axisPi = 4, axisYa = 5 };

  /* Debug */
  int debug_maxMotorEffect;

  /* Hardware */
  Motor *motors;
  sensordata *sensor;

  /* last call and last motor update */
  unsigned long timestamp, timestampCurrent, timestampMotor, timestampPrint;
  float dt;

  /* Calibration */
  int calCnt;
  int sstate[3];
  int min[3], max[3];

  int sampleCnt, sampleSize;
  float acc[3];
  float a[3][2], v[3][2], p[3][2];
  float pRef[3];

  /* PID */
  float Ts;
  float Ti[6], Td[6], K[6], I[6], e[6], eOld[6], u[6];

  /* Internal speed when debugging without real engines */
  float speed_lf, speed_rf, speed_lb, speed_rb;

  void integrate(float v[3][2], float a[3][2]);

public:
  Hover(Motor *motors, sensordata *sensor, float refAltitude);
  void init(Motor *motors, sensordata *sensor, float refAltitude);
  void Calibrate(void);
  void Regulate(void);
};

#endif

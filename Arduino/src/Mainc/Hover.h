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

  /* time and  */
  //unsigned long dt, timestamp, timestampCurrent, timestampMotor;  
  int debug_print;

  /* Calibration */
//  int deadzone_min, deadzone_max, calCnt;
  int calCnt;
  int sampleCnt;
  //float rawToSI;
  long acc[3], sstate[6];
  long a[3][2], v[3][2], p[3][2];
  long pRef[3];

  /* PID */
  int Ts;
  int Td[6], K[6], e[6], eOld[6], u[6];

  /* Internal speed when debugging without real engines */
  int speed[4];

public:
  Hover(Motor *motors, sensordata *sensor, float refAltitude);
  void init(Motor *motors, sensordata *sensor, float refAltitude);
  bool Calibrate(void);
  void Regulate(void);

};

#endif

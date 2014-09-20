#ifndef HOVER_H
#define HOVER_H

#include "../Motor/Motor.h"
#include "../SensorDataStruct.h"

#define SEC 1000000

#define axisX 0
#define axisY 1
#define axisZ 2
#define axisRo 3
#define axisPi 4
#define axisYa 5

#define MOTOR_MAX 50
#define MOTOR_MIN 10

#define MAX_RUNTIME 20        // Number of second the test should last
#define CRUISE_TIME 10
#define SPEED_UP_LIM 30      // Number of +1 speed increase/second (speed decreases rest of time) lift off > 34
#define START_SPEED 20       // Motor start speed

class Hover {

private:
  Motor *motors;
  sensordata *sensor;
  unsigned long timestamp, timestampSpeed;
  int speedUpCnt, calCnt;  
  float e[6], eOld[6], u[6], ref[6];
  float I[6], K[6], Td[6], Ti[6];
  float speed[4];
  void calcPID(float t);
  
public:
  Hover(Motor *motors, sensordata *sensor, float ref[6]);
  void init(Motor *motors, sensordata *sensor, float _ref[6]);
  void Regulate(void);
  void KillMotors(void);
};

#endif

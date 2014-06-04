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

#define MOTOR_MAX 50
#define MOTOR_MIN 10
#define accDeadzone 32      
#define angDeadzone 3

#define COLD_START 1024       // Number of iteration before actual calibration
#define CALIBRATION_CNT 256   // Number of sampled values during calibration
#define SAMPLE_CNT 4          // Samples for average values

#define VELOCITY_REDUCE 0.66f // Velocity must always reduced at every postion calc. 

#define MAX_RUNTIME 12        // Number of second the test should last
#define CRUISE_TIME 2
#define SPEED_UP_LIM 38      // Number of +1 speed increase/second (speed decreases rest of time) lift off > 34
#define START_SPEED 32       // Motor start speed


// Current config will let the quad accelerate from 20 to 30 and then down to 20.
// Idea is to test the top value intil lift of :-)

/*  START_SPEED set to something low.
 *  START_SPEED + SPEED_UP_LIM is then the top speed set during runtime IF MAX_RUNTIME is lower then the sum.
 *  START_SPEED + SPEED_UP_LIMIT - MAX_RUNTIME is the speed before the stop.
 */

class Hover {

private:
  /* Hardware */
  Motor *motors;
  sensordata *sensor;
  /* Time */
  unsigned long startTime, lastTime;
  int speedUpCnt;
  /* PD */
  float e[6], eOld[6], u[6];
  float K[6], Td[6], Ti[6];
  /* MOTOR EFFECT */
  float speed[4];
  int runCnt;

public:
  Hover(Motor *motors, sensordata *sensor, float refAltitude);
  void init(Motor *motors, sensordata *sensor, float refAltitude);
  bool Calibrate(void);
  void Regulate(void);
};

#endif

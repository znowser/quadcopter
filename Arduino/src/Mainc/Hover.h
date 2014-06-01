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

#define MOTOR_MAX 60
#define MOTOR_MIN 10
#define accDeadzone 32      
#define angDeadzone 4

#define COLD_START 1024       // Number of iteration before actual calibration
#define CALIBRATION_CNT 256   // Number of sampled values during calibration
#define SAMPLE_CNT 4          // Samples for average values

#define VELOCITY_REDUCE 0.66f // Velocity must always reduced at every postion calc. 

#define MAX_RUNTIME 20        // Number of second the test should last
#define SPEED_UP_LIM 55      // Number of +1 speed increase/second (speed decreases rest of time) lift off > 44
#define START_SPEED 40       // Motor start speed


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
  unsigned long startTime, speedUpTime;
  int speedUpCnt;
  /* Calibration */
  int calCnt, sampleCnt;  
  //long a[3][2], v[3][2], p[3][2], pRef[3];
  long smp[6], sstate[6];
  /* PD */
  float e[6], eOld[6], u[6];
  float K[6], Td[6];
  /* MOTOR EFFECT */
  float speed[4];

public:
  Hover(Motor *motors, sensordata *sensor, float refAltitude);
  void init(Motor *motors, sensordata *sensor, float refAltitude);
  bool Calibrate(void);
  void Regulate(void);

};

#endif

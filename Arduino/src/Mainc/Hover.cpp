#include "Hover.h"

Hover::Hover(Motor *motors, sensordata *sensor, float ref[6]) {
  init(motors, sensor, ref);
}

void Hover::init(Motor *motors, sensordata *sensor, float _ref[6]) {
  // Hardware
  this->motors = motors;
  this->sensor = sensor;    
  timestamp = speedUpCnt = timestampSpeed = 0;
  for (int axis = axisX; axis <= axisYa; ++axis) {
    ref[axis] = _ref[axis];
    Td[axis] = 4.f;
    Ti[axis] = 1.f;    
    K[axis] = .0675f;                 
    e[axis] = eOld[axis] = u[axis] = 0;
  }
  speed[LF] = speed[RF] = speed[LB] = speed[RB] = START_SPEED;
  Serial.println("Hover initialized...");
}

void Hover::Regulate(void) {
  unsigned long time = micros();
  unsigned long lapsedTime = time - timestamp;
  float dt = lapsedTime / 1000000.f;
  // Safety, kill motors on count or timing.
  if (speedUpCnt > MAX_RUNTIME) {
    KillMotors();
    return;
  }
  // Speed timing sequence, increase time SPEED_UP_LIMIT times then decrease the rest...
  if (time - timestampSpeed > SEC) {
    timestampSpeed = time;
    if (++speedUpCnt <= SPEED_UP_LIM - START_SPEED) {
      ++speed[LF];
      ++speed[RF];
      ++speed[LB];
      ++speed[RB];
    } else if (speedUpCnt > SPEED_UP_LIM - START_SPEED + CRUISE_TIME){
      speed[LF] -= 2;
      speed[RF] -= 2;
      speed[LB] -= 2;
      speed[RB] -= 2;
    }
  } 
  calcPID(dt);
  // LIMIT ENGINE MIN MAX SPEED
  speed[LF] = max(min(speed[LF] + u[axisRo] - u[axisPi], MOTOR_MAX), MOTOR_MIN);
  speed[RF] = max(min(speed[RF] - u[axisRo] - u[axisPi], MOTOR_MAX), MOTOR_MIN);
  speed[LB] = max(min(speed[LB] + u[axisRo] + u[axisPi], MOTOR_MAX), MOTOR_MIN);
  speed[RB] = max(min(speed[RB] - u[axisRo] + u[axisPi], MOTOR_MAX), MOTOR_MIN);
  // SET ENGINE SPEED LIMIT TO MIN AND MAX
  motors[LF].setSpeed(speed[LF]);
  motors[RF].setSpeed(speed[RF]);
  motors[LB].setSpeed(speed[LB]);
  motors[RB].setSpeed(speed[RB]);
  timestamp = time;
}

void Hover::KillMotors() {
  motors[LF].setSpeed(0);
  motors[RF].setSpeed(0);
  motors[LB].setSpeed(0);
  motors[RB].setSpeed(0); 
}

void Hover::calcPID(float t) {
  int axis;
  for (axis = 0; axis < 6; ++axis) {
    e[axis] = sensor->acc[axis] - ref[axis];
    I[axis] = I[axis] + (t / Ti[axis]) * e[axis];
    u[axis] = K[axis] * (e[axis] + I[axis] + (Td[axis] / t) * (e[axis] - eOld[axis]));
    eOld[axis] = e[axis];
  }
}

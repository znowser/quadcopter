#include "Hover.h"

Hover::Hover(Motor *motors, sensordata *sensor, float refAltitude) {
  init(motors, sensor, refAltitude);
}

void Hover::init(Motor *motors, sensordata *sensor, float refAltitude) {
  // Hardware
  this->motors = motors;
  this->sensor = sensor;   
  calCnt = -COLD_START;
  // PD vars
  for (int i = 0; i < 6; ++i) {
    Td[i] = 4.f;  
    Ti[i] = 1.f;    
    K[i] = .0675f;                 
    e[i] = eOld[i] = u[i] = 0;
  }
  // Initial motor speed, then first when Regulate() is called.
  speed[LF] = speed[RF] = speed[LB] = speed[RB] = START_SPEED;
}

/* Calibrates sensorvalues by sampling a number of times and then average on those values */
bool Hover::Calibrate() {
  if (++calCnt > CALIBRATION_CNT) return true;
  if (calCnt == CALIBRATION_CNT) {
    startTime = micros();
    speedUpTime = startTime;
    return calCnt++;
  }
  return false;
}

void Hover::Regulate(void) {
  unsigned long time = micros();
  float dt = ((float)(time - lastTime)) / ((float)SEC);
  lastTime = time;
  // Safety, kill motors on count or timing.
  if (speedUpCnt > MAX_RUNTIME || time - startTime > MAX_RUNTIME * SEC) {
    motors[LF].setSpeed(0);
    motors[RF].setSpeed(0);
    motors[LB].setSpeed(0);
    motors[RB].setSpeed(0);
    return;
  }
  // Speed timing sequence, increase time SPEED_UP_LIMIT times then decrease the rest...
  if (time - startTime > speedUpTime * SEC) {
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
    speedUpTime = time;
  } 
  // axis X
  e[axisX] = pRef[axisX] - sensor_position_TODO;
  I[axisX] = I[axisX] + (dt / Ti[axisX]) * e[axisX]; 
  u[axisX] = K[axisX] * (e[axisX] + I[axisX] + (Td[axisX]/dt) * (e[axisX] - eOld[axisX]));
  // axis Y
  e[axisY] = pRef[axisY] - sensor_position_TODO;
  I[axisY] = I[axisY] + (dt / Ti[axisY]) * e[axisY]; 
  u[axisY] = K[axisY] * (e[axisY] + I[axisY] + (Td[axisY]/dt) * (e[axisY] - eOld[axisY]));
  // axis Z
  e[axisZ] = pRef[axisZ] - sensor_position_TODO;
  I[axisZ] = I[axisZ] + (dt / Ti[axisZ]) * e[axisZ]; 
  u[axisZ] = K[axisZ] * (e[axisZ] + I[axisZ] + (Td[axisZ]/dt) * (e[axisZ] - eOld[axisZ]));
  // axis Roll
  e[axisRo] = ;
  u[axisRo] = K[axisRo] * (e[axisRo] + Td[axisRo] * (e[axisRo] - eOld[axisRo]));
  // axis Pitch
  e[axisPi] = ;
  u[axisPi] = K[axisPi] * (e[axisPi] + Td[axisPi] * (e[axisPi] - eOld[axisPi]));
  // axis Yaw
  e[axisYa] = ;
  u[axisYa] = K[axisYa] * (e[axisYa] + Td[axisYa] * (e[axisYa] - eOld[axisYa]));  
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
  // Store state
  for (int i = 0; i < 6; ++i)
    eOld[i] = e[i];
}

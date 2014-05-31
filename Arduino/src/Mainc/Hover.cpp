#include "Hover.h"

Hover::Hover(Motor *motors, sensordata *sensor, float refAltitude) {
  init(motors, sensor, refAltitude);
}

void Hover::init(Motor *motors, sensordata *sensor, float refAltitude) {
  // Hardware
  this->motors = motors;
  this->sensor = sensor;   
  // Sample interator
  sampleCnt = speedUpCnt = 0;
  calCnt = -1024;
  // acceleration, velocity, position and offset!
  for (int i = 0; i < 3; ++i)
    smp[i] = a[i][0] = a[i][1] = v[i][0] = v[i][1] = p[i][0] = p[i][1] = pRef[i] = sstate[X] = sstate[Y] = sstate[Z] = 0;
  // PD vars
  for (int i = 0; i < 6; ++i) {
    Td[i] = 10.f;
    K[i] = 0.01f;
    e[i] = eOld[i] = u[i] = 0;
  }
  // INITIAL MOTOR SPEED, KEEP BELOW 30! 
  speed[LF] = speed[RF] = speed[LB] = speed[RB] = START_SPEED;
}

/* Calibrates sensorvalues by sampling a number of times and then average on those values */
bool Hover::Calibrate() {
  if (calCnt > CALIBRATION_CNT) return true;
  if (++calCnt > 0) {
    sstate[X] += sensor->acc.x;
    sstate[Y] += sensor->acc.y;
    sstate[Z] += sensor->acc.z;
    sstate[axisRo] += sensor->angleRoll;
    sstate[axisPi] += sensor->anglePitch;
    sstate[axisYa] += sensor->angleYaw;
  }
  if (calCnt == CALIBRATION_CNT) {
    sstate[X] /= CALIBRATION_CNT;
    sstate[Y] /= CALIBRATION_CNT;
    sstate[Z] /= CALIBRATION_CNT;
    sstate[axisRo] /= CALIBRATION_CNT;
    sstate[axisPi] /= CALIBRATION_CNT;
    sstate[axisYa] /= CALIBRATION_CNT;
    startTime = micros();
    return calCnt++;
  }
  return false;
}

void Hover::Regulate(void) {
  // Safety, kill motors
  if (speedUpCnt > MAX_RUNTIME || micros() - startTime > MAX_RUNTIME * SEC) {
    motors[LF].setSpeed(0);
    motors[RF].setSpeed(0);
    motors[LB].setSpeed(0);
    motors[RB].setSpeed(0);
    return;
  }
  // Debug
  int debug_maxMotorEffect = 35;
  int debug_minMotorEffect = 10;
  int debug_print = 0;
  // Deadzones
  int accDeadzone = 16;      
  int angDeadzone = 2;
  // Sample values using filtration of low values.
  int x = sensor->acc.x - sstate[axisX];
  int y = sensor->acc.y - sstate[axisY];
  int z = sensor->acc.z - sstate[axisZ]; 
  int aR = sensor->angleRoll - sstate[axisRo]; 
  int aP = sensor->anglePitch - sstate[axisPi]; 
  int aY = sensor->angleYaw - sstate[axisYa];  
  smp[axisX] += x > accDeadzone || x < -accDeadzone ? x : 0;
  smp[axisY] += y > accDeadzone || y < -accDeadzone ? y : 0;
  smp[axisZ] += z > accDeadzone || z < -accDeadzone ? z : 0;
  smp[axisRo] += aR > angDeadzone || aR < -angDeadzone : aR : 0;
  smp[axisRo] += aP > angDeadzone || aP < -angDeadzone : aP : 0;
  smp[axisRo] += aY > angDeadzone || aY < -angDeadzone : aY : 0;
  // Speed up sequence
  if (micros() - speedUpTime > SEC) {
    ++speedUpCnt;
    if (++speedUpCnt < SPEED_UP_LIM) {
      ++speed[LF];
      ++speed[RF];
      ++speed[LB];
      ++speed[RB];
    } else {
      --speed[LF];
      --speed[RF];
      --speed[LB];
      --speed[RB];
    }
    speedUpTime = micros() - speedUpTime;
  }
  // Do stuff on sensor data.
  if (++sampleCnt % SAMPLE_CNT == 0) {
    // X
    a[X][1] = smp[X] / SAMPLE_CNT;
    v[X][1] = (v[X][0] + a[X][0] + (a[X][1] - a[X][0]) / 2);
    p[X][1] = (p[X][0] + v[X][0] + (v[X][1] - v[X][0]) / 2);
    // Y
    a[Y][1] = smp[Y] / SAMPLE_CNT;
    v[Y][1] = (v[Y][0] + a[Y][0] + (a[Y][1] - a[Y][0]) / 2);
    p[Y][1] = (p[Y][0] + v[Y][0] + (v[Y][1] - v[Y][0]) / 2);
    // Z
    a[Z][1] = smp[Z] / SAMPLE_CNT;
    v[Z][1] = (v[Z][0] + a[Z][0] + (a[Z][1] - a[Z][0]) / 2);
    p[Z][1] = (p[Z][0] + v[Z][0] + (v[Z][1] - v[Z][0]) / 2);
    // Store values
    for (int i = 0; i < 3; ++i) {
      a[i][0] = a[i][1];
      v[i][0] = v[i][1];
      p[i][0] = p[i][1];
    }
    // axis X
    e[axisX] = pRef[X] - p[X][1];
    u[axisX] = K[axisX] * (e[axisX] + Td[axisX] * (e[axisX] - eOld[axisX]));
    // axis Y
    e[axisY] = pRef[Y] - p[Y][1];
    u[axisY] = K[axisY] * (e[axisY] + Td[axisY] * (e[axisY] - eOld[axisY]));
    // axis Z
    e[axisZ] = pRef[Z] - p[Z][1];
    u[axisZ] = K[axisZ] * (e[axisZ] + Td[axisZ] * (e[axisZ] - eOld[axisZ]));
    // axis Roll
    e[axisRo] = smp[axisRo] / SAMPLE_CNT;
    u[axisRo] = K[axisRo] * (e[axisRo] + Td[axisRo] * (e[axisRo] - eOld[axisRo]));
    // axis Pitch
    e[axisPi] = smp[axisPi] / SAMPLE_CNT;
    u[axisPi] = K[axisPi] * (e[axisPi] + Td[axisPi] * (e[axisPi] - eOld[axisPi]));
    // axis Yaw
    e[axisYa] = smp[axisYa] / SAMPLE_CNT;
    u[axisYa] = K[axisYa] * (e[axisYa] + Td[axisYa] * (e[axisYa] - eOld[axisYa]));    
    // LIMIT ENGINE MIN MAX SPEED
    speed[LF] = max(min(speed[LF] + u[axisRo] - u[axisPi], debug_maxMotorEffect), debug_minMotorEffect);
    speed[RF] = max(min(speed[RF] - u[axisRo] - u[axisPi], debug_maxMotorEffect), debug_minMotorEffect);
    speed[LB] = max(min(speed[LB] + u[axisRo] + u[axisPi], debug_maxMotorEffect), debug_minMotorEffect);
    speed[RB] = max(min(speed[RB] - u[axisRo] + u[axisPi], debug_maxMotorEffect), debug_minMotorEffect);
    // SET ENGINE SPEED LIMIT TO MIN AND MAX
    motors[LF].setSpeed(speed[LF]);
    motors[RF].setSpeed(speed[RF]);
    motors[LB].setSpeed(speed[LB]);
    motors[RB].setSpeed(speed[RB]);
    // Reset samples
    smp[axisX] = smp[axisY] = smp[axisZ] = 0; 
    // Store state
    for (int i = 0; i < 6; ++i)
      eOld[i] = e[i];
  }
}

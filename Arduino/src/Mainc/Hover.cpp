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
  calCnt = -COLD_START;
  // acceleration, velocity, position and offset!
  for (int i = 0; i < 3; ++i) {
    smp[i] = sstate[axisRo] = sstate[axisPi] = 0;
    //a[i][0] = a[i][1] = v[i][0] = v[i][1] = p[i][0] = p[i][1] = pRef[i] = sstate[axisX] = sstate[axisY] = sstate[axisZ] = sstate[axisYa] = 0;
  }
  // PD vars
  for (int i = 0; i < 6; ++i) {
    Td[i] = 0.33f;               // Best guess, keep lower then 1
    K[i] = 0.33f;                 // Best guess, regulate level of both P and D
    e[i] = eOld[i] = u[i] = 0;
  }
  // Initial motor speed, then first when Regulate() is called.
  speed[LF] = speed[RF] = speed[LB] = speed[RB] = START_SPEED;
}

/* Calibrates sensorvalues by sampling a number of times and then average on those values */
bool Hover::Calibrate() {
  if (calCnt > CALIBRATION_CNT) return true;
  if (++calCnt > 0) {
    /*
    sstate[axisX] += sensor->acc.x;
    sstate[axisY] += sensor->acc.y;
    sstate[axisZ] += sensor->acc.z;
    sstate[axisYa] += sensor->angleYaw;
    */
    sstate[axisRo] += sensor->angleRoll;
    sstate[axisPi] += sensor->anglePitch;
  }
  if (calCnt == CALIBRATION_CNT) {
    /*
    sstate[axisX] /= CALIBRATION_CNT;
    sstate[axisY] /= CALIBRATION_CNT;
    sstate[axisZ] /= CALIBRATION_CNT;
    sstate[axisYa] /= CALIBRATION_CNT;
    */
    sstate[axisRo] /= CALIBRATION_CNT;
    sstate[axisPi] /= CALIBRATION_CNT;
    startTime = micros();
    speedUpTime = startTime;
    return calCnt++;
  }
  return false;
}

void Hover::Regulate(void) {
  // Safety, kill motors on count or timing.
  if (speedUpCnt > MAX_RUNTIME || micros() - startTime > MAX_RUNTIME * SEC) {
    motors[LF].setSpeed(0);
    motors[RF].setSpeed(0);
    motors[LB].setSpeed(0);
    motors[RB].setSpeed(0);
    return;
  }
  // Sample values
  /*
  int x = sensor->acc.x - sstate[axisX];
  smp[axisX] += x > accDeadzone || x < -accDeadzone ? x : 0;
  int y = sensor->acc.y - sstate[axisY];
  smp[axisY] += y > accDeadzone || y < -accDeadzone ? y : 0;
  int z = sensor->acc.z - sstate[axisZ];
  smp[axisZ] += z > accDeadzone || z < -accDeadzone ? z : 0;
  int aY = sensor->angleYaw - sstate[axisYa];  
  smp[axisRo] += aY > angDeadzone || aY < -angDeadzone ? aY : 0;
  */
  int aR = sensor->angleRoll - sstate[axisRo];
  smp[axisRo] += aR > angDeadzone || aR < -angDeadzone ? aR : 0;
  int aP = sensor->anglePitch - sstate[axisPi]; 
  smp[axisPi] += aP > angDeadzone || aP < -angDeadzone ? aP : 0;
  /*
  if (smp[axisPi] > 10 || smp[axisPi] < -10 || smp[axisRo] > 10 || smp[axisRo] < -10) {
    Serial.print(smp[axisPi]);
    Serial.print(", ");
    Serial.print(sstate[axisPi]);
    Serial.println(" epic fail!");
  }
  */
  // Speed timing sequence, increase time SPEED_UP_LIMIT times then decrease the rest...
  if (micros() - speedUpTime > SEC) {
    if (++speedUpCnt <= SPEED_UP_LIM - START_SPEED) {
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
    speedUpTime = micros();
  }
  // Do stuff on sensor data, use average value of SAMPLE_CNT values.
  if (++sampleCnt % SAMPLE_CNT == 0) {
    /*
    // X
    a[axisX][1] = smp[axisX] / SAMPLE_CNT;
    v[axisX][1] = (v[axisX][0] + a[axisX][0] + (a[axisX][1] - a[axisX][0]) / 2) * VELOCITY_REDUCE;
    p[axisX][1] = (p[axisX][0] + v[axisX][0] + (v[axisX][1] - v[axisX][0]) / 2);
    // Y
    a[axisY][1] = smp[axisY] / SAMPLE_CNT;
    v[axisY][1] = (v[axisY][0] + a[axisY][0] + (a[axisY][1] - a[axisY][0]) / 2) * VELOCITY_REDUCE;
    p[axisY][1] = (p[axisY][0] + v[axisY][0] + (v[axisY][1] - v[axisY][0]) / 2);
    // Z
    a[axisZ][1] = smp[axisZ] / SAMPLE_CNT;
    v[axisZ][1] = (v[axisZ][0] + a[axisZ][0] + (a[axisZ][1] - a[axisZ][0]) / 2) * VELOCITY_REDUCE;
    p[axisZ][1] = (p[axisZ][0] + v[axisZ][0] + (v[axisZ][1] - v[axisZ][0]) / 2);
    // Store values
    for (int i = 0; i < 3; ++i) {
      a[i][0] = a[i][1];
      v[i][0] = v[i][1];
      p[i][0] = p[i][1];
    }
    // axis X
    e[axisX] = pRef[axisX] - p[axisX][1];
    u[axisX] = K[axisX] * (e[axisX] + Td[axisX] * (e[axisX] - eOld[axisX]));
    // axis Y
    e[axisY] = pRef[axisY] - p[axisY][1];
    u[axisY] = K[axisY] * (e[axisY] + Td[axisY] * (e[axisY] - eOld[axisY]));
    // axis Z
    e[axisZ] = pRef[axisZ] - p[axisZ][1];
    u[axisZ] = K[axisZ] * (e[axisZ] + Td[axisZ] * (e[axisZ] - eOld[axisZ]));
    // axis Yaw
    e[axisYa] = smp[axisYa] / SAMPLE_CNT;
    u[axisYa] = K[axisYa] * (e[axisYa] + Td[axisYa] * (e[axisYa] - eOld[axisYa]));  
    */
    // axis Roll
    e[axisRo] = smp[axisRo] / SAMPLE_CNT;
    u[axisRo] = K[axisRo] * (e[axisRo] + Td[axisRo] * (e[axisRo] - eOld[axisRo]));
    // axis Pitch
    e[axisPi] = smp[axisPi] / SAMPLE_CNT;
    u[axisPi] = K[axisPi] * (e[axisPi] + Td[axisPi] * (e[axisPi] - eOld[axisPi]));
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
    /*
    // debug!
    if (e[axisRo] != 0 || e[axisPi] != 0) {
      Serial.print("ERR: ");
      Serial.print(speed[LF]);
      Serial.print(", ");
      Serial.print(speed[RF]);
      Serial.print(", ");
      Serial.print(speed[LB]);
      Serial.print(", ");
      Serial.println(speed[RB]);
    }
    */
    // Reset samples
    sampleCnt = 0;
    //smp[axisX] = smp[axisY] = smp[axisZ] = smp[axisYa] = 0; 
    smp[axisRo] = smp[axisPi] = 0; 
    // Store state
    for (int i = 0; i < 6; ++i)
      eOld[i] = e[i];
  }
}

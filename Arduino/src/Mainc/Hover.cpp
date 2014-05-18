#include "Hover.h"

Hover::Hover(Motor *motors, sensordata *sensor, float refAltitude) {
  init(motors, sensor, refAltitude);
}

void Hover::init(Motor *motors, sensordata *sensor, float refAltitude) {

  /* Hardware */
  this->motors = motors;
  this->sensor = sensor;
  timestampPrint = timestampMotor = timestamp = micros();

  /* Debug */
  debug_maxMotorEffect = 100;

  /* Calibration */
  calCnt = -512;
  sampleSize = 4;
  sampleCnt = 0;
  min = -5;
  max = 5;
  for (int i = 0; i < 3; ++i) {
    acc[i] = 0;
    a[i][0] = 0;
    a[i][1] = 0;
    v[i][0] = 0;
    v[i][1] = 0;
    p[i][0] = 0;
    p[i][1] = 0;
    pRef[i] = 0;
    sstate[i] = 0;
  }
  for (int i = 0; i < 6; ++i) {
    Ti[i] = 1;
    Td[i] = 1;
    K[i] = 1;
    I[i] = 1;
    e[i] = 0;
    eOld[i] = 0;
    u[i] = 0;
  }

  /* Initial motor speed, range 0 - 100 */
  speed[LF] = motors[LF].getSpeed();
  speed[RF] = motors[RF].getSpeed();
  speed[LB] = motors[LB].getSpeed();
  speed[RF] = motors[RF].getSpeed();

  speed[LF] = speed[RF] = speed[LB] = speed[RB] = 50;

  Serial.println("Hoverregulator initialized");
}

bool Hover::Calibrate() {
  if (calCnt > 1023) {
    return true;
  } else {
    // run calibration 1024 times before retrieving actual values.
    if (++calCnt > 511) {
      sstate[X] += sensor->acc.x;
      sstate[Y] += sensor->acc.y;
      sstate[Z] += sensor->acc.z;
      // sample 512 times before getting the actual average.
      if (calCnt == 1024) {
        sstate[X] = (sstate[X] / calCnt);
        sstate[Y] = (sstate[Y] / calCnt);
        sstate[Z] = (sstate[Z] / calCnt);
        Serial.println("Calibration complete");
        Serial.print("Offset X:");
        Serial.println(sstate[X]);
        Serial.print("Offset Y:");
        Serial.println(sstate[Y]);
        Serial.print("Offset Z:");
        Serial.println(sstate[Z]);
        return true;
      }
    }
  }
  return false;
}

void Hover::Regulate(void) {
  //Serial.print("-OK-");
  timestampCurrent = micros();
  /* This must be checked! Very important that dt is the same every time! */
  dt = timestampCurrent - timestamp;
  /* Sample values using filtration of low values. */
  acc[X] += sensor->acc.x < min || sensor->acc.x > max ? sensor->acc.x : 0;
  acc[Y] += sensor->acc.y < min || sensor->acc.y > max ? sensor->acc.y : 0;
  acc[Z] += sensor->acc.z < min || sensor->acc.z > max ? sensor->acc.z : 0;
  if (++sampleCnt % sampleSize == 0) {
    // Get acceleration
    a[X][1] = (acc[X] - sstate[X]) >> 2;
    a[Y][1] = (acc[Y] - sstate[Y]) >> 2;
    a[Z][1] = (acc[Z] - sstate[Z]) >> 2;
    // Get velocity
    v[X][1] = v[X][0] + a[X][0] + (a[X][1] - a[X][0]) >> 1;
    v[Y][1] = v[Y][0] + a[Y][0] + (a[Y][1] - a[Y][0]) >> 1;
    v[Z][1] = v[Z][0] + a[Z][0] + (a[Z][1] - a[Z][0]) >> 1;
    // Get position
    p[X][1] = p[X][0] + v[X][0] + (v[X][1] - v[X][0]) >> 1;
    p[Y][1] = p[Y][0] + v[Y][0] + (v[Y][1] - v[Y][0]) >> 1;
    p[Z][1] = p[Z][0] + v[Z][0] + (v[Z][1] - v[Z][0]) >> 1;

    // Store values
    for (int i = 0; i < 3; ++i) {
      a[i][0] = a[i][1];
      v[i][0] = v[i][1];
      p[i][0] = p[i][1];
    }
    // Reset sampling!
    sampleCnt = 0;
    acc[X] = 0;
    acc[Y] = 0;
    acc[Z] = 0;

    /* Debug prints to serial */
    if (timestampCurrent - timestampPrint > 3000000) {
      // print Z-position
      Serial.print("Z: ");
      Serial.print(p[Z][1]);
      Serial.print(", ");
      Serial.print(v[Z][1]);
      Serial.print(", ");
      Serial.println(a[Z][1]);

      timestampPrint = timestampCurrent;
    }

    /* ===== PID  ===== */// check overall performance?
    /* Errors */
    e[axisX] = pRef[X] - p[X][1];
    e[axisY] = pRef[Y] - p[Y][1];
    e[axisZ] = pRef[Z] - p[Z][1];
    e[axisRo] = sensor->anglePitch < 0 ? 360 - sensor->anglePitch : sensor->anglePitch; // Positive means right side above left.
    e[axisPi] = sensor->anglePitch < 0 ? 360 - sensor->anglePitch : sensor->anglePitch; // Positive pitch means front is above back.
    e[axisYa] = 0.f; // direction 
    /* Integrator part */
    // I_k = I_(k-1) + (Ts / Ti) * e -> Ti = Ts / Ti
    I[axisX] = I[axisX] + Ti[axisX] * e[axisX];
    I[axisY] = I[axisY] + Ti[axisY] * e[axisY];
    I[axisZ] = I[axisZ] + Ti[axisZ] * e[axisZ];
    I[axisRo] = I[axisRo] + Ti[axisRo] * e[axisRo];
    I[axisPi] = I[axisPi] + Ti[axisPi] * e[axisPi];
    I[axisYa] = I[axisYa] + Ti[axisYa] * e[axisYa];
    /* output */
    u[axisX] = K[axisX] * (e[axisX] + I[axisX] + Td[axisX] * (e[axisX] - eOld[axisX]));
    u[axisY] = K[axisY] * (e[axisY] + I[axisY] + Td[axisY] * (e[axisY] - eOld[axisY]));
    u[axisZ] = K[axisZ] * (e[axisZ] + I[axisZ] + Td[axisZ] * (e[axisZ] - eOld[axisZ]));
    u[axisRo] = K[axisRo] * (e[axisRo] + I[axisRo] + Td[axisRo] * (e[axisRo] - eOld[axisRo]));
    u[axisPi] = K[axisPi] * (e[axisPi] + I[axisPi] + Td[axisPi] * (e[axisPi] - eOld[axisPi]));
    u[axisYa] = K[axisYa] * (e[axisYa] + I[axisYa] + Td[axisYa] * (e[axisYa] - eOld[axisYa]));

    /* ===== derictional mapping ===== */
    // limit speed to maximum
    speed[LF] = min(speed[LF] + u[axisRo] - u[axisPi], debug_maxMotorEffect);
    speed[RF] = min(speed[RF] - u[axisRo] - u[axisPi], debug_maxMotorEffect);
    speed[LB] = min(speed[LB] + u[axisRo] + u[axisPi], debug_maxMotorEffect);
    speed[RB] = min(speed[RB] - u[axisRo] + u[axisPi], debug_maxMotorEffect);
    // limit speed to minimum
    speed[LF] = speed[LF] < 10 ? 10 : speed[LF];
    speed[RF] = speed[RF] < 10 ? 10 : speed[RF];
    speed[LB] = speed[LB] < 10 ? 10 : speed[LB];
    speed[RB] = speed[RB] < 10 ? 10 : speed[RB];

    /* set engine speed value from 0 to 100 */
    if (false) {
      motors[LF].setSpeed(speed[LF]);
      motors[RF].setSpeed(speed[RF]);
      motors[LB].setSpeed(speed[LB]);
      motors[RB].setSpeed(speed[RB]);
    }

    /* store state */
    for (int i = 0; i < 6; ++i)
      eOld[i] = e[i];
    timestamp = timestampCurrent;
  }
}

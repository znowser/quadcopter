#include "Hover.h"
#include <cmath>
#include <limits>

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
  calCnt = 0;
  sampleSize = 4;
  min = 9999;
  max = 0;

  sampleCnt = 0;
  for (int i = 0; i < 3; ++i) {
    acc[i] = 0.f;
    a[i][0] = 0.f;
    a[i][1] = 0.f;  
    v[i][0] = 0.f;	  
    v[i][1] = 0.f;
    p[i][0] = 0.f;
    p[i][1] = 0.f;
    pRef[i] = 0.f;
    e[i] = 0.f;
    e[i + 3] = 0.f;
  }
  for (int i = 0; i < 6; ++i) {
    Ti[i] = 0.f;
    Td[i] = 0.f;
    K[i] = 0.f;
    I[i] = 0.f;
    e[i] = 0.f;
    eOld[i] = 0.f;
    u[i] = 0.f;
  }

  /* Initial motor speed, range 0 - 100 */
  speed_lf = motors[leftfront].getSpeed();
  speed_rf = motors[rightfront].getSpeed();
  speed_lb = motors[leftback].getSpeed();
  speed_rb = motors[rightfront].getSpeed();

  speed_lf = speed_rf = speed_lb = speed_rb = 50;

  /*  */
  regI[ROLL] = 0;
  regI[PITCH] = 0;
  regI[YAW] = 0;
  eRollOld = 0;
  ePitchOld = 0;
}

bool Hover::Calibrate() { 
  if (calCnt > 2047)
    return true;
  else {
    // run calibration 1024 times before retrieving actual values.
    if (++calCnt > 1023) {
      sstate[X] += sensor->acc.x;
      sstate[Y] += sensor->acc.y;
      sstate[Z] += sensor->acc.z;
      // get min and max during calibration, use as threshold levels (filter)?
      min[X] = sensor->acc.x < min[X] ? sensor->acc.x : min[X];
      min[Y] = sensor->acc.y < min[Y] ? sensor->acc.y : min[Y];
      min[Z] = sensor->acc.z < min[Z] ? sensor->acc.z : min[Z];
      max[X] = sensor->acc.x > max[X] ? sensor->acc.x : max[X];
      max[Y] = sensor->acc.y > max[Y] ? sensor->acc.y : max[Y];
      max[Z] = sensor->acc.z > max[Z] ? sensor->acc.z : max[Z];
      // sample 1024 times before getting the actual average.
      if (calCnt == 2048) {
        sstate[X] = (sstate[X] / calCnt);
        sstate[Y] = (sstate[Y] / calCnt);
        sstate[Z] = (sstate[Z] / calCnt);
        Serial.print("Calibration complete");
        return true;
      }
    }
  }
  return false;
}

void Hover::integrate(float v[3][2], float a[3][2]) {
  for (int i = 0; i < 3; ++i) {
    v[i][1] = v[i][0] + a[i][0] + ((a[i][1] - a[i][0]) / 2.f) * dt;
  }
}

void Hover::Regulate(void) {
  timestampCurrent = micros();
  /* This must be checked! Very important that dt is the same every time! */
  dt = (timestampCurrent - timestamp) / 1000000.f;
  /* Sample values using filtration of low values. */
  acc[X] += sensor->acc.x < min[X] || sensor->acc.x > max[X] ? sensor->acc.x : 0;
  acc[Y] += sensor->acc.y < min[Y] || sensor->acc.y > max[Y] ? sensor->acc.y : 0;
  acc[Z] += sensor->acc.z < min[Z] || sensor->acc.z > max[Z] ? sensor->acc.z : 0;
  if (sampleCnt % sampleSize == 0) {
    // value = ((raw_acc - zero reference offset) / 2^14) * 9.82 * (1 / 4)
    a[X][1] = (acc[X] - sstate[X]) * 0.0001498413086f;
    a[Y][1] = (acc[Y] - sstate[Y]) * 0.0001498413086f;
    a[Z][1] = (acc[Z] - sstate[Z]) * 0.0001498413086f;
    integrate(v, a);
    integrate(p, v);
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
      Serial.println(p[Z][1]);
      timestampPrint = timestampCurrent;
    }

    /* ===== PID  ===== */// check overall performance?
    Ts = (timestampCurrent - timestampMotor) / 1000000.0;
    /* Errors */
    e[axisX] = pRef[X] - p[X];
    e[axisY] = pRef[Y] - p[Y];
    e[axisZ] = pRef[Z] - p[Z];
    e[axisRo] = sensor->anglePitch < 0 ? 360 - sensor->anglePitch : sensor->anglePitch; // Positive means right side above left.
    e[axisPi] = sensor->anglePitch < 0 ? 360 - sensor->anglePitch : sensor->anglePitch; // Positive pitch means front is above back.
    e[axisYa] = 0.f;
    /* Integrator part */
    I[axisX] = I[axisX] + (Ts / Ti[axisX]) * e[axisX];
    I[axisY] = I[axisY] + (Ts / Ti[axisY]) * e[axisY];
    I[axisZ] = I[axisZ] + (Ts / Ti[axisZ]) * e[axisZ];
    I[axisRo] = I[axisRo] + (Ts / Ti[axisRo]) * e[axisRo];
    I[axisPi] = I[axisPi] + (Ts / Ti[axisPi]) * e[axisPi];
    I[axisYa] = I[axisYa] + (Ts / Ti[axisYa]) * e[axisYa];
    /* output */
    u[axisX] = K[axisX] * (e[axisX] + I[axisX] + (Td[axisX] / Ts) * (e[axisX] - aOld[axisX]));
    u[axisY] = K[axisY] * (e[axisY] + I[axisY] + (Td[axisY] / Ts) * (e[axisY] - aOld[axisY]));
    u[axisZ] = K[axisZ] * (e[axisZ] + I[axisZ] + (Td[axisZ] / Ts) * (e[axisZ] - aOld[axisZ]));
    u[axisRo] = K[axisRo] * (e[axisRo] + I[axisRo] + (Td[axisRo] / Ts) * (e[axisRo] - aOld[axisRo]));
    u[axisPi] = K[axisPi] * (e[axisPi] + I[axisPi] + (Td[axisPi] / Ts) * (e[axisPi] - aOld[axisPi]));
    u[axisYa] = K[axisYa] * (e[axisYa] + I[axisYa] + (Td[axisYa] / Ts) * (e[axisYa] - aOld[axisYa]));

    /* ===== derictional mapping ===== */
    // limit speed to maximum
    speed_lf = min(speed_lf + u[axisRo] - u[axisPi], debug_maxMotorEffect);
    speed_rf = min(speed_rf - u[axisRo] - u[axisPi], debug_maxMotorEffect);
    speed_lb = min(speed_lb + u[axisRo] + u[axisPi], debug_maxMotorEffect);
    speed_rb = min(speed_rb - u[axisRo] + u[axisPi], debug_maxMotorEffect);
    // limit speed to minimum
    speed_lf = speed_lf < 10 ? 10 : speed_lf;
    speed_rf = speed_rf < 10 ? 10 : speed_rf;
    speed_lb = speed_lb < 10 ? 10 : speed_lb;
    speed_rb = speed_rb < 10 ? 10 : speed_rb;

    if (false) {
      motors[leftfront].setSpeed(speed_lf);
      motors[rightfront].setSpeed(speed_rf);
      motors[leftback].setSpeed(speed_lb);
      motors[rightback].setSpeed(speed_rb);
    }

    /* store state */
    for (int i = 0; i < 6; ++i)
      eOld[i] = e[i];
    timestamp = timestampCurrent;
  }
}
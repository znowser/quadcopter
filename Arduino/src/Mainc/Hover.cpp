#include "Hover.h"

Hover::Hover(Motor *motors, sensordata *sensor, float refAltitude) {
  init(motors, sensor, refAltitude);
}

void Hover::init(Motor *motors, sensordata *sensor, float refAltitude) {

  /* Hardware */
  this->motors = motors;
  this->sensor = sensor;
  //timestampMotor = timestamp = micros();

  /* Debug */
  debug_maxMotorEffect = 100;
  
  // Sample interator
  sampleCnt = 0;

  // Deadzone threshold for sensors.
//  deadzone_min = -64;
//  deadzone_max = 64;
  calCnt = -1024;

  /* acceleration, velocity and position */
  for (int i = 0; i < 3; ++i)
    acc[i] = a[i][0] = a[i][1] = v[i][0] = v[i][1] = p[i][0] = p[i][1] = pRef[i] = sstate[X] = sstate[Y] = sstate[Z] = 0;

  /* regulation variable, can later be optimized (codewise) */
  for (int i = 0; i < 6; ++i) {
    Td[i] = 1;
    K[i] = 1;
    e[i] = eOld[i] = u[i] = 0;
  }

  /* Initial motor speed, range 0 - 100 */
  speed[LF] = motors[LF].getSpeed();
  speed[RF] = motors[RF].getSpeed();
  speed[LB] = motors[LB].getSpeed();
  speed[RF] = motors[RF].getSpeed();

  /* Internal speed variable */
  speed[LF] = speed[RF] = speed[LB] = speed[RB] = 50;
  debug_print = 0;
  Serial.println("Hoverregulator initialized");
}

/* Calibrates sensorvalues by sampling a number of times and then average on those values */
bool Hover::Calibrate() {
  if (calCnt > 512) return true;
  if (++calCnt > 0) {
    sstate[X] += sensor->acc.x;
    sstate[Y] += sensor->acc.y;
    sstate[Z] += sensor->acc.z;
    sstate[axisRo] += sensor->angleRoll;
    sstate[axisPi] += sensor->anglePitch;
  }
  if (calCnt == 512) {
    sstate[X] /= 512;
    sstate[Y] /= 512;
    sstate[Z] /= 512;
    sstate[axisRo] /= 512;
    sstate[axisPi] /= 512;
    Serial.println("Calibrated");
    return calCnt++;
  }
  return false;
}

void Hover::Regulate(void) {
  //timestampCurrent = micros();

  /* This must be checked! Very important that dt is the same every time! */
  //dt = timestampCurrent - timestamp;

  /* Sample values using filtration of low values. */
  int x = sensor->acc.x + sstate[X];
  //int y = sensor->acc.y + sstate[Y];
  //int z = sensor->acc.z + sstate[Z];
  
  int zone = 32;
  
  acc[X] += x > zone || x < -zone ? x : 0;
  //acc[Y] += y > zone || y < -zone ? y : 0;
  //acc[Z] += z > zone || z < -zone ? z : 0;
  
  if (++sampleCnt % 8 == 0) {
    /*
    // X
    a[X][1] = acc[X] / 8;
    v[X][1] = (v[X][0] + a[X][0] + (a[X][1] - a[X][0]) / 2);
    p[X][1] = (p[X][0] + v[X][0] + (v[X][1] - v[X][0]) / 2);
    
    // Y
    a[Y][1] = acc[Y] / 8;
    v[Y][1] = (v[Y][0] + a[Y][0] + (a[Y][1] - a[Y][0]) / 2);
    p[Y][1] = (p[Y][0] + v[Y][0] + (v[Y][1] - v[Y][0]) / 2);
    */
    
    // Z
    a[Z][1] = acc[Z] / 8;
    v[Z][1] = (v[Z][0] + a[Z][0] + (a[Z][1] - a[Z][0]) / 2);
    p[Z][1] = (p[Z][0] + v[Z][0] + (v[Z][1] - v[Z][0]) / 2);

    // Store values
    for (int i = 0; i < 3; ++i) {
      a[i][0] = a[i][1];
      v[i][0] = v[i][1];
      p[i][0] = p[i][1];
    }

    // Reset sampling
    sampleCnt = 0;
    acc[X] = acc[Y] = acc[Z] = 0; 
    
    // ===== PID  ===== // check overall performance?
    /*
    // X 
    e[axisX] = pRef[X] - p[X][1];
    I[axisX] = I[axisX] + Ti[axisX] * e[axisX];
    u[axisX] = K[axisX] * (e[axisX] + I[axisX] + Td[axisX] * (e[axisX] - eOld[axisX]));
    // Y
    e[axisY] = pRef[Y] - p[Y][1];
    I[axisY] = I[axisY] + Ti[axisY] * e[axisY];
    u[axisY] = K[axisY] * (e[axisY] + I[axisY] + Td[axisY] * (e[axisY] - eOld[axisY]));
    */
    // Z
    e[axisZ] = pRef[Z] - p[Z][1];
    u[axisZ] = K[axisZ] * (e[axisZ] + Td[axisZ] * (e[axisZ] - eOld[axisZ]));
    // Roll
    e[axisRo] = sensor->angleRoll - sstate[axisRo];
    u[axisRo] = K[axisRo] * (e[axisRo] + Td[axisRo] * (e[axisRo] - eOld[axisRo]));
    // Pitch
    e[axisPi] = sensor->anglePitch - sstate[axisPi];
    u[axisPi] = K[axisPi] * (e[axisPi] + Td[axisPi] * (e[axisPi] - eOld[axisPi]));
    // Yaw
    e[axisYa] = sensor->angleYaw; // compare to desired direction.
    u[axisYa] = K[axisYa] * (e[axisYa] + Td[axisYa] * (e[axisYa] - eOld[axisYa]));

    speed[LF] = max(min(speed[LF] + u[axisRo] - u[axisPi], debug_maxMotorEffect), 10);
    speed[RF] = max(min(speed[RF] - u[axisRo] - u[axisPi], debug_maxMotorEffect), 10);
    speed[LB] = max(min(speed[LB] + u[axisRo] + u[axisPi], debug_maxMotorEffect), 10);
    speed[RB] = max(min(speed[RB] - u[axisRo] + u[axisPi], debug_maxMotorEffect), 10);

    /* Debug prints to serial */
    if (++debug_print > 30) {
      debug_print = 0;
      Serial.println();
      Serial.print("Angles: ");
      Serial.print(e[axisRo]);
      Serial.print(", ");
      Serial.print(e[axisPi]);
      Serial.print(", ");
      Serial.println(e[axisYa]);
      Serial.println();
      
      Serial.print("U: ");
      Serial.print(u[axisRo]);
      Serial.print(", ");
      Serial.println(u[axisPi]);
    }

    // set engine speed value from 0 to 100
    if (false) {
      motors[LF].setSpeed(speed[LF]);
      motors[RF].setSpeed(speed[RF]);
      motors[LB].setSpeed(speed[LB]);
      motors[RB].setSpeed(speed[RB]);
    }
    
    /* store state */
    for (int i = 0; i < 6; ++i)
      eOld[i] = e[i];
    //timestamp = timestampCurrent;
  }
}

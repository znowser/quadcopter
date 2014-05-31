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
  debug_maxMotorEffect = 40;
  debug_minMotorEffect = 6;
    
  // Sample interator
  sampleCnt = 0;

  // Deadzone threshold for sensors.
//  deadzone_min = -64;
//  deadzone_max = 64;
  calCnt = -512;

  /* acceleration, velocity and position */
  for (int i = 0; i < 3; ++i)
    acc[i] = a[i][0] = a[i][1] = v[i][0] = v[i][1] = p[i][0] = p[i][1] = pRef[i] = sstate[X] = sstate[Y] = sstate[Z] = 0;

  /* regulation variable, can later be optimized (codewise) */
  for (int i = 0; i < 6; ++i) {
    Td[i] = 10.f;
    K[i] = 0.01f;
    e[i] = eOld[i] = u[i] = 0;
  }

  /* Initial motor speed, range 0 - 100 */
  /*
  speed[LF] = motors[LF].getSpeed();
  speed[RF] = motors[RF].getSpeed();
  speed[LB] = motors[LB].getSpeed();
  speed[RF] = motors[RF].getSpeed();
*/
  /* Internal speed variable */
  speed[LF] = speed[RF] = speed[LB] = speed[RB] = debug_maxMotorEffect;
  //debug_print = 0;
  //Serial.println("Hoverregulator initialized");
}

/* Calibrates sensorvalues by sampling a number of times and then average on those values */
bool Hover::Calibrate() {
  if (calCnt > 512) return true;
  if (++calCnt > 0) {
    //sstate[X] += sensor->acc.x;
    //sstate[Y] += sensor->acc.y;
    sstate[Z] += sensor->acc.z;
    sstate[axisRo] += sensor->angleRoll;
    sstate[axisPi] += sensor->anglePitch;
  }
  if (calCnt == 512) {
    //sstate[X] /= 512;
    //sstate[Y] /= 512;
    sstate[Z] /= 512;
    sstate[axisRo] /= 512;
    sstate[axisPi] /= 512;
    dt = micros();
    //Serial.println("Calibrated");
    return calCnt++;
  }
  return false;
}

void Hover::Regulate(void) {
  // Kill motors after 30 sec.
  if(micros() - dt > 10000000) {
    motors[LF].setSpeed(0);
    motors[RF].setSpeed(0);
    motors[LB].setSpeed(0);
    motors[RB].setSpeed(0);
    return;    
  }
  //timestampCurrent = micros();

  /* This must be checked! Very important that dt is the same every time! */
  //dt = timestampCurrent - timestamp;

  /* Sample values using filtration of low values. */
  //int x = sensor->acc.x - sstate[X];
  //int y = sensor->acc.y - sstate[Y];
  int z = sensor->acc.z - sstate[Z];
  
  int zone = 16;
  
  //acc[X] += x > zone || x < -zone ? x : 0;
  //acc[Y] += y > zone || y < -zone ? y : 0;
  acc[Z] += z > zone || z < -zone ? z : 0;
  
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
    
    // ===== PD  ===== // check overall performance?
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
    int scale = 16;
    e[axisZ] = (pRef[Z] - p[Z][1]) * scale;
    u[axisZ] = K[axisZ] * (e[axisZ] + Td[axisZ] * (e[axisZ] - eOld[axisZ]));
    u[axisZ] = max(min(u[axisZ], debug_minMotorEffect), debug_maxMotorEffect);
    //u[axisZ] = 5;
    int deadzone = 2;
    // Roll
    e[axisRo] = (sensor->angleRoll - sstate[axisRo])*scale;
    e[axisRo] = e[axisRo] > deadzone || e[axisRo] < -deadzone ? e[axisRo] : 0;
    u[axisRo] = K[axisRo] * (e[axisRo] + Td[axisRo] * (e[axisRo] - eOld[axisRo]));
    // Pitch
    e[axisPi] = (sensor->anglePitch - sstate[axisPi])*scale;
    e[axisPi] = e[axisPi] > deadzone || e[axisPi] < -deadzone ? e[axisPi] : 0;
    u[axisPi] = K[axisPi] * (e[axisPi] + Td[axisPi] * (e[axisPi] - eOld[axisPi]));
    // Yaw
    e[axisYa] = (sensor->angleYaw)*scale; // compare to desired direction.
    u[axisYa] = K[axisYa] * (e[axisYa] + Td[axisYa] * (e[axisYa] - eOld[axisYa]));

    speed[LF] = max(min(speed[LF] + u[axisRo] - u[axisPi] + u[axisZ], debug_maxMotorEffect), debug_minMotorEffect);
    speed[RF] = max(min(speed[RF] - u[axisRo] - u[axisPi] + u[axisZ], debug_maxMotorEffect), debug_minMotorEffect);
    speed[LB] = max(min(speed[LB] + u[axisRo] + u[axisPi] + u[axisZ], debug_maxMotorEffect), debug_minMotorEffect);
    speed[RB] = max(min(speed[RB] - u[axisRo] + u[axisPi] + u[axisZ], debug_maxMotorEffect), debug_minMotorEffect);

    /* Debug prints to serial */
    
    if (false && ++debug_print > 10) {
      debug_print = 0;
      Serial.println(u[axisZ]);
    }
    /*  Serial.print(e[axisRo]);
      Serial.print(", ");
      Serial.print(e[axisPi]);
      Serial.print(", ");
      Serial.println(e[axisYa]);
      Serial.println();
      
      Serial.print("Speed: ");
      Serial.print(speed[LF]);
      Serial.print(", ");
      Serial.print(speed[RF]);
      Serial.print(", ");
      Serial.print(speed[LB]);
      Serial.print(", ");
      Serial.print(speed[RB]);
    }
*/
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

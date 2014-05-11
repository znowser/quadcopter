#include "Hover.h"
#include <cmath>

Hover::Hover(Motor *motors, sensordata *sensor, float refAltitude) {
  init(motors, sensor, refAltitude);
}

void Hover::init(Motor *motors, sensordata *sensor, float refAltitude){
  this->motors = motors;
  this->sensor = sensor;
  this->time = micros();
  speed_lf = motors[leftfront].getSpeed();
  speed_rf = motors[rightfront].getSpeed();
  speed_lb = motors[leftback].getSpeed();
  speed_rb = motors[rightfront].getSpeed();
  refAltitude = 0.0;
  old_lfmh = 0.0;
  old_rfmh = 0.0;
  old_lfmv = 0.0;
  old_rfmv = 0.0;
  old_lfma = 0.0;
  old_rfma = 0.0;
  old_cbh = 0.0;
  old_cbv = 0.0;
  old_cba = 0.0;
  old_errorAltitude = 0.0;
  
  /* debug print out */
  Serial.print("Reference altitude: ");
  Serial.println(refAltitude);
}

void Hover::Regulate(void) {

  /* debug values, simplifies tests */
  int     debug_maxEngineEffect = 50;
  boolean debug_setEngineEffect = false;
  
  /* minimal interval between updates */
  unsigned long minUpdateInterval = 1000000;
  
  /* retrieve current time stamp */
  unsigned long currentTime = micros();
  
  /* calculate elapsed time (us) */
  unsigned long dt = currentTime - this->time;
  
  /* if dt exceeds minimal uptade interval, calculate new action */
  if (dt > minUpdateInterval){
  
    /* maximal error on altitude to put in the regulation algorithm */  
    float maxErrorAltitude = 0.1;
    
    /* naming scheme: position | [position2] | part | measure
     * ex: lfmh = left front motor height
     * ex: cbv = centre body velocity 
     */
    float lfmh;
    float rfmh;
    float lfmv;
    float rfmv;
    float lfma;
    float rfma;
    float cbh;
    float cbv;
    float cba;
    float errorAltitude;
     
    /* retrieve height */
    lfmh = 0.4 * tan(sensor->anglePitch);
    rfmh = 0.4 * tan(sensor->angleRoll);
    cbh = sensor->height;
    
    /* retrieve velocity */
    lfmv = (lfmh - old_lfmh) / dt;
    rfmv = (rfmh - old_rfmh) / dt;
    cbv = (cbh - old_cbh) / dt;

    /* retrieve acceleration */
    lfma = (lfmv - old_lfmv) / dt;
    rfma = (rfmv - old_rfmv) / dt;
    cba = (cbv - old_cbv) / dt;
    
    /* difference in meters from reference altitude (desired altitude) */
    errorAltitude = refAltitude - cbh;
    
    /* limit error to a maximal error altitude */
    errorAltitude = (abs(errorAltitude) > maxErrorAltitude) ? (errorAltitude < 0 ? -maxErrorAltitude : maxErrorAltitude) : errorAltitude;
    
    /* calculate how large alteration is taken on current speed. */
    float cP = 10;
    float cD = 100;
    float dBodySpeed = errorAltitude * cP + ((errorAltitude - old_errorAltitude) / dt) * cD;
    float cP_M = 10;
    float cD_M = 100;
    float dLeftFrontSpeed = lfmh * cP_M + ((lfmh - old_lfmh) / dt) * cD_M;
    float dRightFrontSpeed = rfmh * cP_M + ((rfmh - old_rfmh) / dt) * cD_M;
    
    // DEBUG, do all changes equaly on all engines
    dLeftFrontSpeed = 0.0;
    dRightFrontSpeed = 0.0;
    
    /* Set speed */
    speed_lf = speed_lf + dBodySpeed + dLeftFrontSpeed;
    speed_rf = speed_rf + dBodySpeed + dRightFrontSpeed;
    speed_lb = speed_lb + dBodySpeed - dRightFrontSpeed;
    speed_rb = speed_rb + dBodySpeed - dLeftFrontSpeed;
    
    /* safety check, never set more then 50% of engine effect */    
    speed_lf = min(speed_lf, debug_maxEngineEffect);
    speed_rf = min(speed_rf, debug_maxEngineEffect);
    speed_lb = min(speed_lb, debug_maxEngineEffect);
    speed_rb = min(speed_rb, debug_maxEngineEffect);
    
    /* set engine speed value from 0 to 100 */
    if (debug_setEngineEffect) {
      motors[leftfront].setSpeed(speed_lf);
      motors[rightfront].setSpeed(speed_rf);
      motors[leftback].setSpeed(speed_lb);
      motors[rightback].setSpeed(speed_rb);
    }
    
    /* store state */
    old_lfmh = lfmh;
    old_rfmh = rfmh;
    old_cbh = cbh;
    old_lfmv = lfmv;
    old_rfmv = rfmv;
    old_cbv = cbv;
    old_lfma = lfma;
    old_rfma = rfma;
    old_cba = cba;
    old_errorAltitude = errorAltitude;
    time = currentTime;
    
    /* Debug prints */
    /*Serial.println("=== new cycle ===");   
    Serial.print("P: ");
    Serial.println(errorAltitude * 10);
    Serial.print("D: ");
    Serial.println(((errorAltitude - old_errorAltitude) / dt) * 100);
    Serial.print("Altitude: ");
    Serial.println(cbh);
    Serial.print("Velocity: ");
    Serial.println(cbv);*/
    /*
    Serial.print("Acceleration: ");
    Serial.println(cba);
    */
    Serial.print("Current speed (lf, [rf, lb, rb]): ");
    Serial.println(speed_lf);
    /*
    Serial.println(speed_rf);
    Serial.println(speed_lb);
    Serial.println(speed_rb);
    */
    /*Serial.print("Last speed changes (body, lf, rf): ");
    Serial.println(dBodySpeed);
    *//*
    Serial.println(dLeftFrontSpeed);
    Serial.println(dRightFrontSpeed);
    */
  }  
}

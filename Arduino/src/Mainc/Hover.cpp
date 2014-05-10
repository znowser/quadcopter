#include "Hover.h"
#include <cmath>

Hover::Hover(Motor *motors, sensordata &sensor, float refHeight) {
  this->motors = motors;
  this->sensor = sensor;
  this->time = micros();
  this->refHeight = refHeight;
  old_lfmh = 0.0;
  old_rfmh = 0.0;
  old_lfmv = 0.0;
  old_rfmv = 0.0;
  old_lfma = 0.0;
  old_rfma = 0.0;
  old_cbh = 0.0;
  old_cbv = 0.0;
  old_cba = 0.0;
  old_errorHeight = 0.0;
}

/* Motor:
 * setSpeed(int speed), value ranging 
 * getSpeed(int percentage)
 */

void Hover::Regulate(void) {
  unsigned long currentTime = micros();
  unsigned long dt = currentTime - this->time;
  if (dt > 1000) {
    
    // lfmh = left front motor height
    float lfmh = 0.4 * tan(sensor.anglePitch);
    float rfmh = 0.4 * tan(sensor.angleRoll);
    float cbh = sensor.height;
    
    // velocity
    float lfmv = (lfmh - old_lfmh) / dt;
    float rfmv = (rfmh - old_rfmh) / dt;
    float cbv = (cbh - old_cbh) / dt;

    /* acceleration */
    float lfma = (lfmv - old_lfmv) / dt;
    float rfma = (rfmv - old_rfmv) / dt;
    float cba = (cbv - old_cbv) / dt;
    
    /* Difference in meters from refHeight */
    float errorHeight = (abs(refHeight - cbh) > 0.1) ? ((refHeight - cbh) < 0 ? -0.1 : 0.1) : refHeight - cbh;
    
    float speedDiff = errorHeight * 10 + ((errorHeight - old_errorHeight) / dt) * 100;
    
    int speed = motors[leftfront].getSpeed() + (int)(speedDiff * 1);
    
    
    Serial.print("P-reg: ");
    Serial.println(errorHeight * 10);
    Serial.print("D-reg: ");
    Serial.println(((errorHeight - old_errorHeight) / dt) * 100);
    
    Serial.print("Speed: ");
    Serial.println(speed);
    Serial.print("Speed diff: ");
    Serial.println(speedDiff);
    
    /* safety check, never set more then 50% of engine effect */    
    speed = min(speed, 50);
    
    /*
    motor[leftfront].setSpeed(speed);
    motor[rightfront].setSpeed(speed);
    motor[leftback].setSpeed(speed);
    motor[rightback].setSpeed(speed);
    */
    
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
    old_errorHeight = errorHeight;
    time = currentTime;
  }  
}

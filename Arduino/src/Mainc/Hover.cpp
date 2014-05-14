#include "Hover.h"
#include <cmath>

Hover::Hover(Motor *motors, sensordata *sensor, float refAltitude) {
  init(motors, sensor, refAltitude);
}

void Hover::init(Motor *motors, sensordata *sensor, float refAltitude) {
  /* Hardware */
  this->motors = motors;
  this->sensor = sensor;
  timestampPrint = timestampMotor = timestamp = micros();

  /* Initial motor speed, range 0 - 100 */
  speed_lf = motors[leftfront].getSpeed();
  speed_rf = motors[rightfront].getSpeed();
  speed_lb = motors[leftback].getSpeed();
  speed_rb = motors[rightfront].getSpeed();
  
  speed_lf = 50;
  speed_rf = 50;
  speed_lb = 50;
  speed_rb = 50;
  
  /*  */
  regI[ROLL] = 0;
  regI[PITCH] = 0;
  regI[YAW] = 0;
  eRollOld = 0;
  ePitchOld = 0;
}

void Hover::Regulate(void) {
  unsigned long timestampCurrent = micros();
  double dt = ((double)(timestampCurrent - timestamp)) / 1000000.0;
  double dtSqr = dt * dt;
  
  /* Debug prints to serial */
  if (timestampCurrent - timestampPrint > 2000000) {
    
    /*
    Serial.print("ang roll: ");
    Serial.println(sensor->angleRoll);
    Serial.print("ang pitch: ");
    Serial.println(sensor->anglePitch);
    */
    Serial.print("LF RF LB RB");
    Serial.println(speed_lf);
    Serial.println(speed_rf);
    Serial.println(speed_lb);
    Serial.println(speed_rb);
    
    timestampPrint = timestampCurrent;
  }

  /* debug values, simplifies tests */
  int     debug_maxMotorEffect = 100;
  boolean debug_setMotorEffect = false;

  /* Calculate new values for motors.
   * OBS The value affects PD calculation -> dt = 1000
   */
  if (timestampCurrent - timestampMotor >= 1000) {

    /* retrieve LEFT and RIGHT motors in FRONT altitudes relative centre position */
    /* LEFT and RIGHT motors in BACK will be opposite in altitude

       LF   RF         <- PID on front motors
         \ /
          C            <- PID on C position
    	 / \
       LB   RB         <- reversed effect from front motor PID

    */

    /* Calculate how large alteration is taken on current speed using basic PID!
     * Problem to tackle:
     * Need of different calibration for X, Y, Z?
     * Now to use the output to control position
     * Need for regulation on motor levels? (initially, very much yes!)
     */

    /* ===== leveling PID ===== */
    double Ts = (timestampCurrent - timestampMotor) / 1000000.0;
    double Ti = 100.0;  // Regulate integrator part of PID, set to inf to remove
    double Td = 10000.0; // Regulate derivative part of PID
    double K = 1.0;    // Level constant
    double uPitch, uRoll;  // output
    /* motor altitude error */
    double ePitch = motorDistanceCentre * tan(sensor->anglePitch);
    double eRoll = motorDistanceCentre * tan(sensor->angleRoll);
    /* Integrator part */
    regI[ROLL] = regI[ROLL] + (Ts / Ti) * eRoll;
    regI[PITCH] = regI[PITCH] + (Ts / Ti) * ePitch;
    /* output */
    uRoll = K * (eRoll + regI[ROLL] + (Td / Ts) * (eRoll - eRollOld));
    uPitch = K * (ePitch + regI[PITCH] + (Td / Ts) * (ePitch - ePitchOld));

    /* ===== derictional mapping ===== */
    // TODO how to use uX and uY

    /* Set speed, limited to a max value TODO have this mapped correct! */
    speed_lf = min(speed_lf + uRoll - uPitch, debug_maxMotorEffect);
    speed_rf = min(speed_rf - uRoll - uPitch, debug_maxMotorEffect);
    speed_lb = min(speed_lb + uRoll + uPitch, debug_maxMotorEffect);
    speed_rb = min(speed_rb - uRoll + uPitch, debug_maxMotorEffect);

    /* set engine speed value from 0 to 100 */
    if (debug_setMotorEffect) {
      motors[leftfront].setSpeed(speed_lf);
      motors[rightfront].setSpeed(speed_rf);
      motors[leftback].setSpeed(speed_lb);
      motors[rightback].setSpeed(speed_rb);
    }

    /* store state */
    eRollOld = eRoll;
    ePitchOld = ePitch;
    timestampMotor = timestampCurrent;
  }
  timestamp = timestampCurrent;
}

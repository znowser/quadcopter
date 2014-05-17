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

  a[X][0] = 0.f;
  a[X][1] = 0.f;
  a[Y][0] = 0.f;
  a[Y][1] = 0.f;
  a[Z][0] = 0.f;
  a[Z][1] = 0.f;
  v[X][0] = 0.f;
  v[X][1] = 0.f;
  v[Y][0] = 0.f;
  v[Y][1] = 0.f;
  v[Z][0] = 0.f;
  v[Z][1] = 0.f;
  p[X][0] = 0.f;
  p[X][1] = 0.f;
  p[Y][0] = 0.f;
  p[Y][1] = 0.f;
  p[Z][0] = 0.f;
  p[Z][1] = 0.f;

  /* Initial motor speed, range 0 - 100 */
  speed_lf = motors[LF].getSpeed();
  speed_rf = motors[RF].getSpeed();
  speed_lb = motors[LB].getSpeed();
  speed_rb = motors[RB].getSpeed();

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

void Hover::integrate(float v[3][2], float a[3][2]) {
  for (int i = 0; i < 3; ++i) {
    v[i][0] + a[i][0] + ((a[i][1] - a[i][0]) / 2.f) * dt;
  }
}

void Hover::Regulate(void) {
  long timestampCurrent = micros();
  dt = (timestampCurrent - timestamp) / 1000000.f;
  a[X][1] = sensor->acc.x * 0.00059936523f;
  a[Y][1] = sensor->acc.y * 0.00059936523f;
  a[Z][1] = sensor->acc.z * 0.00059936523f;
  if (micros() > 15000000) {
    //first integration
    //velocityx[1] = velocityx[0] + accelerationx[0] + ((accelerationx[1] - accelerationx[0])>>1)   
    //second integration
    //positionX[1] = positionX[0] + velocityx[0] + ((velocityx[1] - velocityx[0]) >> 1);
    integrate(v, a);
    integrate(p, v);
  }

  a[X][0] = a[X][1];
  a[Y][0] = a[Y][1];
  a[Z][0] = a[Z][1];

         /* Debug prints to serial */
  if (timestampCurrent - timestampPrint > 3000000) {
    if (micros() < 15000000) {
      Serial.println("Stabilizing...");
    } else {
      Serial.print("X: ");
      Serial.println(v[X][1]);
      Serial.print("Y: ");
      Serial.println(v[Y][1]);
      Serial.print("Z: ");
      Serial.println(v[Z][1]);
    }
    /*
    Serial.print("ang roll: ");
    Serial.println(sensor->angleRoll);
    Serial.print("ang pitch: ");
    Serial.println(sensor->anglePitch);

    Serial.print("LF RF LB RB");
    Serial.println(speed_lf);
    Serial.println(speed_rf);
    Serial.println(speed_lb);
    Serial.println(speed_rb);
    */
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
    double ePitch = sensor->anglePitch;
    double eRoll = sensor->angleRoll;
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
    speed_lf = speed_lf < 10 ? 10 : speed_lf;
    speed_rf = speed_rf < 10 ? 10 : speed_rf;
    speed_lb = speed_lb < 10 ? 10 : speed_lb;
    speed_rb = speed_rb < 10 ? 10 : speed_rb;

    /* set engine speed value from 0 to 100 */
    if (debug_setMotorEffect) {
      motors[LF].setSpeed(speed_lf);
      motors[RF].setSpeed(speed_rf);
      motors[LB].setSpeed(speed_lb);
      motors[RB].setSpeed(speed_rb);
    }

    /* store state */
    eRollOld = eRoll;
    ePitchOld = ePitch;
    timestampMotor = timestampCurrent;
  }
  timestamp = timestampCurrent;
}

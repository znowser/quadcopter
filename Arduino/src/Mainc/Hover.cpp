#include "Hover.h"
#include <cmath>

Hover::Hover(Motor *motors, sensordata *sensor, float refAltitude) {
	init(motors, sensor, refAltitude);
}

void Hover::init(Motor *motors, sensordata *sensor, float refAltitude) {
	/* Hardware */
	this->motors = motors;
	this->sensor = sensor;
	timestampMotor = timestamp = micros();	

	/* Desired position */
	positionDesired[X] = 0;
	positionDesired[Y] = 0;
	positionDesired[Z] = refAltitude;

	/* Starting position */
	positionRelative[X] = 0;
	positionRelative[Y] = 0;
	positionRelative[Z] = 0;

	/* Starting error in position */
	positionErrorPrev[X] = 0;
	positionErrorPrev[Y] = 0;
	positionErrorPrev[Z] = refAltitude;

	/* Initial motor speed, range 0 - 100 */
	speed_lf = motors[leftfront].getSpeed();
	speed_rf = motors[rightfront].getSpeed();
	speed_lb = motors[leftback].getSpeed();
	speed_rb = motors[rightfront].getSpeed();

	/* Calibration values */
	angleCalibrateCount = 0;
	angleOffset[ROLL] = 0;
	angleOffset[PITCH] = 0;
	angleOffset[YAW] = 0;

	/*  */
	regI[X] = 0;
	regI[Y] = 0;
	regI[Z] = 0;
	regIM[0] = 0;
	regIM[1] = 0;
	lmeOld = 0;
	lmeOld = 0;

	/* Acceleration buffer values */
	accelerationBufferCounter = 0;
}

/* Buffers sensor values, angle buffer is used for the angle offset, 
 * the acceleration buffer is used to avoid spikes when calculating 
 * position from acceleration.
 */

bool Hover::Calibrate(void)  {

	if (angleCalibrateCount == sampleAngle) return true;

	// Buffer up acceleration values (sampleAcceleration / 3 times)
	accelerationBuffer[accelerationBufferCounter++] = sensor->acc.x;
	accelerationBuffer[accelerationBufferCounter++] = sensor->acc.y;
	accelerationBuffer[accelerationBufferCounter] = sensor->acc.z;
	accelerationBufferCounter = ++accelerationBufferCounter % sampleAcceleration;

	// Buffer up angle values (sampleAngle / 3 times)
	if (angleCalibrateCount == sampleAngle) {
		for (int i = 0; i < sampleAngle; ++i) {
			angleOffset[ROLL] += angleCalibrate[i];
			angleOffset[PITCH] += angleCalibrate[++i];
			angleOffset[YAW] += angleCalibrate[++i];
		}
		angleOffset[ROLL] /= (sampleAngle / 3);
		angleOffset[PITCH] /= (sampleAngle / 3);
		angleOffset[YAW] /= (sampleAngle / 3);
		return true;
	} else {
		angleCalibrate[angleCalibrateCount++] = sensor->angleRoll;
		angleCalibrate[angleCalibrateCount++] = sensor->anglePitch;
		angleCalibrate[angleCalibrateCount++] = sensor->angleYaw;
	}
	return false;
}

void Hover::Regulate(void) {
	unsigned long timestampCurrent = micros();
	unsigned long dt = timestampCurrent - timestamp;
	unsigned long dtSqr = dt * dt;
	float accX = 0, accY =0, accZ = 0;

	/* Buffer up acceleration values */
	accelerationBuffer[accelerationBufferCounter] = sensor->acc.x;
	accelerationBuffer[++accelerationBufferCounter] = sensor->acc.y;
	accelerationBuffer[++accelerationBufferCounter] = sensor->acc.z;
	accelerationBufferCounter = ++accelerationBufferCounter % sampleAcceleration;

	/* Calculate average acceleration from buffer */
	for (int i = 0; i < sampleAcceleration; ++i) {
		accX += accelerationBuffer[i];
		accY += accelerationBuffer[++i];
		accZ += accelerationBuffer[++i];
	}

	/* Calculate relative position change */
	positionRelative[X] += ((3 * accX) / sampleAcceleration) * dtSqr;
	positionRelative[Y] += ((3 * accY) / sampleAcceleration) * dtSqr;
	positionRelative[Z] += ((3 * accZ) / sampleAcceleration) * dtSqr;

	/* Debug prints to serial */	
	if (dt > 1000000) {
		Serial.print("X: ");
		Serial.println(positionRelative[X]);
		Serial.print("Y: ");
		Serial.println(positionRelative[Y]);
		Serial.print("Z: ");
		Serial.println(positionRelative[Z]);
		Serial.print("LF, RF, LB, RB speed: ");
		Serial.println(speed_lf);
		Serial.println(speed_rf);
		Serial.println(speed_lb);
		Serial.println(speed_rb);
	}

	/* debug values, simplifies tests */
	int     debug_maxMotorEffect = 50;
	boolean debug_setMotorEffect = false;

	/* Calculate new values for motors. 
	 * OBS The value affects PD calculation -> dt = 1000
	 */
	if (timestampCurrent - timestampMotor >= 1000){

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

		/* ===== position PID  ===== */
		float Ts = timestampCurrent - timestampMotor;
		float Ti = 100.0;  // Regulate integrator part of PID, set to inf to remove
		float Td = 1000.0; // Regulate derivative part of PID
		float K = 10.0;    // Level constant
		float uX, uY, uZ;  // output
		/* centre body position error */
		float eX = positionDesired[X] - positionRelative[X];
		float eY = positionDesired[Y] - positionRelative[Y];
		float eZ = positionDesired[Z] - positionRelative[Z];
		/* Integrator part */
		regI[X] = regI[X] + (Ts / Ti) * eX;
		regI[Y] = regI[Y] + (Ts / Ti) * eY;
		regI[Z] = regI[Z] + (Ts / Ti) * eZ;
		/* output */
		uX = K * (eX + regI[X] + (Td / Ts) * (eX - positionErrorPrev[X]));
		uY = K * (eY + regI[Y] + (Td / Ts) * (eY - positionErrorPrev[Y]));
		uZ = K * (eZ + regI[Z] + (Td / Ts) * (eZ - positionErrorPrev[Z]));

		/* ===== leveling PID ===== */
		float Tli = 100.0;  // Regulate integrator part of PID, set to inf to remove
		float Tld = 1000.0; // Regulate derivative part of PID
		float Kl = 10.0;    // Level constant
		float uLM, uRM;  // output
		/* motor altitude error */
		float lme = motorDistanceCentre * tan(sensor->anglePitch);
		float rme = motorDistanceCentre * tan(sensor->angleRoll);
		/* Integrator part */
		regIM[0] = regIM[0] + (Tls / Tli) * lme;
		regIM[1] = regIM[1] + (Tls / Tli) * rme;
		/* output */
		uLM = Kl * (lme + regIM[0] + (Tld / Tls) * (lme - lmeOld));
		uRM = Kl * (rme + regIM[1] + (Tld / Tls) * (rme - rmeOld));

		/* ===== derictional mapping ===== */
		// TODO how to use uX and uY

		/* Set speed, limited to a max value TODO have this mapped correct! */
		speed_lf = min(speed_lf + uZ + uLM, debug_maxMotorEffect);
		speed_rf = min(speed_rf + uZ + uRM, debug_maxMotorEffect);
		speed_lb = min(speed_lb + uZ - uRM, debug_maxMotorEffect);
		speed_rb = min(speed_rb + uZ - uLM, debug_maxMotorEffect);

		/* set engine speed value from 0 to 100 */
		if (debug_setMotorEffect) {
			motors[leftfront].setSpeed(speed_lf);
			motors[rightfront].setSpeed(speed_rf);
			motors[leftback].setSpeed(speed_lb);
			motors[rightback].setSpeed(speed_rb);
		}

		/* store state */
		lmeOld = lme;
		rmeOld = rme;
		positionErrorPrev[X] = eX;
		positionErrorPrev[Y] = eY;
		positionErrorPrev[Z] = eZ;
		timestampMotor = timestampCurrent;
	}  
	timestamp = timestampCurrent;
}

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

	leftFrontMotorZOld = 0;
	rightFrontMotorZOld = 0;

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
		Serial.print("Left motor simulated speed: ");
		Serial.println(speed_lf);
	}

	/* debug values, simplifies tests */
	int     debug_maxMotorEffect = 50;
	boolean debug_setMotorEffect = false;

	/* Calculate new values for motors. 
	 * OBS The value affects PD calculation -> dt = 1000
	 */
	if (timestampCurrent - timestampMotor >= 1000){

		/* TODO map motors to axis correct */
		float leftFrontMotorZ;
		float rightFrontMotorZ;
		float posError[3];

		/* retrieve motor altitudes relative centre position */
		leftFrontMotorZ = motorDistanceCentre * tan(sensor->anglePitch);
		rightFrontMotorZ = motorDistanceCentre * tan(sensor->angleRoll);

		/* relative altitude difference from desired altitude */
		posError[Z] = positionDesired[Z] - positionRelative[Z];

		/* calculate how large alteration is taken on current speed. */
		// OBS short-cut since dt = 1000 then the d-part is shortened
		float regPD = posError[Z] * 10.0 + (posError[Z] - positionErrorPrev[Z]) * 0.1;
		float regPDLM = leftFrontMotorZ * 10.0 + (leftFrontMotorZ - leftFrontMotorZOld) * 0.1;
		float regPDRM = rightFrontMotorZ * 10.0 + (rightFrontMotorZ - rightFrontMotorZOld) * 0.1;

		// DEBUG, do all changes equaly on all engines
		regPDLM = 0.0;
		regPDRM = 0.0;

		/* Set speed, limited to a max value TODO have this mapped correct! */
		speed_lf = min(speed_lf + regPD + regPDLM, debug_maxMotorEffect);
		speed_rf = min(speed_rf + regPD + regPDRM, debug_maxMotorEffect);
		speed_lb = min(speed_lb + regPD - regPDRM, debug_maxMotorEffect);
		speed_rb = min(speed_rb + regPD - regPDLM, debug_maxMotorEffect);

		/* set engine speed value from 0 to 100 */
		if (debug_setMotorEffect) {
			motors[leftfront].setSpeed(speed_lf);
			motors[rightfront].setSpeed(speed_rf);
			motors[leftback].setSpeed(speed_lb);
			motors[rightback].setSpeed(speed_rb);
		}

		/* store state */
		leftFrontMotorZOld = leftFrontMotorZ;
		rightFrontMotorZOld = rightFrontMotorZ;
		positionErrorPrev[X] = posError[X];
		positionErrorPrev[Y] = posError[Y];
		positionErrorPrev[Z] = posError[Z];
		timestampMotor = timestampCurrent;
	}  
	timestamp = timestampCurrent;
}

#ifndef HOVER_H
#define HOVER_H

#include "Motor.h"
#include "SensorDataStruct.h"

#define sampleAngle 24        // OBS must be divisible by three
#define sampleAcceleration 9  // OBS must be divisible by three

#define motorDistanceCentre 0.4

/* OBSERVER!
 * How to run this regulator:
 * 1. Run init
 * 2. Run Calibrate until it returns true
 * 3. Run Regulate as many times regulation is needed
 */


class Hover {
private:

	enum Angle { ROLL = 0, PITCH = 1, YAW = 2 };
	enum Position { X = 0, Y = 1, Z = 2 };

	/* Hardware */
	Motor *motors;
	sensordata *sensor;

	/* last call and last motor update */
	unsigned long timestamp, timestampMotor;

	/* Internal position from start*/
	float positionRelative[3];
	float positionDesired[3];
	float positionErrorPrev[3];

	/* calibration of angle */
	int angleCalibrateCount;
	float angleCalibrate[sampleAngle]; // # sample * (roll, pitch, yaw)

	/* Angle offset, sets during calibration or zero */
	float angleOffset[3]; //roll, pitch, yaw

	/* to prevent freak values, acceleration from sensors are averages out on the 8 last values */
	int accelerationBufferCounter;
	float accelerationBuffer[sampleAcceleration]; // # sample * (x, y, z)

	/* Integrating part of PID */
	float regI[3], regIM[2];
	/* old motor level error, (left and right front motor) */
	float lmeOld, rmeOld;

	/* Internal speed when debugging without real engines */
	float speed_lf;
	float speed_rf;
	float speed_lb;
	float speed_rb;	

public:
	Hover() {};
	Hover(Motor *motors, sensordata *sensor, float refAltitude);
	void init(Motor *motors, sensordata *sensor, float refAltitude);
	bool Calibrate(void);
	void Regulate(void);
};

#endif

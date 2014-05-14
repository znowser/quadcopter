#ifndef HOVER_H
#define HOVER_H

#include "Motor.h"
#include "SensorDataStruct.h"

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
	unsigned long timestamp, timestampMotor, timestampPrint;

	/* Integrating part of PID */
	float regI[3];
	/* old motor level error, (left and right front motor) */
	float eRollOld, ePitchOld;

	/* Internal speed when debugging without real engines */
	float speed_lf;
	float speed_rf;
	float speed_lb;
	float speed_rb;	

public:
	Hover(Motor *motors, sensordata *sensor, float refAltitude);
	void init(Motor *motors, sensordata *sensor, float refAltitude);
	void Regulate(void);
};

#endif

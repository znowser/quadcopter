#ifndef HOVER_H
#define HOVER_H

#include "Motor.h"
#include "SensorDataStruct.h"

#define sampleAngle 24        // OBS must be divisible by three
#define sampleAcceleration 24 // OBS must be divisible by three

class Hover {
private:

	float average(float buffer, int cnt);

	/* Hardware */
	Motor *motors;
	sensordata *sensor;

	/* Reference altitude or desired altitude from starting position */
	float refAltitude;
	unsigned long time;

	/* Internal position from start*/
	float position[3];

	/* calibration of angle */
	int angleCalibrateCount;
	float angleCalibrate[sampleAngle]; // # sample * (roll, pitch, yaw)

	/* Angle offset, sets during calibration or zero */
	float angleOffset[3]; //roll, pitch, yaw

	/* to prevent freak values, acceleration from sensors are averages out on the 8 last values */
	int accelerationBufferCounter;
	float accelerationBuffer[sampleAcceleration]; // # sample * (x, y, z)

	float old_lfmh;
	float old_rfmh;
	float old_lfmv;
	float old_rfmv;
	float old_lfma;
	float old_rfma;
	float old_cbh;
	float old_cbv;
	float old_cba;
	float old_errorAltitude;

	/* Internal speed when debugging without real engines */
	float speed_lf;
	float speed_rf;
	float speed_lb;
	float speed_rb;

	enum Angle { Roll = 0, Pitch = 1, Yaw = 2 };
	enum Position { x = 0, y = 1, z = 2 };

public:
	Hover() {};
	Hover(Motor *motors, sensordata *sensor, float refAltitude);
	void init(Motor *motors, sensordata *sensor, float refAltitude);
	bool Calibrate(void);
	void SimpleRegulate(void);
	void Regulate(void);
};

#endif

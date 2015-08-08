#ifndef SENSORDATA_STRUCT
#define SENSORDATA_STRUCT

#include "lib/Arduino/Arduino.h"
#include "FREEIMU/helper_3dmath.h"

#define JS_BUTTON_CROSS 14
#define JS_BUTTON_TRIANGLE 12
#define JS_BUTTON_SQUARE 15
#define JS_BUTTON_CIRCLE 13

#define JS_BUTTON_UP 4
#define JS_BUTTON_DOWN 6
#define JS_BUTTON_LEFT 7
#define JS_BUTTON_RIGHT 5

#define JS_BUTTON_R1 11
#define JS_BUTTON_L1 10
#define JS_BUTTON_R2 9
#define JS_BUTTON_L2 8
#define JS_BUTTON_R3 2
#define JS_BUTTON_L3 1
#define JS_BUTTON_SELECT 0
#define JS_BUTTON_START 3
#define JS_BUTTON_PS3 16

#define NUM_OF_BUTTONS 17
#define PS3CONTROLLER_DEADZONE 30

#define LF 0  //left front
#define RF 1  //right front
#define LB 2  //left back
#define RB 3  //right back

#define CELL1 0
#define CELL2 1
#define CELL3 2

struct ps3Controller {
  int button[NUM_OF_BUTTONS];
  int stick1_x;
  int stick1_y;
  int stick2_x;
  int stick2_y;
};

struct constants {
	float c1;
	float c2;
	float dryAirGasConst = 287.058;
	float gravAcc = 9.82327;
};

struct sensordata {
	
	//flag that indicates whether the engines are allowed to run or not
	bool stop;
	
	/*Angles*/
	int motorSpeed[4];
	int cellVoltage[3];
	int16_t acc[3];
	//int16_t gyro[3];
	float gyro[3];
	/*
	int angleYaw;
	int anglePitch;
	int angleRoll;
	Accelerations
	VectorInt16 acc;
	*/
	float temperature;
	float pressure;
	int32_t rawPressure;
	float height;
	ps3Controller ps3;
	constants alt;
};

#endif

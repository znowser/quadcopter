/*
 * Regulator.cpp
 *
 * Created: 2015-07-29 12:35:52
 *  Author: Erik
 */ 
#include "PID.h"


void PID::init(float P, float I, float D){
	this->P = P;
	this->I = I;
	this->D = D;
}

/* Returns how the power should be split between the two sides of the quad
* in the range 0 - 200 where 100 is even power to both sides.
*/
float PID::regulate(float angle, float desiredAngle){
	float res = 0;
	float error = angle - desiredAngle;
	
	
	res = P * error;
	
	if(res < -1.0)
		res = -1.0;
	else if(res > 1.0)
		res = 1.0;
	
	return res;
}
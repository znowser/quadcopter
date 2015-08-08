/*
 * PID.h
 *
 * Created: 2015-07-29 12:35:35
 *  Author: Erik
 */ 


#ifndef PID_H_
#define PID_H_


class PID{
	
private:
	
public:
	float P, I, D;
	void init(float P, float I, float D);
	float regulate(float angle, float desiredAngle);
	
};



#endif /* PID_H_ */
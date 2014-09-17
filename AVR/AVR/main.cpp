/*
 * AVR.cpp
 *
 * Created: 2014-09-07 18:50:52
 *  Author: Erik
 */ 

#include "lib/Arduino/Arduino.h"
#include "Motor/Motor.h"

#define afdf 1
int main(void)
{
	//initial all pins according to arduino mstandard
	init();
	Motor m1 = Motor(13);
	//m1.setSpeed(150);

	//m1.setSpeed(10);
	//servo.attach()
	
	bool output = true;
	
	pinMode(16, OUTPUT);
	pinMode(15, OUTPUT);
	//pinMode(14, OUTPUT);
	//pinMode(13, OUTPUT);
    while(1)
    {
		digitalWrite(16, output);
		digitalWrite(15, output);
		//digitalWrite(14, output);
		//digitalWrite(13, output);
		
		delay(2000);
		output = !output;
    }
}
/*
 * AVR.cpp
 *
 * Created: 2014-09-07 18:50:52
 *  Author: Erik
 */ 

#include "lib/Arduino/Arduino.h"
#include "Motor/Motor.h"

int main(void)
{
	Motor m1 = Motor(14);
	m1.init(14);
	m1.setSpeed(150);

	//m1.setSpeed(10);
	//servo.attach()
	
	pinMode(16, OUTPUT);
	pinMode(15, OUTPUT);
	pinMode(14, OUTPUT);
	pinMode(13, OUTPUT);
    while(1)
    {
		digitalWrite(16, LOW);
		digitalWrite(15, LOW);
		//digitalWrite(14, HIGH);
		digitalWrite(13, LOW);
    }
}
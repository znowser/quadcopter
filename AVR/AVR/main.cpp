/*
 * main.cpp
 *
 * Created: 2014-09-07 18:50:52
 *  Author: Erik
 */ 

#include "lib/Arduino/Arduino.h"

int main(void) {
	Serial1.begin(115200);

	Serial1.println("Beginning test");
	while(true){
		Serial1.println(" Test print ");
		delay(200);
	}
}
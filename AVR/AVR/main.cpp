/*
 * main.cpp
 *
 * Created: 2014-09-07 18:50:52
 *  Author: Erik
 */ 

#include "lib/Arduino/Arduino.h"

int main(void) {
	//initial all pins according to arduino standard !! MUST BE DONE !!
	init();
	
	Serial1.begin(115200);
	Serial1.println("Beginning test");
	
	unsigned long itr = 0;
	unsigned long start = micros();
	while(true){
		if(itr % 100000 == 0){
			Serial1.print("Iterations per second: ");
			Serial1.println(itr / ((micros() - start) / 1000000));
		}
		//prevent overflow error
		if(++itr == 0)
			start = micros();
	}
	return 0;
}
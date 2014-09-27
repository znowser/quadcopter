/*
 * main.cpp
 *
 * Created: 2014-09-07 18:50:52
 *  Author: Erik
 */ 

#include "lib/Arduino/Arduino.h"
#include "FreeIMU/MS561101BA.h"


#define MOVAVG_SIZE 32

float movavg_buff[MOVAVG_SIZE];
int movavg_i=0;

const float sea_press = 1013.25;
float press, temperature;

float getAltitude(float press, float temp);
void pushAvg(float val);
float getAvg(float * buff, int size);
int main(void) {
//	Wire.begin();
	Serial1.begin(115200);
	//delay(1000);

	Serial1.println("Beginning ");
	// Suppose that the CSB pin is connected to GND.
	// You'll have to check this on your breakout schematics
	//MS561101BA baro = MS561101BA(MS561101BA_ADDR_CSB_LOW);
	//delay(100);
		
	// populate movavg_buff before starting loop
	//for(int i=0; i<MOVAVG_SIZE; i++) {
	//	movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
	//}
	
	while(true){
		Serial1.print(" temp: ");
		/*temperature = baro.getTemperature(MS561101BA_OSR_4096);
		Serial1.print(temperature);
		Serial1.print(" degC pres: ");
	
		press = baro.getPressure(MS561101BA_OSR_4096);
		pushAvg(press);
		press = getAvg(movavg_buff, MOVAVG_SIZE);
		Serial1.print(press);
		Serial1.print(" mbar altitude: ");
		Serial1.print(getAltitude(press, temperature));
		Serial1.println(" m");
		*/
	}
}


float getAltitude(float press, float temp) {
	//return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
	return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void pushAvg(float val) {
	movavg_buff[movavg_i] = val;
	movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}

float getAvg(float * buff, int size) {
	float sum = 0.0;
	for(int i=0; i<size; i++) {
		sum += buff[i];
	}
	return sum / size;
}


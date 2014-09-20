/*
 * main.cpp
 *
 * Created: 2014-09-07 18:50:52
 *  Author: Erik
 */ 

#include "lib/Arduino/Arduino.h"
#include "SensorDataStruct.h"
#include "Motor/Motor.h"
#include "cellVoltage/cellVoltage.h"


int main(void)
{
	//initial all pins according to arduino standard
	init();
	Serial1.begin(115200);
	Motor motor[4];
	CellVoltage battery[3];
	//*==========Init Motors=============*/
	//Hardware mapping of motors
	/*
	motor[LF].init(13);
	motor[RF].init(14);
	motor[LB].init(15);
	motor[RB].init(16);
	//minimum wait time for arming
	delay(8000);
	*/
	/*==================================*/
	/*=====Init battery cells ==========*/
	//Hardware mapping of battery cells
	battery[CELL1].init(ADC0);
	battery[CELL2].init(ADC1);
	battery[CELL3].init(ADC2);
	/*==================================*/

	int speed = 0;
	bool dir = false;	
    while(true){
		Serial1.print("Battery level: ");
		Serial1.print(battery[CELL1].getVoltage());
		Serial1.print(" ");
		Serial1.print(battery[CELL2].getVoltage());
		Serial1.print(" ");
		Serial1.println(battery[CELL3].getVoltage());
		
		/*
		motor[LF].setSpeed(speed);
		motor[RF].setSpeed(speed);
		motor[LB].setSpeed(speed);
		motor[RB].setSpeed(speed);
		
		Serial1.print("Speed ");
		Serial1.println(speed);
		if(speed >= 50 || speed <= 0)
			dir = !dir;
		
		dir?++speed:--speed;
		*/
				
		delay(100);
    }
}
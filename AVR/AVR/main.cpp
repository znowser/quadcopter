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
#include "FreeIMU/MPU6050Abstraction.h"
#include "FreeIMU/MS561101BA.h"
#include "HoverRegulator/Hover.h"

//change this variable to true if you want to turn on the regulator, Torbj�rn.
const bool regulator_activated = true;
const float sea_press = 1013.25;

float getAltitude(float press, float temp);
void updateSensorValues(sensordata &sensorData, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]);
int main(void)
{
	//initial all pins according to arduino standard
	init();
	//setup serial connection
	Serial1.begin(115200);
	
	Motor motor[4];
	CellVoltage battery[3];
	sensordata sensorData;
	float ypr[3];
	/*==========Init Sensors============*/
	//init gyro and magnetic field
	MPUAbstraction mpu = MPUAbstraction();
	//barometer and temperature
	MS561101BA baro = MS561101BA(MS561101BA_ADDR_CSB_LOW);
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

	float press, temp;
	int speed = 0;
	bool dir = false;	
	
	/*==========Hover regulator=============*/
	float refVal[6] = { 0, 0, 1.0, 0, 0, 0 };
	Hover regulator(motor, &sensorData, refVal);
	
    while(true){
		//check if there is new sensordata to recieve from the sensor card
		if (mpu.readYawPitchRoll(ypr, sensorData.acc)) {
			//update sensor struct
			updateSensorValues(sensorData, motor, battery, baro, ypr);
		/*
			Serial1.print("Battery level: ");
			Serial1.print(battery[CELL1].getVoltage());
			Serial1.print(" ");
			Serial1.print(battery[CELL2].getVoltage());
			Serial1.print(" ");
			Serial1.println(battery[CELL3].getVoltage());
		
			Serial1.print("Baro Reading ");
			Serial1.print(press = baro.getPressure(MS561101BA_OSR_4096));
			Serial1.print(" ");
			Serial1.println(temp = baro.getTemperature(MS561101BA_OSR_4096));
			Serial1.print("Esitmated height: ");
			Serial1.println(getAltitude(press, temp));
		
			Serial1.print("Yaw ");
			Serial1.print(sensorData.gyro[0]);
			Serial1.print(" Pitch ");
			Serial1.print(sensorData.gyro[1]);
			Serial1.print(" Roll ");
			Serial1.println(sensorData.gyro[2]);
		*/
		//  if (regulator_activated)
		//	regulator.Regulate();
        }
		
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

float getAltitude(float press, float temp) {
	//return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
	return ((pow((sea_press / press), 1 / 5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void updateSensorValues(sensordata &sensorData, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]) {
	//temperature and pressure cannot be read directly after each other, it
	//must be a small delay between the two functioncalls.
	sensorData.temperature = baro.getTemperature(MS561101BA_OSR_4096);
	//convert from radians to degrees
	sensorData.gyro[2] = ypr[0]  * 180 / M_PI;
	sensorData.gyro[1] = ypr[1]  * 180 / M_PI;
	sensorData.gyro[0] = ypr[2]  * 180 / M_PI;
	sensorData.cellVoltage[CELL1] = battery[CELL1].getVoltage();
	sensorData.cellVoltage[CELL2] = battery[CELL2].getVoltage();
	sensorData.cellVoltage[CELL3] = battery[CELL3].getVoltage();
	//update motor speed
	sensorData.motorSpeed[LF] = motor[LF].getSpeed();
	sensorData.motorSpeed[RF] = motor[RF].getSpeed();
	sensorData.motorSpeed[LB] = motor[LB].getSpeed();
	sensorData.motorSpeed[RB] = motor[RB].getSpeed();
	//get pressure, cannot be done directly after getTemperature, The sensorcard need
	//a small delay between the function calls.
	sensorData.pressure = baro.getPressure(MS561101BA_OSR_4096);
	sensorData.height = getAltitude(sensorData.pressure, sensorData.temperature);
}
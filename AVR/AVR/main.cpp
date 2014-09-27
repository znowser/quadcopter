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

#define MOVAVG_SIZE 256

float movavg_buff[MOVAVG_SIZE];
int movavg_i = 0;
const float sea_press = 1013.25;
float press, temperature;
float getAltitude(float press, float temp);
void pushAvg(float val);
float getAvg(float *buff, int size);

const bool regulator_activated = true;
void updateSensorValues(sensordata &sensor, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]);

int main(void)
{
	//initial all pins according to arduino standard
	init();
	//setup serial connection
	Serial1.begin(115200);
	
	Motor motor[4];
	CellVoltage battery[3];
	sensordata sensor;
	float ypr[3];
	/*==========Init Sensors============*/
	//init gyro and magnetic field
	//MPUAbstraction mpu = MPUAbstraction();
	Wire.begin();
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
	Hover regulator(motor, &sensor, refVal);
	delay(1500);
	
	// populate movavg_buff before starting loop
	for(int i=0; i<MOVAVG_SIZE; i++) {
		movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
	}
	
	int cnt = 0;
	
    while(true){
		//check if there is new sensordata to recieve from the sensor card
		//if (mpu.readYawPitchRoll(ypr, sensor.acc)) {
		//	//update sensor struct
		
		temperature = baro.getTemperature(MS561101BA_OSR_4096);
		//Serial1.print(temperature);
		//Serial1.print(" degC pres: ");
		
		press = baro.getPressure(MS561101BA_OSR_4096);
		pushAvg(press);
		press = getAvg(movavg_buff, MOVAVG_SIZE);
		//Serial1.print(press);
		//Serial1.print(" mbar altitude: ");
		if (!(++cnt % 128))
			Serial1.println(getAltitude(press, temperature));
		
		updateSensorValues(sensor, motor, battery, baro, ypr);
		
//				Serial1.println(sensor.height);
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
			Serial1.print(sensor.gyro[0]);
			Serial1.print(" Pitch ");
			Serial1.print(sensor.gyro[1]);
			Serial1.print(" Roll ");
			Serial1.println(sensor.gyro[2]);
		*/
		//  if (regulator_activated)
		//	regulator.Regulate();
       // }
		
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
    }
}

void updateSensorValues(sensordata &sensor, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]) {
	static byte batInterval = 255;
	// Battery Cell 1
	if (!++batInterval)
		sensor.cellVoltage[CELL1] = battery[CELL1].getVoltage();
	// Gyro
	/*
	sensor.gyro[2] = ypr[0];
	sensor.gyro[1] = ypr[1];
	sensor.gyro[0] = ypr[2];
	*/
	// Battery Cell 2
	if (!batInterval)
		sensor.cellVoltage[CELL2] = battery[CELL2].getVoltage();	
	// Motor	
	sensor.motorSpeed[LF] = motor[LF].getSpeed();
	sensor.motorSpeed[RF] = motor[RF].getSpeed();
	sensor.motorSpeed[LB] = motor[LB].getSpeed();
	sensor.motorSpeed[RB] = motor[RB].getSpeed();
	// Battery Cell 3
	if (!batInterval)
		sensor.cellVoltage[CELL3] = battery[CELL3].getVoltage();
}

float getAltitude(float press, float temp) {
	//return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
	//return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
	return ((pow((sea_press / press), 0.1901697808) - 1.0) * (temp + 273.15)) / 0.0065;
}

void pushAvg(float val) {
	movavg_buff[movavg_i] = val;
	movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}

float getAvg(float * buff, int size) {
	float sum = 0.0;
	for(int i = 0; i<size; i++)
	sum += buff[i];
	return sum / size;
}


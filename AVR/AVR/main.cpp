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

const bool regulator_activated = true;
float getAltitude(sensordata &sensor, float press);
void updateSensorValues(sensordata &sensor, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]);
void initSensorValues(sensordata &sensor, MS561101BA &baro);
void initAltitudeMeasurement(sensordata &sensor, float ref_pres, float ref_temp);

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
	MPUAbstraction mpu = MPUAbstraction();
	//barometer and temperature
	MS561101BA baro = MS561101BA(MS561101BA_ADDR_CSB_LOW);
	//*==========Init Motors=============*/
	//Hardware mapping of motors
	motor[LF].init(13);
	motor[RF].init(14);
	motor[LB].init(15);
	motor[RB].init(16);
	//minimum wait time for arming
	delay(8000);
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
	initSensorValues(sensor, baro);
	static byte printAltInterval = 0;
    while(true){
		//check if there is new sensordata to recieve from the sensor card
		if (mpu.readYawPitchRoll(ypr, sensor.acc)) {
		//	//update sensor struct
			updateSensorValues(sensor, motor, battery, baro, ypr);
			if (!(++printAltInterval % 8))
				Serial1.println(sensor.height);
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

void updateSensorValues(sensordata &sensor, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]) {
	static byte batInterval = 255;
	// Battery Cell 1
	if (!++batInterval)
		sensor.cellVoltage[CELL1] = battery[CELL1].getVoltage();
	// Gyro
	sensor.gyro[2] = ypr[0];
	sensor.gyro[1] = ypr[1];
	sensor.gyro[0] = ypr[2];
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
	// Barometer
	sensor.pressure = baro.getPressure(MS561101BA_OSR_4096);
	sensor.height = getAltitude(sensor, sensor.pressure);
}

float getAltitude(sensordata &sensor, float pressure) {
	return sensor.alt.c1*(sensor.alt.c2 - log(pressure));
}

void initSensorValues(sensordata &sensor, MS561101BA &baro) {
	// Barometer
	sensor.temperature = baro.getTemperature(MS561101BA_OSR_4096);
	// Delay readings, barometer
	delay(100);
	sensor.pressure = baro.getPressure(MS561101BA_OSR_4096);
	// Init reference for altitude measurement
	initAltitudeMeasurement(sensor, sensor.pressure, sensor.temperature);
}

void initAltitudeMeasurement(sensordata &sensor, float ref_pres, float ref_temp) {
	sensor.alt.c1 = (sensor.alt.dryAirGasConst / sensor.alt.gravAcc) * ref_temp;
	sensor.alt.c2 = log(ref_pres);
}


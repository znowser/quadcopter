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
#include "Regulator/PID.h"
#include "Communication/Communication.h"
#include "Communication/Protocol.h"

#define MOVAVG_SIZE 32
#define SCHEDULE_MINOR_CYCLE 10

float movavg_buff[MOVAVG_SIZE];
int movavg_i = 0;
const float sea_press = 1013.25;
float press, temperature;
float getAltitude(float press, float temp);
void pushAvg(float val);
float getAvg(float *buff, int size);

const bool regulator_activated = TRUE;
void updateSensorValues(sensordata &sensor, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]);
void halt();


/*
 TODO add timeout on the rx serial communication and
 implement crc16 on the channel.

*/

int main(void)
{
	Motor motor[4];
	CellVoltage battery[3];
	MPUAbstraction mpu;
	MS561101BA baro;
	sensordata sensor;
	PID rollRegulator;
	Communication com;
	unsigned engineSpeed = 10;
	
	float ypr[3];
	bool bootStatus = TRUE;
	unsigned long startTime, endTime;
	unsigned long scheuleCounter = 0, scheduleDelay = 0;
	
	//init all pins according to Arduino standard
	init();
	
	//init serial communication and set receive deadline to 4ms.
	com.init(4);
	
	// Join the I2C buss. The only implementation that supports all
	// features that is required by the MPU6050 is the standard Arduino 
	// implementation (wire.h and twi.h).
	// The buss is operating at 400 kHz. 
	Wire.begin();
	delay(100);
	
	com.send("Boot Sequence Launched...");
	
	/*==========Init Sensors ============*/
	//init gyro and magnetic field sensors	
	bootStatus = mpu.init();
	if(bootStatus)
		com.send("MPU6050 connection [OK]");
	else
		com.send("MPU6050 connection [FAILED]");
		
	bootStatus = mpu.enableDMP();
	if(bootStatus)
		com.send("MPU6050 initialized [OK]");
	else
		com.send("MPU6050 initialized [FAILED]");
	
	//barometer and temperature
	bootStatus = baro.init(MS561101BA_ADDR_CSB_LOW);
	if(bootStatus)
		com.send("MS561101BA initialized [OK]");
	else
		com.send("MS561101BA initialized [FAILED]");
	//*==========Init Motors=============*/
	
	/*==========Init Engines ============*/
	//Hardware mapping of motors
	/*
	motor[LF].init(13);
	motor[RF].init(14);
	motor[LB].init(15);
	motor[RB].init(16);
	//minimum wait time for arming
	delay(8000);
	Serial1.println("Engines armed [OK]");*/
	
	/*==================================*/
	
	/*==========Init Battery============*/
	//Hardware mapping of battery cells
	battery[CELL1].init(ADC0);
	battery[CELL2].init(ADC1);
	battery[CELL3].init(ADC2);
	com.send("Battery initialized [OK]");
	/*==================================*/

	rollRegulator.init(5.f, 0.f, 0.f);
	
	
	// populate movavg_buff before starting loop
	for(int i=0; i < MOVAVG_SIZE; i++) {
		movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
		//10ms is the maximum time it take to obtain a new value from the sensor
		delay(10);
	}
	
	// Boot status check, don't continue if something went wrong.
	if(bootStatus)
		com.send("Boot_Sequence_[OK]");
	else{
		com.send("Boot Sequence [FAILED]");	
		com.send("Halting...");
		halt();
	}		
	
	//Clear the FIFO buffer of the MPU6050 so that the data that has been generated 
	//from the initialization until now is removed. Syncs the scheduler with the
	//sensor data.
	mpu.resetFIFO();
	
	int packetLength = 0;
	char *packet = NULL;
	//Main program loop
    while(TRUE){
		//Measure the time of the minor cycle
		startTime = millis();		
		
		//First minor cycle, takes 8 ms to process
		if((++scheuleCounter) % 2 == 0){
			if (mpu.readYawPitchRoll(ypr, sensor.acc)) {
			}
			
		}
		
		// Second minor cycle. The second minor cycle is responsible
		// for the communication with the rasp and the regulators.
		else if(scheuleCounter % 2 == 1){
			// check if there is any new data from the Rasp.
			// The deadline is set to 4ms which means that there
			// is room for less than 6ms further calculations
			// after the receive function has been called!
			// 115200 * 10*10^-3 = 1152 bytes can be received on 10 ms.
			// with the timeout set to 4ms about 500 bytes can be received
			// each cycle. This should be enough for the application.
			// Do NOT try to send/receive more than 500 bytes more frequently 
			// than 20 ms. This equals a bit rate of 25000 bytes/s.
			if(com.receive(packet, packetLength)){
				if(!Protocol::decode(&sensor, packet, packetLength))
					com.send("Bad packet");
					
				if(sensor.ps3.button[JS_BUTTON_CIRCLE])
					com.send("Circle pressed");
				if(sensor.ps3.button[JS_BUTTON_CROSS])
					com.send("Cross pressed");
					
				//else
				//	com.send("Ok ");
			    //com.send(d.c_str(), d.length());
				//com.resetTimeouts();
			}
			 
			// == 6 ms more to perform calculations ==				
			/* =========== Roll regulator =========== */
			float rollReg = rollRegulator.regulate(ypr[2], 0.0f);
			/*Serial1.print(rollReg);
			Serial1.print(" ");
			Serial1.println(ypr[2]);
			*/
			//Serial1.print("Left motors: ");
			//Serial1.println(engineSpeed*(1 + rollReg));
			
			//Left side of the aircraft
			/*motor[LF].setSpeed(engineSpeed*(1 - rollReg));
			motor[LB].setSpeed(engineSpeed*(1 - rollReg));

			//Right side of the aircraft
			motor[RF].setSpeed(engineSpeed*(1 + rollReg));
			motor[RB].setSpeed(engineSpeed*(1 + rollReg));*/
			/* ====================================== */
		}
		
		endTime = millis();
				
		if((scheduleDelay = startTime + SCHEDULE_MINOR_CYCLE) > endTime){
			scheduleDelay -= endTime;
			delay(scheduleDelay);
		}
		else if(scheduleDelay == endTime){
			; // Do nothing
		}
		else 
			com.send("Schedule overflow ");
				
		//Serial1.println(mpu.getAccelerationX());
		//check if there is new sensordata to recieve from the sensor card
		/*if (mpu.readYawPitchRoll(ypr, sensor.acc)) {
			//update sensor struct
			//Serial1.println("Data to REAd");
			Serial1.print("Yaw ");
			Serial1.print(ypr[0]);
			Serial1.print(" Pitch ");
			Serial1.print(ypr[1]);
			Serial1.print(" Roll ");
			Serial1.println(ypr[2]);
			
		}*/
		
		
		//temperature = baro.getTemperature(MS561101BA_OSR_4096);
		//Serial1.print(temperature);
		//Serial1.print(" degC pres: ");
		
		//press = baro.getPressure(MS561101BA_OSR_4096);
		//pushAvg(press);
		//press = getAvg(movavg_buff, MOVAVG_SIZE);
		
		/*
		Serial1.print(temperature);
		Serial1.print(" ");
		Serial1.print(press);
		Serial1.print(" ");
		Serial1.println(getAltitude(press, temperature));*/
		
		//Serial1.print(press);
		//Serial1.print(" mbar altitude: ");
		//if (!(++cnt % 128))
		//	Serial1.println(getAltitude(press, temperature));
		
		//	updateSensorValues(sensor, motor, battery, baro, ypr);
		
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
		*/
		/*	Serial1.print("Yaw ");
			Serial1.print(sensor.gyro[0]);
			Serial1.print(" Pitch ");
			Serial1.print(sensor.gyro[1]);
			Serial1.print(" Roll ");
			Serial1.println(sensor.gyro[2]);*/
		//  if (regulator_activated)
		//	regulator.Regulate();
		//}
    }
}

void updateSensorValues(sensordata &sensor, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]) {
	static byte batInterval = 255;
	// Battery Cell 1
	//if (!++batInterval)
	//	sensor.cellVoltage[CELL1] = battery[CELL1].getVoltage();
	// Gyro
	
	sensor.gyro[2] = ypr[0];
	sensor.gyro[1] = ypr[1];
	sensor.gyro[0] = ypr[2];
	/*
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
		*/
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

void halt(){
	while(TRUE);
}
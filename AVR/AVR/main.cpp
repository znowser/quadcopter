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

int main(void){
	
	//Used by the sensors
	Motor motor[4];
	CellVoltage battery[3];
	MPUAbstraction mpu;
	MS561101BA baro;
	sensordata sensor;
	float ypr[3];
	
	//Used by the regulator
	PID rollRegulator;
	unsigned engineSpeed = 0, engineSpeedLF = 0, engineSpeedLB = 0, engineSpeedRF = 0, engineSpeedRB = 0;
	int calibrationDir = 1;
	
	//Used by the communication
	Communication com;
	int packetLength = 0;
	char *packet = NULL;
	int cnt = 0;
	//Used by boot and scheduler
	bool bootStatus = TRUE;
	unsigned long startTime, endTime;
	unsigned long scheuleCounter = 0, scheduleDelay = 0;
	
	//init all pins according to Arduino standard
	init();
	
	//init serial communication and set receive deadline to 4ms.
	//4 fits the current schedule, do not change unless the
	//entire schedule is reworked.
	com.init(4);
	
	// Join the I2C buss. The only implementation that supports all
	// features that is required by the MPU6050 is the standard Arduino 
	// implementation (wire.h and twi.h).
	// The buss is operating at 400 kHz. 
	Wire.begin();
	delay(100);
	
	com.println("Boot Sequence Launched...");
	
	/*==========Init Sensors ============*/
	//init gyro and magnetic field sensors	
	bootStatus = mpu.init();
	mpu.setHardwareOffset(0.2198, 0.039, 0.005);
	if(bootStatus)
		com.println("MPU6050 connection [OK]");
	else
		com.println("MPU6050 connection [FAILED]");
		
	bootStatus = mpu.enableDMP();
	if(bootStatus)
		com.println("MPU6050 initialized [OK]");
	else
		com.println("MPU6050 initialized [FAILED]");
	
	//barometer and temperature
	bootStatus = baro.init(MS561101BA_ADDR_CSB_LOW);
	if(bootStatus)
		com.println("MS561101BA initialized [OK]");
	else
		com.println("MS561101BA initialized [FAILED]");
	//*==========Init Motors=============*/
	
	/*==========Init Engines ============*/
	//Hardware mapping of motors
	
	motor[LF].init(16);
	motor[RF].init(15);
	motor[LB].init(14);
	motor[RB].init(13);
	//minimum wait time for arming
	delay(8000);
	com.println("Engines armed [OK]");
	
	/*==================================*/
	
	/*==========Init Battery============*/
	//Hardware mapping of battery cells
	battery[CELL1].init(ADC0);
	battery[CELL2].init(ADC1);
	battery[CELL3].init(ADC2);
	com.println("Battery initialized [OK]");
	/*==================================*/
	
	// For safety reasons, turn off the engines until
	// the connection to the rasp is established.
	sensor.stop = true;

	rollRegulator.init(0.01f, 0.f, 0.f);
	
	
	// populate movavg_buff before starting loop
	for(int i=0; i < MOVAVG_SIZE; i++) {
		movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
		//10ms is the maximum time it take to obtain a new value from the sensor
		delay(10);
	}
	
	// Boot status check, don't continue if something went wrong.
	if(bootStatus)
		com.println("Boot Sequence [OK]");
	else{
		com.println("Boot Sequence [FAILED]");	
		com.println("Halting...");
		halt();
	}		
	
	//Clear the FIFO buffer of the MPU6050 so that the data that has been generated 
	//from the initialization until now is removed. Syncs the scheduler with the
	//sensor data.
	mpu.resetFIFO();
	
	//Main program loop
    while(TRUE){
		//Measure the time of the minor cycle
		startTime = millis();		
		
		//First minor cycle, takes 8 ms to process
		if((++scheuleCounter) % 2 == 0){
			if (mpu.readYawPitchRoll(ypr, sensor.acc)) {
				;
			}
		}
		
		// Second minor cycle. The second minor cycle is responsible
		// for the communication with the rasp and the regulators.
		else if(scheuleCounter % 2 == 1){
			/*
			com.print("Yaw: ");
			com.print(ypr[0]);
			com.print("   ");
			com.print("pitch: ");
			com.print(ypr[1]);
			com.print("   ");
			com.print("roll: ");
			com.println(ypr[2]);
			*/
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
				if(!Protocol::decode(&sensor, packet, packetLength)){
					memset(&sensor, 0x00, sizeof(sensordata));
					com.println("Bad packet");
				}
				else{
					if(sensor.ps3.button[JS_BUTTON_START]){
						//com.send("Turning engines on");
						sensor.stop = false;
					}
					else if(sensor.ps3.button[JS_BUTTON_SELECT]){
						//com.send("Turning engines off");
						sensor.stop = true;
					}
					
					if(sensor.ps3.button[JS_BUTTON_TRIANGLE]){
						rollRegulator.P += 0.001;
						com.print("P: ");
						com.println(rollRegulator.P);
					}
					if(sensor.ps3.button[JS_BUTTON_CROSS]){
						rollRegulator.P -= 0.001;
						com.print("P: ");
						com.println(rollRegulator.P);
					}
					
					if(sensor.ps3.button[JS_BUTTON_L1]){
						engineSpeed -= 1;
						if(engineSpeed < 0)
						 engineSpeed = 0;
						//com.send("Decreasing speed");
					}
				
					if(sensor.ps3.button[JS_BUTTON_R1]){
						engineSpeed += 1;
						//com.send("Increasing speed");
					}
					
					if(sensor.ps3.button[JS_BUTTON_PS3])
						calibrationDir = -1;
					else
						calibrationDir = 1;
										
					if(sensor.ps3.button[JS_BUTTON_UP])
						engineSpeedLF += engineSpeedLF + calibrationDir >= 0 ? calibrationDir:0;
						
					if(sensor.ps3.button[JS_BUTTON_RIGHT])
						engineSpeedRF += engineSpeedRF + calibrationDir >= 0 ? calibrationDir:0;
					
					if(sensor.ps3.button[JS_BUTTON_LEFT])
						engineSpeedLB += engineSpeedLB + calibrationDir >= 0 ? calibrationDir:0;
					
					if(sensor.ps3.button[JS_BUTTON_DOWN])
						engineSpeedRB += engineSpeedRB + calibrationDir >= 0 ? calibrationDir:0;
				}				
			}
			
			// == 6 ms more to perform calculations ==
			if(sensor.stop){	
				motor[LF].setSpeed(0);
				motor[LB].setSpeed(0);
				motor[RF].setSpeed(0);
				motor[RB].setSpeed(0);
			}
			else{
				/* =========== Roll regulator =========== */
				float rollReg = rollRegulator.regulate(ypr[2], 0.0f);
				#define MAX_REGULATOR_SPEED 100
				com.print("PID signal: ");
				com.print(rollReg);
				com.print("   LF: ");
				com.print(engineSpeed + engineSpeedLF - rollReg*MAX_REGULATOR_SPEED);
				com.print("    LB: ");
				com.print(engineSpeed + engineSpeedLB - rollReg*MAX_REGULATOR_SPEED);
				com.print("    RF: ");
				com.print(engineSpeed + engineSpeedRF + rollReg*MAX_REGULATOR_SPEED);
				com.print("    RB: ");
				com.println(engineSpeed + engineSpeedRB + rollReg*MAX_REGULATOR_SPEED);
				motor[LF].setSpeed(engineSpeed + engineSpeedLF + rollReg*MAX_REGULATOR_SPEED);
				motor[LB].setSpeed(engineSpeed + engineSpeedLB + rollReg*MAX_REGULATOR_SPEED);
				motor[RF].setSpeed(engineSpeed + engineSpeedRF - rollReg*MAX_REGULATOR_SPEED);
				motor[RB].setSpeed(engineSpeed + engineSpeedRB - rollReg*MAX_REGULATOR_SPEED);
				
				/*Serial1.print(rollReg);
				Serial1.print(" ");
				Serial1.println(ypr[2]);
				*/
				/* ====================================== */
			}
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
			com.println("Schedule overflow ");
				
	
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
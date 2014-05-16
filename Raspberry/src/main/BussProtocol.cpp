#include "Motor.h"
#include "BussProtocol.h"


//Function that send away the whole sensorstruct to the rasp
//Format the package by martins format
char* buildSensorPackage(const sensordata &data, char* res, int &len) {
	char tmp[10];

	len = 0;
	len += sprintf(&res[len], "c1:%d!", (int)data.cellVoltage[cell1]);
	len += sprintf(&res[len], "c2:%d!", (int)data.cellVoltage[cell2]);
	len += sprintf(&res[len], "c3:%d!", (int)data.cellVoltage[cell3]);

	len += sprintf(&res[len], "lf:%d!", data.motorSpeed[leftfront]);
	len += sprintf(&res[len], "rf:%d!", data.motorSpeed[rightfront]);
	len += sprintf(&res[len], "lb:%d!", data.motorSpeed[leftback]);
	len += sprintf(&res[len], "rb:%d!", data.motorSpeed[rightback]);

	//workaround because AVR IDE doesn't support sprintf with %f....
	fmtDouble(data.temperature, 2, tmp, 10);
	len += sprintf(&res[len], "temp:%s!", tmp);
	fmtDouble(data.height, 2, tmp, 10);
	len += sprintf(&res[len], "height:%s!", tmp);

	fmtDouble(data.angleYaw, 2, tmp, 10);
	len += sprintf(&res[len], "yaw:%s!", tmp);
	fmtDouble(data.anglePitch, 2, tmp, 10);
	len += sprintf(&res[len], "ptich:%s!", tmp);
	fmtDouble(data.angleRoll, 2, tmp, 10);
	len += sprintf(&res[len], "roll:%s!", tmp);
	return res;
}

void emergencyStopCallback(char *data, int len, void *additional_info) {
	Motor *motor = (Motor*)additional_info;
	//Turn off all motors
	motor[leftfront].setSpeed(0);
	motor[rightfront].setSpeed(0);
	motor[leftback].setSpeed(0);
	motor[rightback].setSpeed(0);
}

void ps3DataCallback(char *data, int len, void *additional_info) {
	sensordata *sensorData = (sensordata*)additional_info;
}

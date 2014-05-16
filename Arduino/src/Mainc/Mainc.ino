#include <Servo.h>
#include "Motor.h"
#include "CellVoltage.h"
#include "Wire.h"
#include "MPU6050Abstraction.h"
#include "MS561101BA.h"
#include "SensorDataStruct.h"
#include "Hover.h"
#include "SerialBuss.h"
#include "ExtendedFloatSupport.h"


//change this variable to true if you want to turn on the regulator, Torbjörn.
const bool regulator_activated = false;
const float sea_press = 1013.25;

/*========= Do NOT change this ===========*/
//Workaround to get normal program-flow of main function.
bool runOnce = true;
/*Do not use or touch these functions!*/
void setup() {}
void loop() {
  runOnce ? runOnce = mainf() : 0;
}
/*========================================*/

float getAltitude(float press, float temp);
char* buildSensorPackage(const sensordata &data, char* res, int &len);
void updateSensorValues(sensordata &sensorData, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]);
void emergencyStopCallback(char *data, int len, void *additional_info);
void ps3DataCallback(char *data, int len, void *additional_info);
//cannot be called with the name: main(), strange Arduino syndrome!
int mainf() {
  SerialBuss serial;
  Motor motor[4];
  CellVoltage battery[3];
  sensordata sensorData;
  float ypr[3];

  /*==========Init Sensors============*/
  //init gyro and magnetic field
  MPUAbstraction mpu = MPUAbstraction();
  //barometer and temperature
  MS561101BA baro = MS561101BA(MS561101BA_ADDR_CSB_LOW);

  /*===================================*/
  //*==========Init Motors=============*/
  //Hardware mapping of motors
  motor[leftfront].init(10);
  motor[rightfront].init(11);
  motor[leftback].init(12);
  motor[rightback].init(13);
  /*==================================*/
  /*=====Init battery cells ==========*/
  //Hardware mapping of battery cells
  battery[cell1].init(A0);
  battery[cell2].init(A1);
  battery[cell3].init(A2);
  /*==================================*/

  serial.registerCallback(emergencyStopCallback, motor, 0x01);
  serial.registerCallback(ps3DataCallback, &sensorData, 0x03);
  /*==========Hover regulator=============*/
  Hover regulator(motor, &sensorData, 0.5);
  //Main regulator/sensor loop
  int len = 0;
  char tmp[200];
  while (true) {
    //TODO continue to implement the new buss protocol
    //serial.recvRasp();

    //check if there is new sensordata to recieve from the sensor card
    if (mpu.readYawPitchRoll(ypr, sensorData.acc)) {
      //update sensor struct
      updateSensorValues(sensorData, motor, battery, baro, ypr);
      //send the sensorstruct to the raspberry or regulate
      if (regulator_activated)
        regulator.Regulate();
      else {
        serial.sendRasp(0x01, buildSensorPackage(sensorData, tmp, len), len);
        memset(tmp, 0, sizeof(tmp));
      }
    }
  }

  //return 0 if nothing more shall be executed, otherwise the main-function will
  //be called again.
  return 0;
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
  sensorData.angleYaw = ypr[0]  * 180 / M_PI;
  sensorData.anglePitch = ypr[1]  * 180 / M_PI;
  sensorData.angleRoll = ypr[2]  * 180 / M_PI;
  sensorData.cellVoltage[cell1] = battery[cell1].getVoltage();
  sensorData.cellVoltage[cell2] = battery[cell2].getVoltage();
  sensorData.cellVoltage[cell3] = battery[cell3].getVoltage();
  //update motor speed
  sensorData.motorSpeed[leftfront] = motor[leftfront].getSpeed();
  sensorData.motorSpeed[rightfront] = motor[rightfront].getSpeed();
  sensorData.motorSpeed[leftback] = motor[leftback].getSpeed();
  sensorData.motorSpeed[rightback] = motor[rightback].getSpeed();
  //get pressure, cannot be done directly after getTemperature, The sensorcard need
  //a small delay between the function calls.
  sensorData.pressure = baro.getPressure(MS561101BA_OSR_4096);
  sensorData.height = getAltitude(sensorData.pressure, sensorData.temperature);
}

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

}

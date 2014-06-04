#include <Servo.h>
#include "Motor.h"
#include "CellVoltage.h"
#include "Wire.h"
#include "MPU6050Abstraction.h"
#include "MS561101BA.h"
#include "SensorDataStruct.h"
#include "Hover.h"
//#include "SerialBuss.h"
#include "BussProtocol.h"

//change this variable to true if you want to turn on the regulator, Torbjörn.
const bool regulator_activated = true;
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
void updateSensorValues(sensordata &sensorData, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]);
//cannot be called with the name: main(), strange Arduino syndrome!
int mainf() {
  Serial.begin(115200);
  //SerialBuss serial;
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
  motor[LF].init(10);
  motor[RF].init(11);
  motor[LB].init(12);
  motor[RB].init(13);
  /*==================================*/
  /*=====Init battery cells ==========*/
  //Hardware mapping of battery cells
  battery[CELL1].init(A0);
  battery[CELL2].init(A1);
  battery[CELL3].init(A2);
  /*==================================*/

  //serial.registerCallback(ps3DataCallback, &sensorData, PS3_CONTROLLER_PACKAGE);
  /*==========Hover regulator=============*/
  float refVal[6] = { 0, 0, 1.0, 0, 0, 0 };
  Hover regulator(motor, &sensorData, refVal);
  //Main regulator/sensor loop
  int len = 0;
  char tmp[150];
  int sendCnt = 0;
  while (true) {
    //TODO continue to implement the new buss protocol
  //  serial.recvRasp();

    //check if there is new sensordata to recieve from the sensor card
    if (mpu.readYawPitchRoll(ypr, sensorData.acc)) {
      //update sensor struct
      updateSensorValues(sensorData, motor, battery, baro, ypr);
      /*
      if (++sendCnt % 10 == 0) {
        Serial.write(buildSensorPackage(sensorData, tmp, len), len);
        Serial.println();
      }
      */
      //send the sensorstruct to the raspberry or regulate
      if (regulator_activated)
        regulator.Regulate();
        
   //   else
   //     serial.sendRasp(SENSORDATA_PACKAGE, buildSensorPackage(sensorData, tmp, len), len);
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

#include <Servo.h>
#include "Motor.h"
#include "CellVoltage.h"
//wire.h must be included here.
#include "Wire.h"
#include "MPU6050Abstraction.h"
#include "MS561101BA.h"
#include "SensorDataStruct.h"
#include "Hover.h"

const float sea_press = 1013.25;

/*========= Do not change this ===========*/
//Workaround to get normal program-flow of main function.
bool runOnce = true;
/*Do not use these functions*/
void setup() {}
void loop() {
  runOnce ? runOnce = mainf() : 0;
}
/*========================================*/

float getAltitude(float press, float temp);
template<class T> void dataToRasp(const char *variableName, T data);
//cannot be called with the name: main(), strange Arduino syndrome!
int mainf() {
  Serial.begin(115200);
  Motor motor[4];
  CellVoltage battery[3];
  float temperature = 0, pressure = 0;
  sensordata sensorData;
  float ypr[3];
  float height;
  bool regulatorIsInitialized = false;

  /*==========Init Sensors============*/
  //init gyro and magnetic field
  MPUAbstraction mpu = MPUAbstraction();
  mpu.init();
  //register sensorcallback
  attachInterrupt(0, MPUAbstraction::MPUInt, RISING);
  //barometer and temperature
  MS561101BA baro = MS561101BA();
  baro.init(MS561101BA_ADDR_CSB_LOW);

  /*===================================*/
  //*==========Init Motors=============*/
  //Hardware mapping of motors
  motor[leftfront].init(10);
  motor[rightfront].init(11);
  motor[leftback].init(12);
  motor[rightback].init(13);
  /*==================================*/
  /*=====Init battery cells ==========*/
  battery[cell1].init(A0);
  battery[cell2].init(A1);
  battery[cell3].init(A2);
  /*==================================*/

  /*==========Hover regulator=============*/
  Hover regulator;

  //Main regulator/sensor loop
  while (true) {
    if (mpu.readYawPitchRoll(ypr)) {
      //temperature and pressure cannot be read directly after each other, it
      //must be a small delay between the two functioncalls.
      temperature = baro.getTemperature(MS561101BA_OSR_4096);

      //========== The following data is sent to the PC on martins format ==========
      /*
      //send battery info
      dataToRasp<int>("c1", battery[cell1].getVoltage());
      dataToRasp<int>("c2", battery[cell2].getVoltage());
      dataToRasp<int>("c3", battery[cell3].getVoltage());

      //send motor data
      dataToRasp<int>("lf", motor[leftfront].getSpeed());
      dataToRasp<int>("rf", motor[rightfront].getSpeed());
      dataToRasp<int>("lb", motor[leftback].getSpeed());
      dataToRasp<int>("rb", motor[rightback].getSpeed());
      */
      //get pressure, cannot be done directly after getTemperature, The sensorcard need
      //a small delay between the function calls.
      pressure = baro.getPressure(MS561101BA_OSR_4096);
      height = getAltitude(pressure, temperature);

      /*
      //send temperature/pressure/height
      dataToRasp<float>("temp", temperature);
      dataToRasp<float>("height", height);

      //send yaw/pitch/yaw
      dataToRasp<float>("yaw", 180 + ypr[0] * 180/M_PI);
      dataToRasp<float>("pitch", 180 + ypr[1] * 180/M_PI);
      dataToRasp<float>("roll", 180 + ypr[2] * 180/M_PI);

      //quick fix, Each package that is sent to the raspberry must end
      //with a newline
       Serial.println();
      */
      //==============================================

      sensorData.temperature = temperature;
      sensorData.pressure = pressure;
      sensorData.height = height;
      sensorData.angleYaw = ypr[0];
      sensorData.anglePitch = ypr[1];
      sensorData.angleRoll = ypr[2];
      
      //init the regulator if it haven't not been done yet, must be initialized
      //with a valide height
      if (!regulatorIsInitialized) {
        regulator.init(motor, &sensorData, (float)(sensorData.height + 0.4));
        regulatorIsInitialized = true;
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

//Martins format
//each variable is sent by the format " variable:data! "
template<class T>
void dataToRasp(const char *variableName, T data) {
  Serial.print(variableName);
  Serial.print(":");
  Serial.print(data);
  Serial.print("!");
}


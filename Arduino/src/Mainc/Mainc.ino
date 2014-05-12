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
/*Do not use or touch these functions!*/
void setup() {}
void loop() {
  runOnce ? runOnce = mainf() : 0;
}
/*========================================*/

float getAltitude(float press, float temp);
void SensorDataToRasp(const sensordata &data);
void updateSensorValues(sensordata &sensorData, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]);
template<class T> void variableToRasp(const char *variableName, T data);
//cannot be called with the name: main(), strange Arduino syndrome!
int mainf() {
  Serial.begin(115200);
  Motor motor[4];
  CellVoltage battery[3];
  sensordata sensorData;
  float ypr[3];
  bool regulatorIsInitialized = false;

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
  battery[cell1].init(A0);
  battery[cell2].init(A1);
  battery[cell3].init(A2);
  /*==================================*/

  /*==========Hover regulator=============*/
  //Hover regulator;
  //Main regulator/sensor loop
  while (true) {
    if (mpu.readYawPitchRoll(ypr)) {
      //update sensor struct
      updateSensorValues(sensorData, motor, battery, baro, ypr);
      //send the sensorstruct to the raspberry
      SensorDataToRasp(sensorData);

      //init the regulator if it haven't not been done yet, must be initialized
      //with a valide height
      /*if (!regulatorIsInitialized && temperature != 0 && pressure != 0) {
        Serial.print("initializing regulator with height: ");
        Serial.println(height);
        Serial.print("Temp and pressure ");
        Serial.println(temperature);
        Serial.println(pressure);
        regulator.init(motor, &sensorData, 0.4);
        regulatorIsInitialized = true;
      }*/
    }

    //if(regulatorIsInitialized)
    //  regulator.Regulate();
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
void variableToRasp(const char *variableName, T data) {
  Serial.print(variableName);
  Serial.print(":");
  Serial.print(data);
  Serial.print("!");
}

void updateSensorValues(sensordata &sensorData, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]){
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
void SensorDataToRasp(const sensordata &data) {
  //========== The following data is sent to the PC on martins format ==========
  //send battery info
  variableToRasp<int>("c1", data.cellVoltage[cell1]);
  variableToRasp<int>("c2", data.cellVoltage[cell2]);
  variableToRasp<int>("c3", data.cellVoltage[cell3]);
  //send motor data
  variableToRasp<int>("lf", data.motorSpeed[leftfront]);
  variableToRasp<int>("rf", data.motorSpeed[rightfront]);
  variableToRasp<int>("lb", data.motorSpeed[leftback]);
  variableToRasp<int>("rb", data.motorSpeed[rightback]);
  //send temperature/pressure/height
  variableToRasp<float>("temp", data.temperature);
  variableToRasp<float>("height", data.height);
  //send yaw/pitch/yaw
  variableToRasp<float>("yaw", data.angleYaw);
  variableToRasp<float>("pitch", data.anglePitch);
  variableToRasp<float>("roll", data.angleRoll);
  //quick fix, Each package that is sent to the raspberry must end
  //with a newline
  Serial.println();
}

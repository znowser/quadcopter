#include <Servo.h>
#include "Motor.h"
#include "CellVoltage.h"
//wire.h must be included here.
#include "Wire.h"
#include "MPU6050Abstraction.h"
#include "MS561101BA.h"
//#include "Hover.h"


#define NUM_OF_MOTORS 4
#define NUM_OF_CELLS 3

/*======== Mapping of hardwarePins ========*/
enum MotorPins { 
  leftfront = 0, rightfront = 1, leftback = 2, rightback = 3}; 
enum CellPins{
  cell1 = 0, cell2 = 1, cell3 = 2};
/*=========================================*/
const float sea_press = 1013.25;

/*========= Do not change this ===========*/
//Workaround to get normal program-flow of main function.
bool runOnce = true;
/*Do not use these functions*/
void setup(){}
void loop(){runOnce ? runOnce = mainf() : 0;}
/*========================================*/

float getAltitude(float press, float temp);
//cannot be called with the name: main(), strange Arduino syndrome!
int mainf(){
  Serial.begin(115200);
  Motor motor[4];
  CellVoltage battery[3];
  float temperature = 0, pressure = 0;
  float ypr[3];
  
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
  
  //Main regulator/sensor loop
  while(true){
    if(mpu.readYawPitchRoll(ypr)){
      //temperature and pressure cannot be read directly after each other, it
      //must be a small delay between the two functioncalls.
      temperature = baro.getTemperature(MS561101BA_OSR_4096);
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[2] * 180/M_PI);
      Serial.print("\t");  
      Serial.print("Altitude: ");
      Serial.print(getAltitude(pressure, temperature));   
      Serial.println(" m");
      pressure = baro.getPressure(MS561101BA_OSR_4096);
    }
    
    delay(10);
  }  
  return 0;
}

float getAltitude(float press, float temp) {
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}


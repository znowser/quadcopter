#include <Servo.h>
//wire.h must be included here.
#include "Wire.h"
#include "MPU6050Abstraction.h"
#include "MS561101BA.h"
#include "Motor.h"
#include "CellVoltage.h"
#include "Hover.h"

/*======== Mapping of hardwarePins ========*/
enum MotorPins { 
  leftfront = 10, rightfront = 11, leftback = 12, rightback = 13}; 
enum CellPins{
  cell1 = A0, cell2 = A1, cell3 = A2};
/*=========================================*/
const float sea_press = 1013.25;

/*========= Do not change this ===========*/
//Workaround to get normal program-flow of main function.
bool runOnce = true;
/*Do not use these functions*/
void setup(){}
void loop(){runOnce ? runOnce = main() : 0;}
/*========================================*/

float getAltitude(float press, float temp);
int main(){
  Motor motor[4];
  CellVoltage battery[3];
  float temperature = 0, pressure = 0;

  //gyro and magnetic field
  MPUAbstraction mpu = MPUAbstraction();
  mpu.init();
  //register sensorcallback
  attachInterrupt(0, MPUAbstraction::MPUInt, RISING);
  //barometer and temperature
  MS561101BA baro = MS561101BA();
  baro.init(MS561101BA_ADDR_CSB_LOW);

  //maybe not needed (?)
  delay(100);

  /*==============Init ==============*/
  //init motors
  for(int i = 0; i < sizeof(motor); i++)
    motor[i].init((MotorPins)i);

  //init batterycells
  for(int i = 0; i < sizeof(battery); i++)
    battery[i].init((CellPins)i);
  /*================================*/

  //set speed
  //motor[leftfront].setSpeed(10);
  //get current speed
  //s = motor[leftfront].getSpeed();  

  float ypr[3];
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
  }

  int s = motor[leftfront].getSpeed();  
  
  /*==============Hover=============*/
  Hover regulator(motor, sensor);
  regulator.Start();
  
  //return 0 if nothing more shall be executed, otherwise the main-function will
  //be called again.
  return 0;
}

float getAltitude(float press, float temp) {
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}




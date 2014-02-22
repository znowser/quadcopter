#include <Servo.h>

#include "Motor.h"
#include "CellVoltage.h"

/*======== Mapping of hardwarePins ========*/
enum MotorPins { 
  leftfront = 10, rightfront = 11, leftback = 12, rightback = 13}; 
enum CellPins{
  cell1 = A0, cell2 = A1, cell3 = A2};
/*=========================================*/

bool runOnce = false;
/*Do not use this function*/
void setup(){}
/*Do not change this */
//workaround to get normal program-flow of main function.
void loop(){
  if(!runOnce)
    main();
  runOnce = true;  
}

int main(){
  Motor motor[4];
  CellVoltage battery[3];

  /*==============Init ==============*/
  //init motors
  for(int i = 0; i < sizeof(motor); i++)
    motor[i].init((MotorPins)i);

  //init batterycells
  for(int i = 0; i < sizeof(battery); i++)
    battery[i].init((CellPins)i);
  /*================================*/

  motor[leftfront] = 40;
  motor[leftfront].setSpeed(40);

  int s = motor[leftfront];
  s = motor[leftfront].getSpeed();  
  
  return 0;
}






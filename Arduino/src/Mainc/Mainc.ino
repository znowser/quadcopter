#include <Servo.h>

#include "Motor.h"
#include "CellVoltage.h"
#include "Hover.h"

/*======== Mapping of hardwarePins ========*/
enum MotorPins { 
  leftfront = 10, rightfront = 11, leftback = 12, rightback = 13}; 
enum CellPins{
  cell1 = A0, cell2 = A1, cell3 = A2};
/*=========================================*/

/*========= Do not change this ===========*/
//Workaround to get normal program-flow of main function.
bool runOnce = true;
/*Do not use these functions*/
void setup(){}
void loop(){runOnce ? runOnce = main() : 0;}
/*========================================*/

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
  
  /*==============Hover=============*/
  Hover regulator(motor, sensor);
  regulator.Start();
  
  //return 0 if nothing more shall be executed, otherwise the main-function will
  //be called again.
  return 0;
}

void hover(void) {
  
}




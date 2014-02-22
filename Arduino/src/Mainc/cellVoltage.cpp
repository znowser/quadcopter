#include <arduino.h>
#include "CellVoltage.h"

//static constant-table
const float CellVoltage::callibration[3] = { 
  1.04, 1, 1 };
 
void CellVoltage::init(int pin){
  this->pin = pin;
  //map callibration to the corresponding pin.
  switch (pin){
  case A0:
    c = callibration[0];
    break;
  case A1:
    c = callibration[1];
    break;
  case A2:
    c = callibration[2];
    break;
  default:
    c = 1;
  }
  //Formula to convert value from AD to accual voltage.
  c = c * (5.0 / 1023.0);  
}

CellVoltage::CellVoltage(int pin): pin(pin){
  init(pin);
}

float CellVoltage::getVoltage(){
  return (analogRead(pin) * c);
}


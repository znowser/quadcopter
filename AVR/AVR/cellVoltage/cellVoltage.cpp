#include "../lib/Arduino/Arduino.h"
#include "CellVoltage.h"

void CellVoltage::init(int pin){
  this->pin = pin;
 }

CellVoltage::CellVoltage(int pin): pin(pin){
  init(pin);
}

//return voltage in percentage for the cell
byte CellVoltage::getVoltage(){
  return analogRead(pin);//(analogRead(pin)/255.f)*VREF;
}


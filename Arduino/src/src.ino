#include "CellVoltage.h"

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  CellVoltage cell1(A0);
  CellVoltage cell2(A1);
  CellVoltage cell3(A2);
  
  while(Serial.available() > 0){
    Serial.read();
    
    Serial.print("Cell1: ");
    Serial.print(cell1.getVoltage());
    Serial.print(" , Cell2: ");
    Serial.print(cell2.getVoltage());
    Serial.print(" , Cell3: ");
    Serial.println(cell3.getVoltage());
    
    Serial.print("Total voltage: ");
    Serial.println(cell1.getVoltage() + cell2.getVoltage() + cell3.getVoltage());
  }
  
}





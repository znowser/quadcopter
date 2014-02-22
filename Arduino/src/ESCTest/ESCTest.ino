#include <Servo.h> 
#include "Motor.h"

Motor leftFrontMotor;
Motor rightFrontMotor;
Motor leftbackMotor;
Motor rightbackMotor;
void setup() 
{ 
  Serial.begin(9600);
  leftFrontMotor.init(10);
  rightFrontMotor.init(11);
  leftbackMotor.init(12);
  rightbackMotor.init(13);
}

void loop() 
{ 
  int res = 0;
  int received_byte = 0;

  //quit if no data is available on serial
  if(Serial.available()  <= 0){
    return;
  }

  while(Serial.available() > 0){
    //convert from ascii to int
    received_byte = Serial.read() - 0x30;

    //check if the recieved data is a number, else quit
    if(received_byte < 0 || received_byte > 9){
      Serial.println("Bad format");
      return; 
    }    
    res = res*10 + received_byte;
    //make sure the next character is loaded before next iteration.
    delay(10);
  }

  Serial.print("Value recieved: ");
  Serial.println(res, DEC);

  //write recieved data to the motor.
  leftFrontMotor.setSpeed(res);
  rightFrontMotor.setSpeed(res);
  leftbackMotor.setSpeed(res);
  rightbackMotor.setSpeed(res);
} 


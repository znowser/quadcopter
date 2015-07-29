#include "../lib/Arduino/Arduino.h"
#include "Motor.h"


Motor::Motor(int pin){
  initialized = FALSE;
  init(pin);
}

void Motor::init(int pin){
  //Prevent multiply initializing of same instance
  if(!initialized){
    //init members
    this->pin = pin;
    speed = MIN;
    initialized = TRUE;
    //Serial.println("Initialize motor");
    esc.attach(pin);
    //=== arm ESC ===
    esc.write(MIN);
    //To arm the ESC a PPM with value MIN is applied for 1.5s.
   // delay(1500);
    //==============
  }
}

void Motor::callibrateESC(){
  esc.write(MAX);
  delay(2000);
  esc.write(MIN);
  delay(2000);
}

//allow different syntax for setter
unsigned Motor::operator=(unsigned speed){
  return setSpeed(speed);
}

//Set the motor-speed. If the ESC is not initialized or an invalid value is passed
//the function will return 0, speed otherwise.
unsigned Motor::setSpeed(unsigned _speed){
  _speed += MIN;
  if(initialized && _speed <= MAX){
    this->speed = _speed;
    esc.write(this->speed);
    return _speed;
  }
  return 0;
}

unsigned Motor::getSpeed(){
  return this->speed - MIN;
}

//allow different syntax for getter
Motor::operator unsigned(){
  return speed - MIN;
}

#include "Motor.h"
#include <arduino.h>

Motor::Motor(int pin){
  initialized = false;
  init(pin);
}

void Motor::init(int pin){
  //Prevent multiply initializing of same instance
  if(!initialized){
    //init members
    this->pin = pin;
    speed = MIN;
    initialized = true;
    Serial.println("initte");
    esc.attach(pin);
    //=== arm ESC ===
    esc.write(MIN);
    //To arm the ESC a PPM with value MIN is applied for 1.5s.
    delay(1500);
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
int Motor::operator=(int speed){
  return setSpeed(speed);
}

//Set the motor-speed. If the ESC is not initialized or an invalid value is passed
//the function will return 0, speed otherwise.
int Motor::setSpeed(int speed){
  if(initialized && speed >= MIN && speed <= MAX){
    this->speed = speed;
    esc.write(speed);
    return speed;
  }
  return 0;
}

int Motor::getSpeed(){
  return speed;
}

//allow different syntax for getter
Motor::operator int(){
  return speed;
}

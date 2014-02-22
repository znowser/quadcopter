#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>

/*Representation of motor with ESC.
 The Motorclass can be initialized either by the constructor Motor(int pin)
 or later in the program by void init(int pin). The instance MUST be 
 initialized before the speed can be changed.
 
   //there are two different syntax to get and set the speed:
    motor[leftfront] = 40;
    motor[leftfront].setSpeed(40);
  
    int s = motor[leftfront];
    s = motor[leftfront].getSpeed(); 
 */
class Motor{
private:
  Servo esc;
  int pin;
  int speed;
  bool initialized;
public:
  Motor(int pin);
  Motor():initialized(false){};
  void init(int pin);
  void callibrateESC();
  //void programESC(); //TODO
  //setter functions
  int setSpeed(int percent);
  int operator=(int);
  
  //getter functions
  int getSpeed();
  operator int();  
  
  enum SPEED { 
    MAX = 180, MIN = 75, OFF = 75     };
};

#endif



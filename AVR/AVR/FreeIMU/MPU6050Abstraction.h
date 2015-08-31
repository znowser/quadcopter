#ifndef MPU6050ABSTRACTION_H
#define MPU6050ABSTRACTION_H

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "../lib/Arduino/Arduino.h"

// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
//make sure the correct definition of the include is used.
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include "MPU6050.h"

//Hardware wiring
#define MPU_INTERRUPT_PIN 7 // External_Interrupt_7

class MPUAbstraction{
private:
  // class default I2C address is 0x68
  // specific I2C addresses may be passed as a parameter here
  // AD0 low = 0x68 (default for InvenSense evaluation board)
  // AD0 high = 0x69
  MPU6050 mpu;
  uint8_t devStatus;
  static volatile bool mpuDataReady;
  //MPU6050 accelgyro(0x69); // <-- use for AD0 high

  //MPU control variables
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  uint8_t mpuIntStatus;
  uint16_t fifoCount;     // count of all bytes currently in FIFO

  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  
  float offsetYaw, offsetPitch, offsetRoll;
  
public:
  bool init();
  bool enableDMP();
  void resetFIFO();
  bool readYawPitchRoll(float ypr[3], int16_t acc[3]);
  static void MPUInt();
  bool deviceStatus();
  void setHardwareOffset(float yaw, float pitch, float roll);
};

#endif


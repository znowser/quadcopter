#include "MPU6050Abstraction.h"
//contains definitions, can only be included once!
#include "MPU6050_6Axis_MotionApps20.h"

volatile bool MPUAbstraction::mpuDataReady = false;

MPUAbstraction::MPUAbstraction(){
}

void MPUAbstraction::init(){ 
  Wire.begin();
  // initialize device
  //Serial.println("Initializing I2C devices...");
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0){
    //enable the Digital Motion Processor (DMP) on the 6050
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    //attach FreeIMUs INTA to interrupt port (Digital in port 2)
    attachInterrupt(0, MPUInt, RISING);
  }
}

void MPUAbstraction::MPUInt(){
  mpuDataReady = true;
}

//return true if yaw, pitch, roll was reveiced from the sensor MPU6050, false otherwise
bool MPUAbstraction::readYawPitchRoll(float ypr[3]){
  // if programming failed or there is no data available, don't try to do anything
  if (mpuDataReady == false || (devStatus != 0)) return false;

  //switch to master-mode on the i2c
  //Wire.begin();
  
  //reset interrupt-flag
  mpuDataReady = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return true;
  }
  return false;
}
bool MPUAbstraction::deviceStatus(){
  return devStatus != 0 ? false:mpu.testConnection();
}


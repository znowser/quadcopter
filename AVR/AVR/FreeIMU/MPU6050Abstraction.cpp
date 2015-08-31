#include "MPU6050Abstraction.h"
//contains definitions, can only be included once!
#include "MPU6050_6Axis_MotionApps20.h"

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

volatile bool MPUAbstraction::mpuDataReady = FALSE;

bool MPUAbstraction::init(){
	mpu.initialize();
	// verify connection
	return mpu.testConnection();
}

bool MPUAbstraction::enableDMP(){  
  // initialize the DMP module
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0){
    //enable the Digital Motion Processor (DMP) on the 6050
    mpu.setDMPEnabled(TRUE);
    packetSize = mpu.dmpGetFIFOPacketSize();
    //attach FreeIMUs INTA to interrupt port (Digital in port 2)
    //register sensorcallback
    attachInterrupt(MPU_INTERRUPT_PIN, MPUInt, RISING);
  }
  return devStatus == 0;
}

void MPUAbstraction::MPUInt(){
  mpuDataReady = TRUE;  
}
void MPUAbstraction::resetFIFO(){
	mpu.resetFIFO();
	//Clear overflow flag
	mpu.getIntStatus();
	fifoCount = 0;
}

//return true if yaw, pitch, roll was reveiced from the sensor MPU6050, false otherwise
bool MPUAbstraction::readYawPitchRoll(float ypr[3], int16_t acc[3]){
  // if programming failed or there is no data available, don't try to do anything
  if (mpuDataReady == FALSE || (devStatus != 0)) return FALSE;
  
  //reset interrupt-flag
  mpuDataReady = FALSE;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    Serial1.println(F("FIFO overflow!"));
	mpu.resetFIFO();
	fifoCount = 0;
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
    
    
    //get acceleration
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld((VectorInt16*)acc, &aaReal, &q);
    
    //invert yaw and pitch to correspond to the real world
    //roll is right from beggining.
    ypr[0] = (-ypr[0] + offsetYaw) * 180 / M_PI;
    ypr[1] = (-ypr[1] + offsetPitch) * 180 / M_PI;
	ypr[2] = (ypr[2] + offsetRoll) * 180 / M_PI;
    return TRUE;
  }
  return FALSE;
}
bool MPUAbstraction::deviceStatus(){
  return devStatus != 0 ? FALSE:mpu.testConnection();
}

 void MPUAbstraction::setHardwareOffset(float yaw, float pitch, float roll){
	 offsetYaw = yaw;
	 offsetPitch = pitch;
	 offsetRoll = roll;	 
 }
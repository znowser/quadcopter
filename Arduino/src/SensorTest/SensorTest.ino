//wire.h must be included here.
#include "Wire.h"
#include "MPU6050Abstraction.h"
#include "MS561101BA.h"

MPUAbstraction mpu = MPUAbstraction();
MS561101BA baro = MS561101BA();
int count = 0;
const float sea_press = 1013.25;
float temperature = 0, pressure = 0;

float getAltitude(float press, float temp);
void setup() {
  // initialize serial communication
  mpu.init();
  Serial.begin(115200); 
  //attach interruptfunction that is called whenever there are data to be read.
  attachInterrupt(0, MPUAbstraction::MPUInt, RISING);

  //wrong test?
  //Serial.println(!mpu.deviceStatus() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  baro.init(MS561101BA_ADDR_CSB_LOW);
  delay(100);
}

void loop() {
  float ypr[3];
  //switch to i2c master-mode
  
  if(mpu.readYawPitchRoll(ypr)){
    //temperature and pressure cannot be read directly after each other, it
    //must be a small delay between the two functioncalls.
    temperature = baro.getTemperature(MS561101BA_OSR_4096);
     Serial.print(ypr[0] * 180/M_PI);
     Serial.print("\t");
     Serial.print(ypr[1] * 180/M_PI);
     Serial.print("\t");
     Serial.print(ypr[2] * 180/M_PI);
     Serial.print("\t");  
     Serial.print("Altitude: ");
     Serial.print(getAltitude(pressure, temperature));   
     Serial.println(" m");
    pressure = baro.getPressure(MS561101BA_OSR_4096);
  }
}

float getAltitude(float press, float temp) {
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}




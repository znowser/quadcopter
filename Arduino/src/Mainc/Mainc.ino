#include <Servo.h>
#include "Motor.h"
#include "CellVoltage.h"
#include "Wire.h"
#include "MPU6050Abstraction.h"
#include "MS561101BA.h"
#include "SensorDataStruct.h"
#include "Hover.h"
//#include "SerialBuss.h"
#include "BussProtocol.h"
#include "vector_math.h"

//change this variable to true if you want to turn on the regulator, Torbjörn.
const bool regulator_activated = true;
const float sea_press = 1013.25;

/*========= Do NOT change this ===========*/
//Workaround to get normal program-flow of main function.
bool runOnce = true;
/*Do not use or touch these functions!*/
void setup() {}
void loop() {
  runOnce ? runOnce = mainf() : 0;
}
/*========================================*/

float getAltitude(float press, float temp);
float estimateHeight(int16_t accZ);
float getEstimatedAlt(float height);
float invSqrt(float number);
void updateSensorValues(sensordata &sensorData, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]);
//cannot be called with the name: main(), strange Arduino syndrome!
int mainf() {
  Serial.begin(115200);
  //SerialBuss serial;
  Motor motor[4];
  CellVoltage battery[3];
  sensordata sensorData;
  float ypr[3];

  //*==========Init Motors=============*/
  //Hardware mapping of motors
  motor[LF].init(10);
  motor[RF].init(11);
  motor[LB].init(12);
  motor[RB].init(13);
  /*==================================*/
  /*=====Init battery cells ==========*/
  //Hardware mapping of battery cells
  battery[CELL1].init(A0);
  battery[CELL2].init(A1);
  battery[CELL3].init(A2);
  /*==================================*/

  /*==========Init Sensors============*/
  //barometer and temperature
  MS561101BA baro = MS561101BA(MS561101BA_ADDR_CSB_LOW);
  //init gyro and magnetic field
  MPUAbstraction mpu = MPUAbstraction();
  /*===================================*/


  //serial.registerCallback(ps3DataCallback, &sensorData, PS3_CONTROLLER_PACKAGE);
  /*==========Hover regulator=============*/
  Hover regulator(motor, &sensorData, 0.5);
  //Main regulator/sensor loop
  int len = 0;
  char tmp[150];
  int sendCnt = 0;
  long mean = 0;
  while (true) {
    //TODO continue to implement the new buss protocol
    //  serial.recvRasp();

    //check if there is new sensordata to recieve from the sensor card
    if (mpu.readYawPitchRoll(ypr, sensorData.acc, sensorData.rawacc)) {
      //update sensor struct
      updateSensorValues(sensorData, motor, battery, baro, ypr);
      
      //Serial.println("================");
      /*Serial.print("Pressure ");
      Serial.print(sensorData.pressure / 100.0);
      Serial.print("   temp ");
      Serial.print(sensorData.temperature / 100.0);
      Serial.print("   Height: ");
      Serial.println(sensorData.height);*/
      //Serial.println("================");
      
     // Serial.print("Fused height: ");
      //Serial.println(getEstimatedAlt(sensorData.height));
      //Serial.print("sensor freq: ");
      //Serial.println(++sendCnt / (micros() / 1000000.f));
      float h = estimateHeight(sensorData.rawacc.z);
      if (++sendCnt % 10 == 0) {
        //Serial.write(buildSensorPackage(sensorData, tmp, len), len);
        Serial.print("Raw z acceleration ");
        Serial.println(h);
      }
      /*
      if (++sendCnt % 10 == 0) {
        Serial.write(buildSensorPackage(sensorData, tmp, len), len);
        Serial.println();
      }
      */
      //send the sensorstruct to the raspberry or regulate
      /*if (regulator_activated && regulator.Calibrate())
        regulator.Regulate();
        */
      //   else
      //     serial.sendRasp(SENSORDATA_PACKAGE, buildSensorPackage(sensorData, tmp, len), len);
    }
  }

  //return 0 if nothing more shall be executed, otherwise the main-function will
  //be called again.
  return 0;
}

float getAltitude(float press, float temp) {
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press / press), 1 / 5.257) - 1.0) * (temp + 273.15)) / 0.0065;
  
 // -KT*ln(Pn/P0)/mg = h
 // K boltzmann's constant
 // T Temperature in kelvin
 // Pn current pressure
 // P0 sea pressure
 // m Mass of one molecule = 29.95 (average of air)
 // g 9.8
 //-(temp + 273.15)*1.3806488*ln(press/sea_press)
}

void updateSensorValues(sensordata &sensorData, Motor motor[4], CellVoltage battery[3], MS561101BA &baro, float ypr[3]) {
  //temperature and pressure cannot be read directly after each other, it
  //must be a small delay between the two functioncalls.
  //sensorData.temperature = baro.getTemperature(MS561101BA_OSR_4096);
  //convert from radians to degrees
  sensorData.angleYaw = ypr[0]  * 180 / M_PI;
  sensorData.anglePitch = ypr[1]  * 180 / M_PI;
  sensorData.angleRoll = ypr[2]  * 180 / M_PI;
  sensorData.cellVoltage[CELL1] = battery[CELL1].getVoltage();
  sensorData.cellVoltage[CELL2] = battery[CELL2].getVoltage();
  sensorData.cellVoltage[CELL3] = battery[CELL3].getVoltage();
  //update motor speed
  sensorData.motorSpeed[LF] = motor[LF].getSpeed();
  sensorData.motorSpeed[RF] = motor[RF].getSpeed();
  sensorData.motorSpeed[LB] = motor[LB].getSpeed();
  sensorData.motorSpeed[RB] = motor[RB].getSpeed();
  //get pressure, cannot be done directly after getTemperature, The sensorcard need
  //a small delay between the function calls.
  //baro.getData(sensorData.temperature, sensorData.pressure, MS561101BA_OSR_4096);
  baro.getData(sensorData.temperature, sensorData.pressure, MS561101BA_OSR_256);
  //sensorData.pressure = baro.getPressure(MS561101BA_OSR_4096);
  sensorData.height = getAltitude(sensorData.pressure / 100.0, sensorData.temperature / 100.0);
}

// // complementary filter from MultiWii project v1.9
//
 #define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
 #define INIT_DELAY      4000000  // 4 sec initialization delay
 #define Kp1 0.55f                // PI observer velocity gain
 #define Kp2 1.0f                 // PI observer position gain
 #define Ki  0.001f               // PI observer integral gain (bias cancellation)
 #define dt  (UPDATE_INTERVAL / 1000000.0f)
//
// /**
// * Returns an altitude estimate from baromether fused with accelerometer readings
// */
float getEstimatedAlt(float height) {
  static uint8_t inited = 0;
  static int16_t AltErrorI = 0;
  static float AccScale  = 0.0f;
  static uint32_t deadLine = INIT_DELAY;
  int16_t AltError;
  int16_t InstAcc;
  int16_t Delta;
  static int32_t  EstVelocity;
  static int32_t  EstAlt;

  long currentTime = micros();

  //if (currentTime < deadLine) return EstAlt;
  //deadLine = currentTime + UPDATE_INTERVAL;
  // Soft start
Serial.print("Height : ");
Serial.println(height);
  if (!inited) {
    inited = 1;
    EstAlt = height;
    EstVelocity = 0;
    AltErrorI = 0;
  }
  // Estimation Error
  AltError = height - EstAlt;
  AltErrorI += AltError;
  AltErrorI = constrain(AltErrorI, -25000, +25000);
  // Gravity vector correction and projection to the local Z
  //InstAcc = (accADC[YAW] * (1 - acc_1G * InvSqrt(isq(accADC[ROLL]) + isq(accADC[PITCH]) + isq(accADC[YAW])))) * AccScale + (Ki) * AltErrorI;
#if defined(TRUSTED_ACCZ)
  InstAcc = (accADC[YAW] * (1 - acc_1G * InvSqrt(isq(accADC[ROLL]) + isq(accADC[PITCH]) + isq(accADC[YAW])))) * AccScale +  AltErrorI / 1000;
#else
  InstAcc = AltErrorI / 1000;
#endif

  // Integrators
  Delta = InstAcc * dt + (Kp1 * dt) * AltError;
//  EstAlt += (EstVelocity / 5 + Delta) * (dt / 2) + (Kp2 * dt) * AltError;
  EstVelocity += Delta * 10;


 // vmath::quat<float> ciao = vmath::quat<float>(0.0, 0.1, 0.2, 0.3);
  //vmath::quat<float> ciao2;

 // ciao = ciao * ciao2;


  //Serial.print(":::");
  //Serial.println(ciao.w);

  return EstAlt;
}

/**
 * Fast inverse square root implementation
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}

#define DT 0.2 // 10 ms
#define DEADZONE 0.3

//return estimated height in m.
float estimateHeight(int16_t accZ){
  //static float velocity[2];
  //static float position[2];
  //static float prevacc;
  //static uint8_t n;
  static float velocity;
  static float position;
  static unsigned long lastUpdate;
  
  
  if(micros() - lastUpdate < DT*1000000)
    return position;
  
  //set acc to zero if within deadzone otherwise convert to m/s^2.
  float acc = accZ < DEADZONE && accZ > -DEADZONE ? 0.0:accZ*0.0012043 - 9.82;
  
  velocity += acc*DT;
  position += velocity*DT;
  //velocity[n % 2] = velocity[(n + 1) % 2] + ((acc - prevacc)*DT / 2);
  //position[n % 2] = position[(n + 1) % 2] + ((velocity[n % 2] - velocity[(n + 1) % 2])*DT / 2);
  
  //++n;
  //prevacc = acc;
  //return position[n % 2];
  lastUpdate = micros();
  return position;
}

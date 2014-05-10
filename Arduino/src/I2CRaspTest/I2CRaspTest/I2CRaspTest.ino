#include <Wire.h>

#define SLAVE_ADDRESS 0x040
int number = 0;
int state = 0;

void setup() {
  //pinMode(13, OUTPUT);
  Serial.begin(9600);         // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveDataFromRasp);
  Wire.onRequest(sendDataToRasp);

  Serial.println("Ready!");
}

void loop() {
  delay(100);
}

// callback for received data
void receiveDataFromRasp(int byteCount){

  while(Wire.available()) {
    number = Wire.read();
    Serial.print("data received: ");
    Serial.println(number);
  }
}

// callback for sending data
void sendDataToRasp(){
  //Wire.write(number);
}


#include <string.h>
#include <stdlib.h>
#include "Arduino.h"
#include "SerialBuss.h"

SerialBuss::SerialBuss() {
  Serial.begin(BAUDRATE);
  
  readingHeader = true;
  dataPos = 0;
  dataLength = 0;
  memset(funcs, NULL, (NUM_OF_PACKAGES + 1)*sizeof(callbck));
  memset(additional_info, NULL, (NUM_OF_PACKAGES + 1)*sizeof(callbck));
  memset(headerBuffer, 0, HEADER_BUFFER_SIZE * sizeof(headerBuffer));
}

void SerialBuss::sendRasp(char id, unsigned len, char* data) {
  char length[10];
  memset(length, 0, 10);
  itoa(len, length, 10);
  int bytesSent = 0;
  int lenSize = strlen(length);

  //Ensure that all data is always sent
  while (Serial.write(id) != 1);
  while ((bytesSent += Serial.write(length + bytesSent, lenSize - bytesSent)) != lenSize);
  while (Serial.write(0x00) != 1);
  bytesSent = 0;
  while ((bytesSent += Serial.write(data + bytesSent, len - bytesSent)) != len);
}

void SerialBuss::recvRasp() {
  while (Serial.available() > 0) {
    if (readingHeader) {
      //Shall never happen but if it does prevent segment fault.
      if (dataPos >= HEADER_BUFFER_SIZE)
        dataPos = 0;

      headerBuffer[dataPos] = Serial.read();
      if (headerBuffer[dataPos] == 0x00) {
        readingHeader = false;
        dataPos = 0;
        //get package id
        packageID = headerBuffer[0];
        int i = 1;
        //determine length of the new package
        do {
          dataLength = ((headerBuffer[i] - 0x30) & 0x0F);
        } while (headerBuffer[++i] != 0x00 && i < HEADER_BUFFER_SIZE - 1);

        //allocate space for the new package
        data = new char[dataLength];
        //skip the ++dataPos
        continue;
      }
    }
    else {
      data[dataPos] = Serial.read();
      //check if whole package is received
      if (dataPos == dataLength - 1) {
        //Make the right action for each package
        if (packageID <= NUM_OF_PACKAGES && packageID >  0 && funcs[packageID] != NULL)
          funcs[packageID](data, dataLength, additional_info[packageID]);

        dataPos = 0;
        readingHeader = true;
        delete[] data;
        //skip the ++dataPos
        continue;
      }
    }
    ++dataPos;
  }
}

void SerialBuss::registerCallback(callbck func, void* ptr, char packageID) {
  if (packageID > NUM_OF_PACKAGES || packageID < 0)
    return;
  funcs[packageID] = func;
  additional_info[packageID] = ptr;
}

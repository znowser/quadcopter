#include <string.h>
#include <stdlib.h>
#include <wiringSerial.h>
#include "SerialBuss.h"

SerialBuss::SerialBuss() {
  fd = serialOpen(SERIAL_PORT, BAUDRATE);
  if(fd == -1){
	  cout << "Cannot open the serial port, try root?" << endl;
	  return;
  }

  readingHeader = true;
  dataPos = 0;
  dataLength = 0;
  memset(funcs, NULL, (NUM_OF_PACKAGES + 1)*sizeof(callbck));
  memset(additional_info, NULL, (NUM_OF_PACKAGES + 1)*sizeof(callbck));
  memset(headerBuffer, 0, HEADER_BUFFER_SIZE * sizeof(headerBuffer));
}

void SerialBuss::sendRasp(char id, char* data, unsigned len) {
  char length[10];
  memset(length, 0, 10);
  itoa(len, length, 10);
  int bytesSent = 0;
  int lenSize = strlen(length);

  //Ensure that all data is always sent
  serialPutchar(fd, id);
  while (bytesSent < lenSize){
	  serialPutchar(fd, *(length + bytesSent));
	  ++bytesSent;
  }
  serialPutchar(fd, 0x00);
  bytesSent = 0;
  
  while (bytesSent < len){
	  serialPutchar(fd, *(data + bytesSent));
	  ++bytesSent;
  }
}

void SerialBuss::recvRasp() {
	while (serialDataAvail(fd) > 0) {
    if (readingHeader) {
      //Shall never happen but if it does prevent segment fault.
      if (dataPos >= HEADER_BUFFER_SIZE)
        dataPos = 0;

	  headerBuffer[dataPos] = serialGetchar(fd);
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
		data[dataPos] = serialGetchar(fd);
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

void SerialBuss::close(){
	serialClose(fd);
}
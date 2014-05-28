#include <string.h>
#include <stdlib.h>
#include "Arduino.h"
#include "SerialBuss.h"

SerialBuss::SerialBuss() {
  Serial.begin(BAUDRATE);

  waitForSync = true;
  readingHeader = true;
  dataPos = 0;
  dataLength = 0;
  memset(funcs, NULL, (NUM_OF_PACKAGES + 1)*sizeof(callbck));
  memset(additional_info, NULL, (NUM_OF_PACKAGES + 1)*sizeof(callbck));
  memset(headerBuffer, 0, HEADER_BUFFER_SIZE * sizeof(headerBuffer));
}

void SerialBuss::sync() {
  char sync[] = { SYNC_DATA, SYNC_DATA, SYNC_DATA, SYNC_DATA };
  dataPos = 0;
  dataLength = 0;
  waitForSync = true;
  readingHeader = true;
  //cout << "SYNC PACKAGE SENT" << endl;
  sendRasp(SYNC_PACKAGE, sync, 4);
  //gettimeofday(&lastSync, NULL);
}

void SerialBuss::sendRasp(char id, char* data, unsigned len) {
  char length[10];
  memset(length, 0, 10);
  itoa(len, length, 10);
  int bytesSent = 0;
  int lenSize = strlen(length);

  //Ensure that all data is always sent
  /*while (Serial.write(id) != 1);
  while ((bytesSent += Serial.write(&length[lenSize - 1] - bytesSent, 1)) != lenSize);
  while (Serial.write(0x00) != 1);
  bytesSent = 0;
  while ((bytesSent += Serial.write(data + bytesSent, len - bytesSent)) != len);*/

  Serial.write(id);
  for (int i = lenSize - 1; i >= 0; --i)
    Serial.write(&length[i], 1);
  Serial.write(0x00);
  Serial.write(data, len);
}

void SerialBuss::recvRasp() {
  /*if (waitForSync) {
    timeval curtime;
    gettimeofday(&curtime, NULL);
    if (curtime.tv_sec > lastSync.tv_sec + SYNC_TIMEOUT)
      sync();
  }*/

  while (Serial.available() > 0) {
    //printf("%c", serialGetchar(fd));
    //cout << (char)serialGetchar(fd)<< " ";
    //check if the buss needs to be synchronized

    if (waitForSync) {
      headerBuffer[dataPos % HEADER_BUFFER_SIZE] = Serial.read();
      waitForSync = !(headerBuffer[dataPos % HEADER_BUFFER_SIZE] == SYNC_DATA &&
                      headerBuffer[(dataPos - 1) % HEADER_BUFFER_SIZE] == SYNC_DATA &&
                      headerBuffer[(dataPos - 2) % HEADER_BUFFER_SIZE] == SYNC_DATA &&
                      headerBuffer[(dataPos - 3) % HEADER_BUFFER_SIZE] == SYNC_DATA);
      // cout << "header buffer: " << hex << (int)headerBuffer[0] << " " << (int)headerBuffer[1] << " " << (int)headerBuffer[2] << " " << (int)headerBuffer[3] << " " << (int)headerBuffer[4] << endl;
      if (!waitForSync) {
        //tell the rasp that the sync package was received
        sync();
        waitForSync = false;
        //cout << "SYNC PACKAGE RECEIVED" << endl;
        dataPos = 0;
        //serialFlush(fd);
        //skip the ++dataPos
      }

      else
        dataPos = (dataPos + 1) % HEADER_BUFFER_SIZE;
    }
    //read packages as normal
    else {
      if (readingHeader) {
        //Shall never happen but if it does prevent segment fault.
        if (dataPos >= HEADER_BUFFER_SIZE) {
          //cout << "header buffer: " << hex << (int)headerBuffer[0] << " " << (int)headerBuffer[1] << " " << (int)headerBuffer[2] << " " << (int)headerBuffer[3] << " " << (int)headerBuffer[4] << endl;
          //cout << "out of header dataPos" << dataPos << endl;
          sync();
        }
        else {
          headerBuffer[dataPos] = Serial.read();
          if (headerBuffer[dataPos] == 0x00) {
            readingHeader = false;
            dataPos = 0;
            //get package id
            packageID = headerBuffer[0];
            int i = 1;
            //determine length of the new package
            do {
              dataLength = dataLength + (headerBuffer[i] - 0x30) * potensOfTen(i - 1);
            } while (headerBuffer[++i] != 0x00 && i < HEADER_BUFFER_SIZE);
            //cout << "Header received id " << packageID << " length " << dataLength << endl;
            //cout << "header buffer: " << hex << (int)headerBuffer[0] << " " << (int)headerBuffer[1] << " " << (int)headerBuffer[2] << " " << (int)headerBuffer[3] << endl;
            //allocate space for the new packag
            if (dataLength > MAX_PACKAGE_SIZE) {
              //cout << "THRASHING PACKAGE" << endl;
              sync();
            }
            else
              data = new char[dataLength];
          }
          else
            ++dataPos;
        }
      }
      else {
        data[dataPos] = Serial.read();
        //check if whole package is received
        if (dataPos == dataLength - 1) {
          //cout << "Received whole package, id " << packageID << " length " << dataLength << endl;
          //Make the right action for each package
          if (packageID <= NUM_OF_PACKAGES && funcs[packageID] != NULL)
            funcs[packageID](data, dataLength, additional_info[packageID]);
          dataLength = 0;
          dataPos = 0;
          readingHeader = true;
          delete[] data;
        }
        else
          ++dataPos;
      }
    }
  }
}

void SerialBuss::registerCallback(callbck func, void* ptr, char packageID) {
  if (packageID > NUM_OF_PACKAGES || packageID < 0)
    return;
  funcs[packageID] = func;
  additional_info[packageID] = ptr;
}

unsigned SerialBuss::potensOfTen(unsigned i) {
  switch (i) {
    case 1:
      return 10;
    case 2:
      return 100;
    case 3:
      return 1000;
    case 4:
      return 10000;
    default:
      return 1;
  }
}



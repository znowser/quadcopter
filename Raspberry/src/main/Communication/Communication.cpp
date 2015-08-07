/*
 * Communication.cpp
 *
 * Created: 2015-07-29 16:53:12
 *  Author: Erik
 */
#include <wiringSerial.h>
#include <iostream> 
#include <string.h>
#include "Communication.h"
#include "crc16.h"

using namespace std;

int Communication::fd = 0;
int Communication::packetLength = 0;
char Communication::buffer[HDLC_MAX_LGT];
bool Communication::active = false;

void Communication::init(){
  memset(buffer, 0x00, sizeof(buffer));
  fd = serialOpen(SERIAL_PORT, BAUDRATE);
  if(fd == -1){
    cout << "Cannot open the serial port, try root?" << endl;
    return;
  }
}

bool Communication::receive(char* &packet, int &packetLength){
  bool res = false;
  int statusRX = hdlc_RX(rx, buffer ,&packetLength, sizeof(buffer));
  packet = NULL;

  if(statusRX == 0){
    if(packetLength > 0){
      res = true;
      packet = buffer;
    }
  }
  
  return res;
}

int Communication::send(char *data, int len){		
  return hdlc_TX(snd, data, len);
}

/* send callback used by the hdlc implementation returns 0 on success */
int Communication::snd(char c){
  serialPutchar(fd, c);
  return 0;
}

/* receive callback used by the hdlc implementation returns 0 on success */
int Communication::rx(char *c){
  time_t start = time(NULL);

  /*  while(serialDataAvail(fd) <= 0){
    if(time(NULL) - start > SERIAL_TIMEOUT){
      active = false;
      return -1;
    }
  };
  
  active = true;*/

  if(serialDataAvail(fd) <= 0)
    return -1;

  *c = serialGetchar(fd);
  return 0;
}

bool Communication::isActive(){
  return active;
}

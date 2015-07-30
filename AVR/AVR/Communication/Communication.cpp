/*
 * Communication.cpp
 *
 * Created: 2015-07-29 16:53:12
 *  Author: Erik
 */ 
#include "Communication.h"
#include "crc16.h"

int Communication::packetLength = 0;
char Communication::buffer[HDLC_MAX_LGT];

void Communication::init(){
	// Setup serial connection
	Serial1.begin(SERIAL_BAUDRATE);
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

int Communication::send(char *text){
	return hdlc_TX(snd, text, strlen(text));
}

/* send callback used by the hdlc implementation returns 0 on success */
int Communication::snd(char c){
	Serial1.print(c);
	return 0;
}

/* receive callback used by the hdlc implementation returns 0 on success */
int Communication::rx(char *c){
	if(!Serial1.available())
		return -1;
	
	*c = Serial1.read();
	return 0;
}
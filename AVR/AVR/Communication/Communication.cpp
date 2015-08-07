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
unsigned long Communication::deadline = 0;
unsigned long Communication:: startTime = 0;
int Communication::numOfTimeouts = 0;

void Communication::init(unsigned long deadline){
	// Setup serial connection
	Serial1.begin(SERIAL_BAUDRATE);
	
	this->deadline = deadline;
}

/*
* The receive function waits for or receive data until the deadline is met.
* if a whole packet is received within the deadline the function returns.
* If the packet is not received within the deadline the function continues 
* receiving the rest of the packet the next time the function is called.
*/
bool Communication::receive(char* &packet, int &packetLength){
	bool res = false;
	startTime = millis();
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

int Communication::send(const char *data, int len){		
	return hdlc_TX(snd, data, len);
}

int Communication::send(const char *text){
	return hdlc_TX(snd, text, strlen(text));
}

/* send callback used by the hdlc implementation returns 0 on success */
int Communication::snd(const char c){
	Serial1.print(c);
	return 0;
}

/* receive callback used by the hdlc implementation returns 0 on success */
int Communication::rx(char *c){	
	while(!Serial1.available()){
		if(millis() - startTime >= deadline){
			++numOfTimeouts;
			return -1;
		}
	}
	
	*c = Serial1.read();
	return 0;
}

int Communication::getTimeouts(){
	return numOfTimeouts;
}

void Communication::resetTimeouts(){
	numOfTimeouts = 0;
}
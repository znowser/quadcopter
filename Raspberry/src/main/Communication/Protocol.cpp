/*
 * Protocol.cpp
 *
 * Created: 2015-08-07 19:04:54
 *  Author: Erik
 */ 


#include "Protocol.h"
/*

bool Protocol::decode(sensordata *data, char *packet, unsigned length){
	
	if(length != packet[LENGTH_POSITION]){
		Serial1.println("Length not valid");
		return false;
	}
	
	switch(packet[PACKET_TYPE_POSITION]){
		case PS3_CONTROLLER_DATA:
		Serial1.println("Received ps3");
		break;
		
		default:
			return false;
	}
	
	
	return true;
}*/

void Protocol::encodePS3(wwvi_js_event *ps3,char *encoding, int *len){
 
  *len = 2;

  for(int i = 0; i < NUM_OF_BUTTONS; ++i)
    encoding[(*len)++] = ps3->button[i];
  encoding[LENGTH_POSITION] = *len;
  encoding[PACKET_TYPE_POSITION] = PS3_CONTROLLER_DATA;
}

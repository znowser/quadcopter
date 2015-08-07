/*
 * Protocol.cpp
 *
 * Created: 2015-08-07 19:04:54
 *  Author: Erik
 */ 


#include "Protocol.h"


bool Protocol::decode(sensordata *data, char *packet, unsigned length){
	bool res = true;
	
	if(length != packet[LENGTH_POSITION])
		res = false;
	else{
		switch(packet[PACKET_TYPE_POSITION]){
			case PS3_CONTROLLER_DATA:
				for(int i = 2; i < length; ++i)
					data->ps3.button[i - 2] = packet[i];
			break;
			
			default:
				res = false;
		}
	}
	return res;
}
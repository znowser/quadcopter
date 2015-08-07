/*
 * IncFile1.h
 *
 * Created: 2015-08-07 19:04:26
 *  Author: Erik
 */ 


/*
	Simple implementation of a communication protocol. 
	First byte of the packet determines the type of the packet.
	Second byte determines the length, the length is the number of bytes
	inclusive the header. The header is only two bytes. After the 
	header is up to 255 bytes of data.
*/

#ifndef PROTOCOL_H__
#define PROTOCOL_H__

#include "../SensorDataStruct.h"


#define PACKET_TYPE_POSITION 0
#define LENGTH_POSITION 1


#define PS3_CONTROLLER_DATA 0

class Protocol{	
	public:
		static bool decode(sensordata *data, char *packet, unsigned length);
	};



#endif /* PROTOCOL_H__ */
/*
 * Communication.h
 *
 * Created: 2015-07-29 16:52:53
 *  Author: Erik
 */ 


#ifndef COMMUNICATION_H___
#define COMMUNICATION_H___

#include "../lib/Arduino/Arduino.h"
#include "hdlc.h"

/*
 Maximum packet size is HDLC_MAX_LGT = 128 bytes, do NOT try to send larger packets!
*/

#define SERIAL_BAUDRATE 115200

class Communication{
	private:
		static int packetLength;
		static char buffer[HDLC_MAX_LGT];
		static int snd(char c);
		static int rx(char *c);
		static int dataPos;
	public:
		void init();
		static bool receive(char* &packet, int &packetLength);
		/*Send data of length len*/
		static int send(char *data, int len);
		/*Send null terminated text string*/
		static int send(char *text);
};



#endif /* COMMUNICATION_H___ */
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
 
 The Communication will return when the deadline is met.
*/

#define SERIAL_BAUDRATE 115200

class Communication{
	private:
		static int packetLength;
		static char buffer[HDLC_MAX_LGT];
		static int snd(const char c);
		static int rx(char *c);
		static int dataPos;
		static unsigned long deadline, startTime;
		static int numOfTimeouts;
	public:
		void init(unsigned long deadline);
		static bool receive(char* &packet, int &packetLength);
		/*Send data of length len*/
		static int send(const char *data, int len);
		/*Send null terminated text string*/
		static int send(const char *text);
		static int getTimeouts();
		static void resetTimeouts();
};

#endif /* COMMUNICATION_H___ */
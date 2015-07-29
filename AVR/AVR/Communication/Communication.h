/*
 * Communication.h
 *
 * Created: 2015-07-29 16:52:53
 *  Author: Erik
 */ 


#ifndef COMMUNICATION_H___
#define COMMUNICATION_H___

#include "../lib/Arduino/Arduino.h"
#include "../lib/oshdlc/hdlclib.h"

/*
 Maximum packet size is 256, do NOT try to send larger packets!
*/

class Communication{
	private:
	hdlc_chan_t *p_hdlc;
	uint8_t enc_out[HDLC_PKT_MAXLEN];
	uint8_t recvBuffer[HDLC_PKT_MAXLEN];
	uint16_t pos;
	
	public:
	Communication();
	int receive();
	void send(const uint8_t *data, int len);
};



#endif /* COMMUNICATION_H___ */
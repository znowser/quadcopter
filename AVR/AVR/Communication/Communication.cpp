/*
 * Communication.cpp
 *
 * Created: 2015-07-29 16:53:12
 *  Author: Erik
 */ 
#include "Communication.h"

Communication::Communication(){
	HDLC_createChannel();
	pos = 0;
}

int Communication::receive(){
	hdlc_packet_t *dec_packet;
	
	while(Serial1.available()){
		recvBuffer[pos++] = Serial1.read();
		
		//Check if a complete packet has been received
		if(HDLC_decode(p_hdlc, recvBuffer, HDLC_PKT_MAXLEN)){
			dec_packet = HDLC_getDecodedPacket(p_hdlc);
			//TODO decode packet...
			HDLC_freePacket ( &dec_packet );
			pos = 0;
		}	
	}
	
	return 0;
}

void Communication::send(const uint8_t *data, int len){
	int enc_osize = HDLC_encode(p_hdlc, data, len, enc_out, sizeof(enc_out));
	
	for(int i = 0; i < enc_osize; ++i)
		Serial1.print(enc_out[i]);
}
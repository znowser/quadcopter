/*
 * Communication.h
 *
 * Created: 2015-07-29 16:52:53
 *  Author: Erik
 */ 


#ifndef COMMUNICATION_H___
#define COMMUNICATION_H___

#include "hdlc.h"

#define BAUDRATE 115200
#define SERIAL_PORT "/dev/ttyAMA0"

/* Time until the serial communication discards the current packe t*/
#define SERIAL_TIMEOUT 1
/*
 Maximum packet size is HDLC_MAX_LGT = 128 bytes, do NOT try to send larger packets!
*/

class Communication{
 private:
  static int fd;
  static int packetLength;
  static char buffer[HDLC_MAX_LGT];
  static bool active;
  static int snd(char c);
  static int rx(char *c);
 public:
  static void init();
  static bool receive(char* &packet, int &packetLength);
  static int send(char *data, int len);
  static bool isActive();
};



#endif /* COMMUNICATION_H___ */

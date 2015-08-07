/*****************************************************
* HDLC byte stuffing package                         *
*                                                    *
*  Created on: Dec 2013                              *
*      Author: jdn                                   *
*                                                    *
******************************************************
*                                                    *
*                                                    *
* (C) 2012,2013                                      *
*                                                    *
* Jens Dalsgaard Nielsen <jdn@es.aau.dk>             *
* http://www.control.aau.dk/~jdn                     *
* Studentspace/Satlab                                *
* Section of Automation & Control                    *
* Aalborg University,                                *
* Denmark                                            *
*                                                    *
* "THE BEER-WARE LICENSE" (frit efter PHK)           *
* <jdn@es.aau.dk> wrote this file. As long as you    *
* retain this notice you can do whatever you want    *
* with this stuff. If we meet some day, and you think*
* this stuff is worth it ...                         *
*  you can buy me a beer in return :-)               *
* or if you are real happy then ...                  *
* single malt will be well received :-)              *
*                                                    *
* Use it at your own risk - no warranty              *
*****************************************************/


#include "hdlc.h"

using namespace std;
/*  SIMPLE HDLC TX/RX
* BYTE STUFFING VRS
* Dont care about contents and flags
* Need jus an arry of bytes to send and length
* and send function

* /Jens
* flag == 0x7e !!!
*
* [0x7e | address | control | info | fcs | 0x7e]
*/



#define TX(tx,c) if ( 0 != (*tx)(c))  goto exit
#define TX_STUFF(tx,c) if ( 0 != stuff_tx(tx,c))  goto exit

#define RX(rx,pC) if (0 != (*rx)(pC)) goto exit


static bool flipNext = false;
static unsigned short dataPos = 0;

//----------------------------------------------------------------------------
static int stuff_tx( int (*ch_tx)(char ch), char c)
{

  if (c != HDLC_FLAG  &&  c != HDLC_STUFF_CH) {
    return (*ch_tx)(c);
  } else {
    if ((*ch_tx)(HDLC_STUFF_CH) == 0) {
      return (*ch_tx)(c^BIT_FLIP);
    } else
      return (-1);
  }
}

//----------------------------------------------------------------------------
int hdlc_TX( int (*ch_tx)(const char ch),  const char *data ,int len)
{

  if (HDLC_MAX_LGT < len)
    return (-1);

  TX(ch_tx,HDLC_FLAG); // start

  while (len--) {
    TX_STUFF(ch_tx,*data);
    data++;
  }

  TX(ch_tx,HDLC_FLAG); // start/stop

  return (0); // ok
exit:
  return (-1); // err
}

//----------------------------------------------------------------------------
static int stuff_rx( int (*ch_rx)(char *ch), char *c){
	/* Receive byte from the channel*/
	RX(ch_rx,c);
	
	/* Check if the previous byte was an stuffed byte which indicates that
	   the newly received byte should be flipped */
	if(flipNext){
		*c ^= BIT_FLIP;
		flipNext = false;
	}

	/* start or stop of frame */
	if (*c == HDLC_FLAG)
		return (0); 

	/* Detect stuff byte */
	if (*c == HDLC_STUFF_CH) 
		flipNext = true;
	  
  return (1);
exit:
  return (2);
}

//----------------------------------------------------------------------------
int hdlc_RX(int (*ch_rx)(char * ch), char *buffer , int *packetLenght, int bufferSize){
	char c;
	*packetLenght = 0;

	if (HDLC_MAX_LGT > bufferSize)
		return (-11);
		
	do {
		switch (stuff_rx(ch_rx,&c)) {
		  /* Start or stop byte encountered */
		  case 0:
			*packetLenght = dataPos;
			dataPos = 0;
			return 0;
			break;
			
		  /* Continue to read loop */
		  case 1:
			break;
			
		  /* cant read stuff */
		  case 2:
		  default:
			return -1;
		}
		buffer[dataPos++] = c;
  } while (dataPos < HDLC_MAX_LGT);
  
  /* Only possible way to get here is to have the buffer
     filled up and no end flag received. Discard the buffer
	 and start looking for a new frame. 
  */
  dataPos = 0;
  return -1;
}

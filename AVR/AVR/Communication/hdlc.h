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
#ifndef HDLC_BYTE
#define HDLC_BYTE


/*  SIMPLE HDLC TX/RX
* BYTE STUFFING VRS
* Dont care about contents and flags
* Need just an arry of bytes to send and length
* and send function
* you have to supply with a send and receive function
* /Jens
* flag == 0x7e !!!
*
* HDLC frame format - small version
* [0x7e | address | control | info | fcs | 0x7e]
*   1B      1B        1B        1B    xx    1B
* NB addres, control info and fcs shall all
* be in the byte array you are coming with
* So this is only a "frame protocol"
*/

#define HDLC_FLAG 0x7E
#define HDLC_STUFF_CH 0x7D
#define HDLC_MAX_LGT 128

#define BIT_FLIP_NR 5
#define BIT_FLIP 0x10



//----------------------------------------------------------------------------
int hdlc_TX( int (*ch_tx)(const char ch),  const char *data ,int len);


//----------------------------------------------------------------------------
int hdlc_RX( int (*ch_rx)(char * ch), char *buffer , int *packetLenght, int bufferSize);


#endif

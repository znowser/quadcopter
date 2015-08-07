#include "crc16.h"

// (C) Jens Dalsgaard Nielsen
// Beer license :-)

/*****************************************************
* CRC16  package                                     *
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
// performance on arduino uno
// 20000 loops with add_crc 
// on a  16B message takes  612 milliseconds
// on a  32B message takes 1175 milliseconds
// on a 100B message takes 3569 milliseconds
// on a 200B message takes 7091 millisecond
//
// so add_crc on one package takes
//   16B takes  30 usec
//   32B takes  59 usec
//  100B takes 178 usec
//  200B takes 354 usec
// ----------------------
// code ...
// char s[300]; // just for testing
//  Serial.print("start "); Serial.print(millis());
//  for (j=0; j < 20000; j++)
//    add_crc16(s,100); 
//  erial.print("stop "); Serial.print(millis());

#define FCSINIT 0xFFFF
#define FCSGOOD 0xF0B8
#define FCSSIZE 2 
/* 16 bit FCS */
 
/* 16 bit FCS lookup table per RFC1331 */

const uint16_t fcstab[] PROGMEM = {
  //int fcstab[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

//---------------------------------------------------------------------------
unsigned int do_crc16(char buf[],int l)
{
  int i,indx;
  unsigned int j ,fcsval = FCSINIT;

  for(i=0 ; i< l ; i++) {
    indx = (fcsval ^ buf[i]) & 0xff;
    j = pgm_read_word_near(fcstab + indx); 
    fcsval =  (fcsval >> 8) ^ j;
    //fcsval = (fcsval >>8) ^ fcstab[ (fcsval ^ inbuf[i]) & 0xff ];
  }
  return fcsval;
}

//---------------------------------------------------------------------------
int add_crc16(char buf[], int l)
{
  unsigned int crc;
  crc = do_crc16(buf,l);
  crc = ~crc;
  buf[l] = crc & 0xff; /* Least significant byte of FCS */
  buf[l+1] = (crc >> 8) & 0xff; /* MSB of FCS */
}

//---------------------------------------------------------------------------
 char chk_crc16(char buf[], int l)
{
  int i,indx;
  unsigned int j ,fcsval = FCSINIT;

  for(i=0 ; i< l ; i++) {
    indx = (fcsval ^ buf[i]) & 0xff;
    j = pgm_read_word_near(fcstab +indx); 
    fcsval =  (fcsval >> 8) ^ j;
    //fcsval = (fcsval >>8) ^ fcstab[ (fcsval ^ inbuf[i]) & 0xff ];
  }
  if (fcsval ==  FCSGOOD)
    return (0); 
  else
    return(1);
}

// JDN

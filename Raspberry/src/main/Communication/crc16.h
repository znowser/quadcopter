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

#ifndef crc16jdn
#define crc16jdn

//#include <avr/pgmspace.h>  // bq we have crc16 table in prog mem space for saving RAM

/*
* add_crc uses 2 bytes in end of buf to add crc16 !
*
*
*/
unsigned int do_crc16(char buf[],int l);
int add_crc16(char buf[], int l);
char chk_crc16(char buf[], int l);
#endif

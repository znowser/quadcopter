A SMALL HDC TESTER

send by seoftserial and receive on std serial -

add a wire from dig port 11 to RX pin (pin 0)

NB:
full configurable so you do also comes with tx/rx functions

In this small example they are snd and rx

/Jens
-------


#include <hdlc.h>
#include <crc16.h>
#include <SoftwareSerial.h>

SoftwareSerial tx(10,11); // rx tx


int snd(char c)
{
 // just send by softserial

  tx.write(c);
  return (0);
}

int rx(char *c)
{
  unsigned char cc;
 // I assume you will return with something and return(0)
 // or return(-1) == timeout/error or whateve
 // IN this test we will wait forever on a char ... 

  while (Serial.available()<= 0);

  *c = Serial.read();

  return (0);
}

char s[300] = "AAABBBCCCDDDEEEFF";
char s2[110];
int i;


void setup()
{
  Serial.begin(9600); // NB remember wire from dig pin 11 to dig pin 0
  tx.begin(9600); // for debug
}

void loop()
{
  int j;

  static char a='0';

  // fill s array with 0's 
  for (j=0; j < 100; j++) {
    s[j]=a;
  }

  a++; // so we in next loop transmits '1' then '2 etc 
  if ('9' < a)
    a = '0';
    
  add_crc16(s,16); 

  hdlc_TX(snd,s,16+2); // send string via snd function NB 16+2 bq crc16 adds 2 bytes to string
                       // hdlc_TX dont care if it is a crc16 chl sum. It just tx a string and o byte stuffing with 7E's


  for (i = 0;i < 50;i++) // reset string for receive
    s2[i] = 0x00;

  hdlc_RX(rx,0,s2,&i,100);

  Serial.print("lgt "); 
  Serial.print(i); 
  Serial.print(" [");

  for (j=0; j < i; j++)
    Serial.print(s2[j]);

  Serial.println("]"); 


  if (0 == chk_crc16(s2,i))
    Serial.println(" crc ok");
  else {
    Serial.println(" crc nope");
    delay(500);
  }

  delay(1000);
}







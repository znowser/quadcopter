#include <stdio.h>
#include <string.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdlib>
#include <netdb.h>
#include <unistd.h>
#include <wiringSerial.h>
#include <iostream>
#include <sys/time.h>


#define SERIALPORT "/dev/ttyAMA0"

using namespace std;

int diff_ms(timeval t1, timeval t2);
int main(int argc, char *argv[])
{
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;


  if (argc < 3) {
    fprintf(stderr,"usage %s hostname port\n", argv[0]);
    exit(0);
  }
  portno = atoi(argv[2]);
  /* Create a socket point */
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) 
    {
      perror("ERROR opening socket");
      exit(1);
    }
  server = gethostbyname(argv[1]);
  if (server == NULL) {
    fprintf(stderr,"ERROR, no such host\n");
    exit(0);
  }

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, 
	(char *)&serv_addr.sin_addr.s_addr,
	server->h_length);
  serv_addr.sin_port = htons(portno);

  /* Now connect to the server */
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
    {
      perror("ERROR connecting");
      exit(1);
    }


  /*==== open serial interface added by erik ====*/
  int fd = serialOpen(SERIALPORT, 115200);
  bool currentPackage = false;
  bool whichbuffer = false;
  char buffer1[256], buffer2[256];

  if(fd == -1){
    cout << "Cannot open the serial port, The program must be executed as root" << endl;
    return 0;
  }
  /*=============================================*/


  /* Now ask for a message from the user, this message
   * will be read by server
   */
  int bufferPos1 = 0, bufferPos2 = 0;
  int dataSize = 0;
  bool throwFirstPackage = false;
  struct timeval last, current;


  //send only two times per sec
  gettimeofday(&current, NULL);
  last = current;

  while (true){   
      gettimeofday(&current, NULL);
      
      while(serialDataAvail(fd)){
	char c = (char)serialGetchar(fd);	
	//cout << c; 
	//check if the first package has been throwed away
	//must be dne to ensure that all nextcomming packages
	//are in sync.
	if(throwFirstPackage == false){
	  if(c == '\n')
	    throwFirstPackage = true;
	}
	//first package has been thrashed, read to buffer
	else{
	  if(c == '\n'){
	    whichbuffer = !whichbuffer;
	    dataSize = whichbuffer ? bufferPos2 : bufferPos1;
	  }
	  else{
	    if(whichbuffer){
	      buffer1[bufferPos1] = c;
	      ++bufferPos1;
	      bufferPos2 = 0;
	    }
	    else{
	      buffer2[bufferPos2] = c;
	      ++bufferPos2;
	      bufferPos1 = 0;
	    }
	  }	
	}
      }   

      //send update to server every 250ms
      if(diff_ms(current, last) > 250 && throwFirstPackage){
	//debug print, remove later!
	cout << "sending " << dataSize << endl;
	/* Send message to the server */
	//send from the complete buffer
	if(whichbuffer)
	  n = write(sockfd, buffer2, dataSize);
	else
	  n = write(sockfd, buffer1, dataSize);
	
	if (n < 0) {
	  perror("ERROR writing to socket");
	  exit(1);
	}
	gettimeofday(&last, NULL);
      }
  }
  return 0;
}
  
int diff_ms(timeval t1, timeval t2){
  return (((t1.tv_sec - t2.tv_sec) * 1000000) + 
	  (t1.tv_usec - t2.tv_usec))/1000;
}

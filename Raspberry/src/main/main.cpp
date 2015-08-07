#include <iostream>
#include <string.h>
#include "CJoystick.h"
#include "Communication/Communication.h"
#include "Communication/Protocol.h"

using namespace std;


int main(){

  Communication com;
  
  struct wwvi_js_event controller;
  memset(&controller, 0, sizeof(wwvi_js_event));
  PS3Controller ps3;
  

  int packetLength;
  int i = 0;
  char *packet = NULL;

  char encodingBuffer[100];
  int  encodingLen = 0;

  if (!ps3.open_joystick(JOYSTICK_DEVNAME)) {
    printf("failed to initialize the PS3 controller.\n");
    return 0;
  }

  com.init();
  cout << "Starting..." << endl;


  while(true){

    //update the status of the ps3 controller
    ps3.get_joystick_status(&controller);

    //Check if the aircraft want to tell us something..
    if(com.receive(packet, packetLength)){
      cout << "packet received(" << packetLength << "): ";
      for(int c = 0; c < packetLength; ++c)
	  cout << packet[c];
      cout << endl;
      }
    
    //Send data from the ps3 controller 
    if(++i % 10000 == 0){
      
      //Encode and send packet
      Protocol::encodePS3(&controller, encodingBuffer, &encodingLen);
      com.send(encodingBuffer, encodingLen);
      /*
      for(int c = 0; c < NUM_OF_BUTTONS; c++){
      	printf("%i ", controller.button[c]);
      }
      cout << endl;
      */
    }
  }
  /*
    
    
    while (true){
    ps3.get_joystick_status(&status);
    
    cout << "x: " << status.stick1_x / 256<< " y: " << status.stick1_y /256 << endl;
    usleep(100000);
    }
  */
  return 0;
}

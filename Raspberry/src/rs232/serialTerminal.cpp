#include <iostream>
#include <wiringSerial.h>

using namespace std;

int main(){

  int fd = serialOpen ("/dev/ttyAMA0", 115200);
  char bufferData[100];

  if(fd == -1){
    cout << "Cannot open the serial port, try root?" << endl;
    return 0;
  }

  cout << "Listening after data on serial port..." << endl;
  while(true){
    //serialPutchar(fd, 'E'); 
    while(serialDataAvail(fd)){
      cout << (char)serialGetchar(fd);
    }
  }

  serialClose(fd);
  return 0;
}

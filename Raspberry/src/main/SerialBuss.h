#ifndef SERIALBUSS_H
#define SERIALBUSS_H

//only needed for the #define NUM_OF_PACKAGES ..n 
#include "BussProtocol.h"

#define BAUDRATE 115200
#define SERIAL_PORT "/dev/ttyAMA0"
#define HEADER_BUFFER_SIZE 11
//This define must be updated if new packages are added

//=== Package format ===
//    package-id      length of package data    0x00                                     data
//    byte            string                    tag to identify end of header            whatever

//All packages are handled by callback functions
//When a package is received the callback associated with the package-id is called.

typedef void (*callbck)(char *data, int len, void *additional_info);

class SerialBuss {
  private:
	int fd;
    bool readingHeader;
    char headerBuffer[HEADER_BUFFER_SIZE];
    unsigned packageID;
    int dataPos;
    char *data;
    int dataLength;
    callbck funcs[NUM_OF_PACKAGES + 1];
    void* additional_info[NUM_OF_PACKAGES + 1];
  public:
    SerialBuss();
    void registerCallback(callbck func, void *ptr, char id);
    void sendRasp(char id, char* data, unsigned len);
    void recvRasp();
	void close();
};

#endif

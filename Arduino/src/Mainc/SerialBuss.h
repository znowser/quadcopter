#ifndef SERIALBUSS_H
#define SERIALBUSS_H

#define BAUDRATE 115200

//Protocol definitions
#define TEXT_PACKAGE 0x01
#define PS3_CONTROLLER_PACKAGE 0x02
#define EMERGENCY_STOP_PACKAGE 0x03

#define HEADER_BUFFER_SIZE 11
//This define must be updated if new packages are added
#define NUM_OF_PACKAGES 3

//=== Package format ===
//    package-id      length of package data    0x00                                     data
//    byte            string                    tag to identify end of header            whatever

//All packages are handled by callback functions
//When a package is received the callback associated with the package-id is called.

typedef void (*callbck)(char *data, int len, void *additional_info);

class SerialBuss {
  private:
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
    void sendRasp(char id, unsigned len, char* data);
    void recvRasp();
};

#endif

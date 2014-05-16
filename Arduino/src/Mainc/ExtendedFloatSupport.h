#ifndef EXTENDED_FLOAT_SUPPORT
#define EXTENDED_FLOAT_SUPPORT

void fmtDouble(double val, char precision, char *buf, unsigned bufLen = 0xffff);
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen = 0xffff, char width = 0);

#endif

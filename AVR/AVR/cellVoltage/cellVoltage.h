#ifndef CELLVOLTAGE_H
#define CELLVOLTAGE_H
/* 
  Class that make it possible to read the voltage of lipo-batterys cells.
  The cells are wired to input ADC0, ADC1 and ADC2. 8-bit resolution is used.
  
  The value returned by getVoltage is between 0-255, where 255 represent 4.20V
  and 0 represents 0V.
*/

//Hardware constant
#define VREF 5.0

class CellVoltage{
private:
	int pin;
public:
    CellVoltage(){};
	CellVoltage(int pin);
    void init(int pin);
	byte getVoltage();
};
#endif

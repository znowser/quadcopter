#ifndef CELLVOLTAGE_H
#define CELLVOLTAGE_H
/* 
  Class that make it possible to read the voltage of lipo-batterys cells.
  The cells are wired to input ADC0, ADC1 and ADC2. 8-bit resolution is used.
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
	float getVoltage();
};
#endif

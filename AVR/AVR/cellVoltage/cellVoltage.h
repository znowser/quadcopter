#ifndef CELLVOLTAGE_H
#define CELLVOLTAGE_H
/* 
  Class that make it possible to read the voltage of lipo-batterys cells.
  The cells are wired to input A0, A1 and A2.
*/
//safe limit, may be decresed 
#define CELL_VOLTAGE_MIN 3.5
#define CELL_VOLTAGE_MAX 4.2
#define BATTERY_MAX_CAPACITY 12.6

class CellVoltage{
private:
	int pin;
	const static float callibration[3];
	const static float optimizedConvert;
	float c;
public:
        CellVoltage(){};
	CellVoltage(int pin);
        void init(int pin);
	float getVoltage();
};
#endif

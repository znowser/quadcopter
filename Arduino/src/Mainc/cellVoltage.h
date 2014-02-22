#ifndef CELLVOLTAGE_H
#define CELLVOLTAGE_H
/* 
 Class that make it possible to read the voltage of lipo-batterys cells.
 The cells are wired to input A0, A1 and A2.
 */
class CellVoltage{
private:
  int pin;
  const static float callibration[3];
  const static float optimizedConvert;
  float c;
public:
  CellVoltage(int pin);
  CellVoltage(){};
  void init(int pin);
  float getVoltage();
};
#endif


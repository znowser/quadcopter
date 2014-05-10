#include "Motor.h"
#include "SensorDataStruct.h"
#include <cmath>

class Hover {
private:
  Motor motors[4];
  sensordata sensor;
  float refHeight;
  unsigned long time;
  float old_lfmh;
  float old_rfmh;
  float old_lfmv;
  float old_rfmv;
  float old_lfma;
  float old_rfma;
  float old_cbh;
  float old_cbv;
  float old_cba;
  float old_errorHeight;
public:
  Hover(Motor motors[4], sensordata &sensor, float refHeight);
  void Regulate(void);
}

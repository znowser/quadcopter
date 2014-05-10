#include "Motor.h"

class Hover {
private:
  Motor motors[4];
  Sensor sensor;
public:
  Hover(Motor motors[4], Sensor sensor);
  void Start(void);
}

#include <iostream>
#include <string.h>
#include "CJoystick.h"

using namespace std;


int main(){

	struct wwvi_js_event status;
	PS3Controller ps3;

	memset(&status, 0, sizeof(wwvi_js_event));

	if (!ps3.open_joystick(JOYSTICK_DEVNAME)) {
		printf("open failed.\n");
		return 0;
	}

	while (true){
		ps3.get_joystick_status(&status);

		cout << "x: " << status.stick1_x / 256<< " y: " << status.stick1_y /256 << endl;
		usleep(100000);
	}
	
	return 0;
}

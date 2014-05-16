#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "CJoystick.h"


bool PS3Controller::open_joystick(const char *joystick_device)
{
  joystick_fd = open(JOYSTICK_DEVNAME, O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
  if (joystick_fd < 0)
    return false;
  return true;
}

int PS3Controller::read_joystick_event(struct js_event *jse)
{
  int bytes;
  bytes = read(joystick_fd, jse, sizeof(*jse)); 
  if (bytes == -1)
    return 0;

  if (bytes == sizeof(*jse))
      return 1;

    printf("Unexpected bytes from joystick:%d\n", bytes);

  return -1;
}

void PS3Controller::close_joystick()
{
  close(joystick_fd);
}

int PS3Controller::get_joystick_status(struct wwvi_js_event *wjse)
{
  int rc;
  struct js_event jse;
  if (joystick_fd < 0)
    return -1;
  // memset(wjse, 0, sizeof(*wjse));
  while ((rc = read_joystick_event(&jse) == 1)) {
    //jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
    if (jse.type == JS_EVENT_AXIS) {
      switch (jse.number) {
      case 0: wjse->stick1_x = jse.value;
	break;
      case 1: wjse->stick1_y = jse.value;
	break;
      case 2: wjse->stick2_x = jse.value;
	break;
      case 3: wjse->stick2_y = jse.value;
	break;
      default:
	break;
      }
    } else if (jse.type == JS_EVENT_BUTTON)
	wjse->button[jse.number] = jse.value;
   }
   return 0;
}

#if 0
/* a little test program */
int main(int argc, char *argv[])
{
  int rc;
  int done = 0;

  struct wwvi_js_event status;
  PS3Controller ps3;
  
  if(!ps3.open_joystick(JOYSTICK_DEVNAME)) {
    printf("open failed.\n");
    exit(1);
  }
  status.stick1_x = 0;
  status.stick1_y = 0;

  //memset
  for(int c = 0; c < NUM_OF_BUTTONS; c++)
    status.button[c] = 0;


  while (!done) {
    //rc = read_joystick_event(&jse);
    rc = ps3.get_joystick_status(&status);
    usleep(1000);
    
    for(int c = 0; c < NUM_OF_BUTTONS; c++){
      printf("%i ", status.button[c]);
    }
    printf("stick1_x: %i, stick1_y: %i", status.stick1_x / 256, status.stick1_y / 256);
    printf("\n");
  }
}
#endif

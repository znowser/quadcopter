/* 
   2    (C) Copyright 2007,2008, Stephen M. Cameron.
   3
   4    This file is part of wordwarvi.
   5
   6    wordwarvi is free software; you can redistribute it and/or modify
   7    it under the terms of the GNU General Public License as published by
   8    the Free Software Foundation; either version 2 of the License, or
   9    (at your option) any later version.
   10
   11    wordwarvi is distributed in the hope that it will be useful,
   12    but WITHOUT ANY WARRANTY; without even the implied warranty of
   13    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   14    GNU General Public License for more details.
   15
   16    You should have received a copy of the GNU General Public License
   17    along with wordwarvi; if not, write to the Free Software
   18    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
   19
   20 */
#include <stdio.h>
#ifndef __CJOYSTICK_H__
#define __CJOYSTICK_H__

#define JOYSTICK_DEVNAME "/dev/input/js0"

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

#define JS_BUTTON_CROSS 14
#define JS_BUTTON_TRIANGLE 12
#define JS_BUTTON_SQUARE 15
#define JS_BUTTON_CIRCLE 13

#define JS_BUTTON_UP 4
#define JS_BUTTON_DOWN 6
#define JS_BUTTON_LEFT 7
#define JS_BUTTON_RIGHT 5

#define JS_BUTTON_R1 11
#define JS_BUTTON_L1 10
#define JS_BUTTON_R2 9
#define JS_BUTTON_L2 8
#define JS_BUTTON_R3 2
#define JS_BUTTON_L3 1
#define JS_BUTTON_SELECT 0
#define JS_BUTTON_START 3
#define JS_BUTTON_PS3 16

#define NUM_OF_BUTTONS 17
#define PS3CONTROLLER_DEADZONE 30

struct js_event {
  unsigned int time;      /* event timestamp in milliseconds */
  short value;   /* value */
  unsigned char type;     /* event type */
  unsigned char number;   /* axis/button number */
};

struct wwvi_js_event {
  int button[NUM_OF_BUTTONS];
  int stick1_x;
  int stick1_y;
  int stick2_x;
  int stick2_y;
};

class PS3Controller{
 private:
  int joystick_fd;
 public:
 PS3Controller():joystick_fd(-1){};
  bool open_joystick(const char *joystick_device);
  int read_joystick_event(struct js_event *jse);
  void set_joystick_y_axis(int axis);
  void set_joystick_x_axis(int axis);
  void close_joystick();
  int get_joystick_status(struct wwvi_js_event *wjse);
};

#endif

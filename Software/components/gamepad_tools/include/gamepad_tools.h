#ifndef GAMEPAD_TOOLS_H
#define GAMEPAD_TOOLS_H

#include <stdbool.h>
#include <stdlib.h>

#define UP_BTN 9
#define DOWN_BTN 11
#define LEFT_BTN 10
#define RIGHT_BTN 46

#define START_BTN 3

extern uint8_t DEST_MAC[6];
extern uint8_t BROADCAST_MAC[6];

typedef struct {
	bool L_up;
	bool L_down;
  	bool L_left;
  	bool L_right;

 	bool R_up;
  	bool R_down;
  	bool R_left;
  	bool R_right;

  	bool start_btn;
  	bool select_btn;

  	uint8_t L_joystick_X;
  	uint8_t L_joystick_Y;
  	bool L_joystick_btn;

  	uint8_t R_joystick_X;
  	uint8_t R_joystick_Y;
  	bool R_joystick_btn;
	
} gamepad_data_t;

void init_gamepad_gpios(void);

#endif

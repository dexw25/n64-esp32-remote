#ifndef N64_API_H
#define N64_API_H

#include <stdbool.h>
#include <stdint.h>

// Config defines
#define N64_GPIO 21

// Definition of data structure to hold n64 controller state
typedef struct {
	bool a;
	bool b;
	bool z;
	bool start;
	bool d_up;
	bool d_down;
	bool d_left;
	bool d_right;
	bool l;
	bool r;
	bool c_up;
	bool c_down;
	bool c_left;
	bool c_right;
	int8_t joy_x;
	int8_t joy_y;
} con_state;

// Macros to ease generation of RMT items
// zero is 3us low, one us high, one is reverse (timing only, not level)
#define ITEM_ONE {{{1, 0, 3, 1}}}
#define ITEM_ZERO {{{3, 0, 1, 1}}}

// Stop bit (console side, controller stop again swaps pulse timings)
#define ITEM_STOP {{{1, 0, 2, 1}}}

// End RMT sequence with zero length pulse
#define ITEM_END {{{ 0, 1, 0, 0}}}

// API functions
// Start this task to commence polling of N64 controller
void con_poll(void*);

#endif
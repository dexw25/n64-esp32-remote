#ifndef N64_API_H
#define N64_API_H

#include <stdbool.h>
#include <stdint.h>

// Config defines
#define N64_GPIO 21

// Definition of data structure to hold n64 controller state (specify bitfields)
typedef struct {
	bool a : 1;
	bool b : 1;
	bool z : 1;
	bool start : 1;
	bool d_up : 1;
	bool d_down : 1;
	bool d_left : 1;
	bool d_right : 1;
	bool reset : 1;
	bool reserved : 1;
	bool l : 1;
	bool r : 1;
	bool c_up : 1;
	bool c_down : 1;
	bool c_left : 1;
	bool c_right : 1;
	int8_t joy_x : 8;
	int8_t joy_y : 8;
} con_state;

typedef struct {
	con_state state;
	int64_t ts;
} state_packet;

typedef enum {
	CON_A,
	CON_B,
	CON_Z,
	CON_START,
	CON_DU,
	CON_DD,
	CON_DL,
	CON_DR,
	CON_RESET,
	CON_RESERVED,
	CON_L,
	CON_R,
	CON_CU,
	CON_CD,
	CON_CL,
	CON_CR
} button_enums;

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
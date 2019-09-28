#ifndef N64_API_H
#define N64_API_H

#include <stdbool.h>
#include <stdint.h>

// Config defines
#define N64_GPIO 21

// Definition of data structure to hold n64 controller state (specify bitfields)
typedef union {
	struct {
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
	};
	uint32_t raw;
	uint8_t bytes[4];
} con_state;

typedef struct {
	uint32_t state;
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

// APB frequency from which to derive RMT filter and decoding params
#define APB_FREQ 80 * 1000 * 1000 

// TX params to ensure 1MHZ period/1us per item
#define TX_TARGET_FREQ 1 * 1000 * 1000
#define TX_DIV (APB_FREQ / (TX_TARGET_FREQ))

// RX params
#define RX_TARGET_FREQ 80 * 1000 * 1000
#define RX_DIV ((APB_FREQ) / (RX_TARGET_FREQ))

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

// Start this to send states out on IR peripheral
void con_send(void*);

#endif
#include "n64_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/rmt.h"
#include "driver/gpio.h"

const static char TAG[] = "CON_POLL";

// 0x01 is poll byte
rmt_item32_t pollcmd[] = {
	ITEM_ZERO, 
	ITEM_ZERO, 
	ITEM_ZERO, 
	ITEM_ZERO, 
	ITEM_ZERO, 
	ITEM_ZERO, 
	ITEM_ZERO, 
	ITEM_ONE, 
	ITEM_STOP,
	ITEM_END
};

// Given one bit, set item to encode 1 or 0
static inline void n64_set_item_val(rmt_item32_t *itemptr, bool val){
	assert(itemptr);
	if (val) {
		itemptr->duration0 = 1;
		itemptr->level0 = 0;
		itemptr->duration1 = 3;
		itemptr->level1 = 1;
	} else {
		itemptr->duration0 = 3;
		itemptr->level0 = 0;
		itemptr->duration1 = 1;
		itemptr->level1 = 1;
	}
}

// Given an item return 1 or 0 
static inline uint8_t n64_get_item(rmt_item32_t *item){
	assert(item);
	return (item->duration0 < 100) ? 1 : 0;
}

// Fill item with stop code
static inline void n64_set_item_stop(rmt_item32_t *itemptr){
	assert(itemptr);
	itemptr->duration0 = 1;
	itemptr->level0 = 0;
	itemptr->duration1 = 2;
	itemptr->level1 = 1;
}

// Fill item with values to signal RMT to end transmission
static inline void rmt_set_item_end(rmt_item32_t *itemptr){
	assert(itemptr);
	itemptr->duration0 = 0;
	itemptr->level0 = 1;
	itemptr->duration1 = 0;
	itemptr->level1 = 0;
}

// Pack a byte buffer into RMT items suitable for sending
static size_t n64_pack_buf(rmt_item32_t *itemptr, size_t max_items, uint8_t *buf, size_t num_bytes){
	assert(itemptr);
	// Save base for assertion below
	rmt_item32_t *baseptr = itemptr;

	// Save items written
	size_t ret = 0;

	// Write nothing if not given enough space
	if ((num_bytes * 8 + 2) <= max_items){
		// For each byte
		for(int i = 0; i < num_bytes; i++) {
			// For bit in byte (MSB first)
			for (int j = 7; j >= 0; j--){
				// One item per bit
				n64_set_item_val(itemptr++, (buf[i] & (1 << j)) ? true : false);
				ret++;
			}
		}

		// Fill stop bit
		n64_set_item_stop(itemptr++);
		ret++;

		// RMT end code
		rmt_set_item_end(itemptr++);
		ret++;

		// Check code above, 8 bits per byte and 2 stop/end items should have been written
		assert(itemptr - baseptr == num_bytes * 8 + 2);
	}
	return ret;
}

// Parse a controller status struct, 32 bits (any extras will be ignored)
bool n64_parse_resp(rmt_item32_t *item, size_t num_items, con_state *con){
	bool ret = false;
	rmt_item32_t *first_item = item;
	int8_t joy_new;
	bool on;

	// Do not update state if we don't have enough packets. This also blocks buffer overflow
	if (num_items < 32){
		return false;
	}

	// parse digital inputs, flag state change if it happened
	//  Enum maps 0-15 to digital states
	for (int i = 0; i < 16; i++){
		on = n64_get_item(item++);
		switch (i) {
				case CON_A:
					ret |= on == con->a ? 0 : 1;
					con->a = on;
					break;
				case CON_B:
					ret |= on == con->b ? 0 : 1;
					con->b = on;
					break;
				case CON_Z:
					ret |= on == con->z ? 0 : 1;
					con->z = on;
					break;
				case CON_START:
					ret |= on == con->start ? 0 : 1;
					con->start = on;
					break;
				case CON_DU:
					ret |= on == con->d_up ? 0 : 1;
					con->d_up = on;
					break;
				case CON_DD:
					ret |= on == con->d_down ? 0 : 1;
					con->d_down = on;
					break;
				case CON_DL:
					ret |= on == con->d_left ? 0 : 1;
					con ->d_left = on;
					break;
				case CON_DR:
					ret |= on == con->d_right ? 0 : 1;
					con->d_right = on;
					break;
				case CON_RESET:
					ret |= on == con->reset ? 0 : 1;
					con->reset = on;
					break;
				case CON_RESERVED:
					ret |= on == con->reserved ? 0 : 1;
					con->reserved = on;
					break;
				case CON_L:
					ret |= on == con->l ? 0 : 1;
					con->l = on;
					break;
				case CON_R:
					ret |= on == con->r ? 0 : 1;
					con->r = on;
					break;
				case CON_CU:
					ret |= on == con->c_up ? 0 : 1;
					con->c_up = on;
					break;
				case CON_CD:
					ret |= on == con->c_down ? 0 : 1;
					con->c_down = on; 
					break;
				case CON_CL:
					ret |= on == con->c_left ? 0 : 1;
					con->c_left = on;
					break;
				case CON_CR:
					ret |= on == con->c_right ? 0 : 1;
					con->c_right = on;
					break;
				default:
					break;
		}
	}

	// Parse analog axes as 8 bit signed ints, MSB first
	joy_new = 0;
	for (int i=7; i >= 0; i--) {
		joy_new |= n64_get_item(item++) << i;
	}

	if (joy_new != con->joy_x) {
		ret = true;
		con->joy_x = joy_new;
	}

	joy_new = 0;
	for (int i=7; i >= 0; i--) {
		joy_new |= n64_get_item(item++) << i;
	}

	if (joy_new != con->joy_y) {
		ret = true;
		con->joy_y = joy_new;
	}

	// Assert no error in the above code, only 32 bits read in, no more or less
	assert((item - first_item) == 32);

	return ret;
}

int parse_cmd(rmt_item32_t *item, size_t num_items){
	rmt_item32_t *first_item = item;
	if (num_items < 8){
		// Malformed command, return error
		return -1;
	}
	// Parse 8 bit unsigned int, MSB first
	uint8_t ret = 0;
	for (int i = 7; i >= 0; i--){
		ret |= n64_get_item(item++) << i;
	}

	// Assert we did not access more than 8 items
	assert((item - first_item) <= 8);

	return ret;
}

rmt_item32_t *reply_to_cmd(int cmd, size_t *num_items, con_state *state){
	rmt_item32_t *ret = NULL;
	switch (cmd){
		case 0x00:
		// Status cmd, reply 0x05, 0x00, (0x01 for controller pack, 0x02 for no pack, 0x04 if CRC error)
		// 24 bits (3x8) of data plus stop bit and RMT end
			ret = malloc(sizeof(rmt_item32_t) * 26);
			if (ret) {
				uint8_t msg[] = {
					0x05,
					0x00,
					0x01
				};

				// No assertion here, add one if bugs pop up
				n64_pack_buf(ret, 26, msg, sizeof msg);
			}
			break;
		case 0x01:
		// Poll cmd, send controller state
			ret = malloc(sizeof(rmt_item32_t) * 34);
			if (ret) {
				// No assertion here, add one if bugs pop up
				n64_pack_buf(ret, 34, state->bytes, sizeof state->bytes);
			}
			break;
		default:
		// Do not set ret
			break;
	}
	return ret;
}

void con_poll(void *pvParameters){
	RingbufHandle_t rx_ring;
	rmt_item32_t *rx_item;
	size_t rx_item_size;
	QueueHandle_t mbuf = pvParameters;
	assert(mbuf);
	state_packet st;

	// Static controller state var
	static con_state state;
	static bool new_state;

	// Timing params for poll loop
	TickType_t now_ticks, last_wake_time = xTaskGetTickCount();
	// 60hz (tick rate must be a multiple of 60, 120 recommended)
	const TickType_t poll_period = (1000/60) / portTICK_PERIOD_MS;

	ESP_LOGI(TAG, "Starting controller driver init");

	//Init RMT TX channel
	rmt_config_t txconf, rxconf;
	txconf.rmt_mode = RMT_MODE_TX;
	txconf.gpio_num = N64_GPIO;
	txconf.mem_block_num = 1;
	txconf.clk_div = TX_DIV; // APB is 80Mhz so this makes for a 1us period
	txconf.channel = 0;

	txconf.tx_config.loop_en = 0;
	txconf.tx_config.carrier_en = 0;
	txconf.tx_config.idle_output_en = 0;
	txconf.tx_config.idle_level = 1;

	//Init RMT rx channel (simultaneous operation possible on same pin?)
	rxconf.rmt_mode = RMT_MODE_RX;
	// rxconf.gpio_num = N64_GPIO;
	rxconf.gpio_num = 14;
	rxconf.mem_block_num = 1;
	rxconf.clk_div = 1; // APB is 80Mhz, don't divide on rx to get better resolution
	rxconf.channel = 1;

	rxconf.rx_config.filter_en = 1;
	rxconf.rx_config.filter_ticks_thresh = 60;
	rxconf.rx_config.idle_threshold = 800;

	ESP_LOGD(TAG, "TX init");
	ESP_ERROR_CHECK(rmt_config(&txconf));
	ESP_ERROR_CHECK(rmt_driver_install(txconf.channel, 0, 0));

	ESP_LOGD(TAG, "RX init");
	ESP_ERROR_CHECK(rmt_config(&rxconf));
	ESP_ERROR_CHECK(rmt_driver_install(rxconf.channel, 1024, 0));
	ESP_ERROR_CHECK(rmt_get_ringbuf_handle(rxconf.channel, &rx_ring));

	// Start RX, never stops as long as this task is running
	ESP_ERROR_CHECK(rmt_rx_start(rxconf.channel, true));

	while(1){
		// Send pollcmd, no delay needed for TX finish
		rmt_write_items(txconf.channel, pollcmd, sizeof pollcmd / sizeof pollcmd[0], false);

		// Controller response will be concatenated with our command if present
		//  Wait a tick to allow bytes to come in, if not then this frame of state
		//  +is delayed to the next poll and this is not desirable
		//  +Desired period is about 16 ms, one tick is 10 ms. This sicks but is better than 16. 
		//  Actual delay is much less because rmt_rx wakes us up
		// This is fine, latency with a response packet is 212 us (so small, so fast)
		rx_item = xRingbufferReceive(rx_ring, &rx_item_size, 1);

		// If no items rx'ed do not attempt to parse anything
		if (rx_item){
			// Check enough items were rx'ed (32 for state, 9 for our cmd and 1 stop bit)
			if (rx_item_size < 42){
				// Not an error condition, this means controller disconnected. turn off activity light
		        gpio_set_level(13, 0);
			} else {
				new_state = n64_parse_resp(rx_item + 9, (rx_item_size / sizeof *rx_item) - 9, &state);
				st.state = state.raw;
				st.ts = esp_timer_get_time();
		        gpio_set_level(13, 1);
			}
			vRingbufferReturnItem(rx_ring, rx_item);
		} else {
			ESP_LOGE(TAG, "Protocol error");
	        gpio_set_level(13, 0);
		}

		// Don't wait just drop the packet here if theres no room
		// But only put it in the queue if something is new
		if (new_state){
			new_state = false;
			xQueueOverwrite(mbuf, &st);
		}

		// Check timing against last wake time, and error if we overran the period
		now_ticks = xTaskGetTickCount();
		if ((now_ticks - last_wake_time) > poll_period){
			// Overran our schedule, update last_wake_time to now_ticks, otherwise the next poll happens immediately and the controller may not like that
			last_wake_time = now_ticks;
		}

		// Ensure 60hz(ish) period
		vTaskDelayUntil(&last_wake_time, poll_period);
	}
	// send state (maybe just state changes?) to MQTT(?) endpoint (or print to serial debug for now)
}

void con_send(void *params) {
	QueueHandle_t mbuf = params;
	state_packet pkt;
	RingbufHandle_t rx_ring;
	uint8_t cmd;
	assert(mbuf);
	rmt_item32_t *rx_item, *tx_item;
	size_t tx_item_len, rx_item_size;
	size_t resp_bits_sent = 0;
	con_state state;

	//Init RMT TX channel
	rmt_config_t txconf, rxconf;
	txconf.rmt_mode = RMT_MODE_TX;
	txconf.gpio_num = 15;
	txconf.mem_block_num = 1;
	txconf.clk_div = TX_DIV;
	txconf.channel = 3;

	txconf.tx_config.loop_en = 0;
	txconf.tx_config.carrier_en = 0;
	txconf.tx_config.idle_output_en = 0;
	txconf.tx_config.idle_level = 1;

	//Init RMT rx channel (simultaneous operation possible on same pin?)
	rxconf.rmt_mode = RMT_MODE_RX;
	rxconf.gpio_num = 16;
	rxconf.mem_block_num = 1;
	rxconf.clk_div = RX_DIV; // APB is 80Mhz, don't divide on rx to get more resolution
	rxconf.channel = 2;

	// Filter params drive parsing code above, see macro definitions
	rxconf.rx_config.filter_en = 1;
	rxconf.rx_config.filter_ticks_thresh = 60 ;
	rxconf.rx_config.idle_threshold = 800;

	ESP_LOGD(TAG, "TX init");
	ESP_ERROR_CHECK(rmt_config(&txconf));
	ESP_ERROR_CHECK(rmt_driver_install(txconf.channel, 0, 0));

	ESP_LOGD(TAG, "RX init");
	ESP_ERROR_CHECK(rmt_config(&rxconf));

	ESP_ERROR_CHECK(rmt_driver_install(rxconf.channel, 1024, 0));

	ESP_ERROR_CHECK(rmt_get_ringbuf_handle(rxconf.channel, &rx_ring));

	// Start RX, never stops as long as this task is running
	ESP_ERROR_CHECK(rmt_rx_start(rxconf.channel, true));

	// suspend until first packet is in
	xQueueReceive(mbuf, &pkt, portMAX_DELAY);
	state.raw = pkt.state;

	while(1){
		// Update state if update is available
		xQueueReceive(mbuf, &pkt, 0);

		// Receive console cmd, suspend until there is something to respond to
		rx_item = xRingbufferReceive(rx_ring, &rx_item_size, portMAX_DELAY);

		if (rx_item) {
			// Decode command byte
			cmd = parse_cmd(rx_item + resp_bits_sent, (rx_item_size / sizeof *rx_item) - resp_bits_sent);
			vRingbufferReturnItem(rx_ring, rx_item);

			// reply_to_cmd will build a reply and return an item list (dynamically allocated)
			tx_item = reply_to_cmd(cmd, &tx_item_len, &state);

			// tx_item will be null if no response is expected or if there was a memory error
			if (tx_item) {
				// Do not RX our response
				rmt_rx_stop(rxconf.channel);

				// Must wait to avoid use-after-free error
				rmt_write_items(txconf.channel, pollcmd, sizeof pollcmd / sizeof pollcmd[0], true);
				free(tx_item);

				rmt_rx_start(rxconf.channel, true);
			}
		}
	}
}

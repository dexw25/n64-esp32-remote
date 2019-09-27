#include "n64_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/gpio.h"

const char TAG[] = "CON_POLL";

// 0x00 is poll byte
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

// Static controller state var
static con_state state;

// Parse up to 42 items
void n64_parse_resp(rmt_item32_t *item, size_t num_items, con_state *con){
	rmt_item32_t *first_item = item;

	// Do not update state if we don't have enough packets. This also blocks buffer overflow
	if (num_items < 42){
		return;
	}

	for (int i=0; i < 9; i++) item++; // seek cmd and cmd stop bit

	// parse digital inputs
	for (int i = 0; i < 16; i++){
		bool on = (item++)->duration0 < 100 ? true : false;
		switch (i) {
				case CON_A:
					con->a = on;
					break;
				case CON_B:
					con->b = on;
					break;
				case CON_Z:
					con->z = on;
					break;
				case CON_START:
					con->start = on;
					break;
				case CON_DU:
					con->d_up = on;
					break;
				case CON_DD:
					con->d_down = on;
					break;
				case CON_DL:
					con ->d_left = on;
					break;
				case CON_DR:
					con->d_right = on;
					break;
				case CON_RESET:
					con->reset = on;
					break;
				case CON_RESERVED:
					con->reserved = on;
					break;
				case CON_L:
					con->l = on;
					break;
				case CON_R:
					con->r = on;
					break;
				case CON_CU:
					con->c_up = on;
					break;
				case CON_CD:
					con->c_down = on; 
					break;
				case CON_CL:
					con->c_left = on;
					break;
				case CON_CR:
					con->c_right = on;
					break;
				default:
					break;
		}
	}

	// Parse analog axes as 8 bit signed ints, MSB first
	con->joy_x = 0;
	for (int i=7; i >= 0; i--) {
		if (item->duration0 < 100) con->joy_x |= 1 << i;
		item++;
	}
	con->joy_y = 0;
	for (int i=7; i >= 0; i--) {
		if (item->duration0 < 100) con->joy_y |= 1 << i;
		item++;
	}

	// Assert no buffer overflow condition occurred
	assert((item - first_item) <= num_items);

}

void con_poll(void *pvParameters){
	RingbufHandle_t rx_ring;
	rmt_item32_t *rx_item;
	size_t rx_item_size;

	// Timing params for poll loop
	TickType_t now, last_wake_time = xTaskGetTickCount();
	// 60hz-ish
	const TickType_t poll_period = (1000/60) / portTICK_PERIOD_MS;

	ESP_LOGI(TAG, "Starting controller driver init");

	//Init RMT TX channel
	rmt_config_t txconf, rxconf;
	txconf.rmt_mode = RMT_MODE_TX;
	txconf.gpio_num = N64_GPIO;
	txconf.mem_block_num = 1;
	txconf.clk_div = 80; // APB is 80Mhz so this makes for a 1us period
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
	rxconf.clk_div = 1; // APB is 80Mhz, don't divide on rx to get more resolution
	rxconf.channel = 1;

	rxconf.rx_config.filter_en = 1;
	rxconf.rx_config.filter_ticks_thresh = 60;
	rxconf.rx_config.idle_threshold = 800;

	ESP_LOGI(TAG, "TX init");
	ESP_ERROR_CHECK(rmt_config(&txconf));
	ESP_ERROR_CHECK(rmt_driver_install(txconf.channel, 0, 0));

	ESP_LOGI(TAG, "RX init");
	ESP_ERROR_CHECK(rmt_config(&rxconf));

	ESP_ERROR_CHECK(rmt_driver_install(rxconf.channel, 1024, 0));

	ESP_ERROR_CHECK(rmt_get_ringbuf_handle(rxconf.channel, &rx_ring));

	// Start RX, never stops as long as this task is running
	ESP_ERROR_CHECK(rmt_rx_start(rxconf.channel, true));

	while(1){
		// Send pollcmd, no delay needed for TX finish
		rmt_write_items(txconf.channel, pollcmd, sizeof pollcmd / sizeof pollcmd[0], false);

		// Controller response will be concatenated with our command if present
		//  No delay here, write_items starts TX fast enough that the ring buffer is locked by rx before this call
		//  
		rx_item = xRingbufferReceive(rx_ring, &rx_item_size, 0);

		// If no items rx'ed do not attempt to parse anything
		if (rx_item){
			// Check enough items were rx'ed
			if (rx_item_size < 42){
				ESP_LOGE(TAG, "got %d items, expected at least 42", rx_item_size / sizeof *rx_item);
			} else {
				n64_parse_resp(rx_item, rx_item_size / sizeof *rx_item, &state);
			}
			vRingbufferReturnItem(rx_ring, rx_item);
		}

		// Check timing against last wake time, and error if we overran the period
		now = xTaskGetTickCount();
		if ((now - last_wake_time) > poll_period){
			// Overran our schedule, update last_wake_time to now, otherwise the next poll happens immediately and the controller may not like that
			last_wake_time = now;
		}

		// Ensure 60hz(ish) period
		vTaskDelayUntil(&last_wake_time, poll_period);
	}
	// send state (maybe just state changes?) to MQTT(?) endpoint (or print to serial debug for now)
}
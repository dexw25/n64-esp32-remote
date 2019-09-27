#include "n64_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"

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
uint32_t n64_parse_resp(rmt_item32_t *item, size_t num_items, con_state *con){
	uint32_t ret = 0;

	if (num_items < 42){
		return 0;
	}

	for (int i=0; i < 9; i++) item++; // skip cmd

	// parse digital inputs
	for (int i = 0; i < 16; i++){
		bool on = (item++)->duration0 < 100 ? true : false;
		switch (i) {
				case CON_A:
					if (on != con->a) {
						con->a = on;
						ESP_LOGI(TAG, "a state changed");
					} 
					break;
				case CON_B:
					if (on != con->b) {
						con->b = on;
						ESP_LOGI(TAG, "b state changed");
					} 
					break;
				case CON_Z:
					if (on != state.z) {
						con->z = on;
						ESP_LOGI(TAG, "z state changed");
					} 
					break;
				case CON_START:
					if (on != state.start) {
						con->start = on;
						ESP_LOGI(TAG, "start state changed");
						if (con->start){
							gpio_set_level(13, 1);
						} else {
							gpio_set_level(13, 0);
						}
					} 
					break;
				case CON_DU:
					if (on != state.d_up) {
						con->d_up = on;
						ESP_LOGI(TAG, "d_up state changed");
					} 
					break;
				case CON_DD:
					if (on != state.d_down) {
						con->d_down = on;
						ESP_LOGI(TAG, "d_down state changed");
					} 
					break;
				case CON_DL:
					if (on != state.d_left) {
						con->d_left = on;
						ESP_LOGI(TAG, "d_left state changed");
					} 
					break;
				case CON_DR:
					if (on != state.d_left) {
						con->d_left = on;
						ESP_LOGI(TAG, "d_left state changed");
					} 
					break;
				case CON_RES0:
					break;
				case CON_RES1:
					break;
				case CON_L:
					if (on != state.l) {
						con->l = on;
						ESP_LOGI(TAG, "l state changed");
					} 
					break;
				case CON_R:
					if (on != state.r) {
						con->r = on;
						ESP_LOGI(TAG, "r state changed");
					} 
					break;
				case CON_CU:
					if (on != state.c_up) {
						con->c_up = on;
						ESP_LOGI(TAG, "c_up state changed");
					} 
					break;
				case CON_CD:
					if (on != state.c_down) {
						con->c_down = on;
						ESP_LOGI(TAG, "c_down state changed");
					} 
					break;
				case CON_CL:
					if (on != state.c_left) {
						con->c_left = on;
						ESP_LOGI(TAG, "c_left state changed");
					} 
					break;
				case CON_CR:
					if (on != state.c_right) {
						con->c_right = on;
						ESP_LOGI(TAG, "c_right state changed");
					} 
					break;
				default:
					break;
		}

		// TODO: Handle joystick axes
	}
	return ret;
}

void con_poll(void *pvParameters){
	RingbufHandle_t rx_ring;
	rmt_item32_t *rx_item;
	size_t rx_item_size;

	// Timing params for poll loop
	TickType_t last_wake_time = xTaskGetTickCount();
	// 60hz-ish
	// const TickType_t poll_period = (1000/60) / portTICK_PERIOD_MS;
	const TickType_t poll_period = (1000/15) / portTICK_PERIOD_MS; //DELETEME: slow this down to 15hz

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

	//Clear controller state

	//while(60 times per second)
	while(1){
		// TODO: Check if we overrun 60hz period and throw exception if so

		// Rx first. We will get the tx in the buffer but this is fine
		ESP_ERROR_CHECK(rmt_rx_start(rxconf.channel, true));

		// Send pollcmd and wait for finish
		rmt_write_items(txconf.channel, pollcmd, sizeof pollcmd / sizeof pollcmd[0], true);

		// Wait a couple milliseconds for rx
		// vTaskDelay(1 / portTICK_PERIOD_MS);

		rx_item = xRingbufferReceive(rx_ring, &rx_item_size, pdMS_TO_TICKS(1));

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

		// Ensure 60hz(ish) period
		vTaskDelayUntil(&last_wake_time, poll_period);
	}
	// Send 0x00

	// RX controller state

	// send state (maybe just state changes?) to MQTT(?) endpoint (or print to serial debug for now)
	//end while
}
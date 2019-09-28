/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "n64_api.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

const static char TAG[] = "MAIN";

void app_main(void)
{
    QueueHandle_t sbuf;
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Create queue to pass state structs, always holds the last update
    sbuf = xQueueCreate(1, sizeof (state_packet));

    // start controller poll task
    xTaskCreate(
        con_poll,
        "controller_poll",
        4096,
        sbuf, // no params
        1,
        NULL); // Do not save task handle//*/

    // TEST: see controller values and metadata
        state_packet pkt;
        int64_t then=0, now;
        while(1){
            xQueueReceive(sbuf, &pkt, portMAX_DELAY);
            now = esp_timer_get_time();
            ESP_LOGI(TAG, "Time since last packet: %lldus, Time since this packet: %lldus", pkt.ts - then, now - pkt.ts);
            then = pkt.ts;
        }

    // Suspend main task
    while(1){
      vTaskDelay(portMAX_DELAY);
    }
}

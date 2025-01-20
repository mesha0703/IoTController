#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "espnow_handler.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "gamepad_tools.h"
#include "portmacro.h"

#define TAG "gamepad_tools"

#define DEBOUNCE_TIME_MS 50 // Debounce time in milliseconds
#define ESP_INTR_FLAG_DEFAULT 0

//uint8_t DEST_MAC[6] = { 0xC8, 0xC9, 0xA3, 0xD0, 0xE9, 0xE8 };
uint8_t DEST_MAC[6] = { 0xC8, 0xC9, 0xA3, 0xD0, 0xE9, 0xE8 };
uint8_t BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static QueueHandle_t gpio_evt_queue = NULL;
gamepad_data_t gamepad_data = { 0 };
static int64_t last_event_time = 0;      // Timestamp of the last valid button event

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void send_gamepad_data() {
	esp_err_t result = esp_now_send(DEST_MAC, (uint8_t *) &gamepad_data, sizeof(gamepad_data));
	if (result == ESP_OK) {
        ESP_LOGI(TAG, "ESP-NOW message sent successfully");
    } else {
        ESP_LOGE(TAG, "Failed to send ESP-NOW message");
    }
}

// Task to handle GPIO events
static void gpio_task(void* arg) {
    uint32_t gpio_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY) == pdTRUE) {
			gamepad_data_t new_gamepad_data = {
				.L_up = gpio_get_level(UP_BTN),
				.L_down = gpio_get_level(DOWN_BTN),
				.L_left = gpio_get_level(LEFT_BTN),
				.L_right = gpio_get_level(RIGHT_BTN),
				.start_btn = gpio_get_level(START_BTN),
			};
			
			// check if the data has changed. If not, skip sending.
			if (memcmp(&gamepad_data, &new_gamepad_data, sizeof(gamepad_data_t)) == 0) {
				continue;
			}
			
			// wait for debouncing time before sending new data

			int64_t now = esp_timer_get_time() / 1000;   // Current time in milliseconds

			if (now - last_event_time >= DEBOUNCE_TIME_MS) {
				gamepad_data.L_up = new_gamepad_data.L_up;
				gamepad_data.L_down = new_gamepad_data.L_down;
				gamepad_data.L_left = new_gamepad_data.L_left;
				gamepad_data.L_right = new_gamepad_data.L_right;
				gamepad_data.start_btn = new_gamepad_data.start_btn;
				
				send_gamepad_data(); // Send the GPIO state over ESP-NOW
				
				last_event_time = now;
			}
        }
    }
}

/* before calling this function, you have to firstly init wifi and esp_now */
/* GPIOs are set in a way, that an interrupt function is called whenever status of the BTNs change */
void init_gamepad_gpios(void) {
	ESP_LOGI(TAG, "Initializing GPIOs...");
	
	// Zero-initialize the config structure.
    gpio_config_t io_conf = {};

    // Interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    // Bit mask of the pins
    io_conf.pin_bit_mask =	(1ULL<<UP_BTN) | (1ULL<<DOWN_BTN) | (1ULL<<LEFT_BTN) | (1ULL<<RIGHT_BTN) | \
    						(1ULL<<START_BTN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);
    
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 10, NULL);
	
	//install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(UP_BTN, gpio_isr_handler, (void*) UP_BTN);
    gpio_isr_handler_add(DOWN_BTN, gpio_isr_handler, (void*) DOWN_BTN);
    gpio_isr_handler_add(LEFT_BTN, gpio_isr_handler, (void*) LEFT_BTN);	
    gpio_isr_handler_add(RIGHT_BTN, gpio_isr_handler, (void*) RIGHT_BTN);
    gpio_isr_handler_add(START_BTN, gpio_isr_handler, (void*) START_BTN);
}
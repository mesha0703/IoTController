#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "rom/ets_sys.h"

#define UART_NUM UART_NUM_1
#define TX_PIN 5
#define RX_PIN 4
#define INTERRUPT_PIN 16
#define BUF_SIZE 1024
#define TEST_MESSAGE "PING" // Test message to check connection
#define PULSE_DURATION_MS 100

static const char *TAG = "UART_TEST";

// Interrupt handler function
void IRAM_ATTR send_pulse_isr(void *arg) {
    // Send a pulse on the interrupt pin
    gpio_set_level(INTERRUPT_PIN, 1);
    ets_delay_us(PULSE_DURATION_MS * 1000); // Pulse duration in microseconds
    gpio_set_level(INTERRUPT_PIN, 0);
}

void uart_check_task(void *arg) {
    uint8_t data[BUF_SIZE];
    while (1) {
        // Send a test message
        uart_write_bytes(UART_NUM, TEST_MESSAGE, sizeof(TEST_MESSAGE));

        // Delay for the response to arrive
        vTaskDelay(pdMS_TO_TICKS(100));

        // Check for response
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(50));
        if (len > 0) {
            ESP_LOGI(TAG, "Device responded: %.*s", len, data);
            // Trigger the interrupt
            send_pulse_isr(NULL);
        } else {
            ESP_LOGW(TAG, "No response from the device");
        }

        // Wait before the next check
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t init_uart_over_pogo(void) {
	esp_err_t err;
	
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    err = uart_param_config(UART_NUM, &uart_config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    err = uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
	if (err != ESP_OK) return err;
	
    // Configure the interrupt pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INTERRUPT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    // Create the UART check task
    xTaskCreate(uart_check_task, "uart_check_task", 2048, NULL, 10, NULL);

	// Delete later
	gpio_set_level(INTERRUPT_PIN, 1);

	return ESP_OK;
}

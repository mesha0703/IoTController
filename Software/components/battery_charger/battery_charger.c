#include <stdio.h>
#include "esp_check.h"
#include "driver/gpio.h"
#include "battery_charger.h"
#include "hal/gpio_types.h"

//#define EN1_GPIO ?
//#define EN2_GPIO ?
#define SYSOFF_GPIO 6
#define STANDBY_GPIO 17

#define TAG "battery_charger"

esp_err_t init_batteryCharger(void) {
	esp_err_t err;
	
	// Configure GPIOs
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask =	(1ULL<<SYSOFF_GPIO) | (1ULL<<STANDBY_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    
    err = gpio_set_level(SYSOFF_GPIO, 0);
    if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error while setting level of SYSOFF_GPIO: (%s)", esp_err_to_name(err));
		return err;
    }
    
    err = gpio_set_level(STANDBY_GPIO, 0);
    if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error while setting level of STANDBY_GPIO: (%s)", esp_err_to_name(err));
		return err;
    }
	
	return ESP_OK;
}

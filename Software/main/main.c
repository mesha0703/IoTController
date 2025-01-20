#include <stdio.h>
#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"
#include "espnow_handler.h"
#include "gamepad_tools.h"
#include "battery_fuelGauge.h"
#include "battery_charger.h"
#include "uart_over_pogo.h"

#define TAG "MAIN"

bool is_fuelGauge_init = false;
TaskHandle_t getData_fuelGauge_taskHandle = NULL;
BaseType_t getData_fuelGauge_taskRet;
TaskHandle_t updateNVSParam_fuelGauge_taskHandle = NULL;
BaseType_t updateNVSParam_fuelGauge_taskRet;

void app_main(void)
{	
	esp_err_t err_check;
	
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	ESP_LOGI(TAG, "Starting IoT-Controller initialization.");
	
	// Initialize NVS
	ESP_LOGI(TAG, "Initializing NVS...");
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	    ESP_ERROR_CHECK( nvs_flash_erase() );
	    ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );
	ESP_LOGI(TAG, "Done.");
	
	ESP_LOGI(TAG, "Initializing battery charger...");
	ESP_ERROR_CHECK( init_batteryCharger() );
	ESP_LOGI(TAG, "Done.");
	
	// Initialize ESP-NOW
	wifi_init();
	espnow_init();
	
	// Initialize gamepad GPIO interrupts
	init_gamepad_gpios();
	
	/*
	// Init Fuel Gauge and create tasks if init was successful.
	err_check = init_fuelGauge();
	if (err_check == ESP_OK) {
		ESP_ERROR_CHECK_WITHOUT_ABORT( loadNVSParamAfterPOR_fuelGauge() );
		
		getData_fuelGauge_taskRet = xTaskCreate(getData_fuelGauge, "getData_fuelGauge", 4096,\
												NULL, 6, &getData_fuelGauge_taskHandle);
		if (getData_fuelGauge_taskRet != pdPASS) {
    		ESP_LOGE(TAG, "Task creation for getData_fuelGauge failed!");
		}
		
		updateNVSParam_fuelGauge_taskRet = xTaskCreate(updateNVSParam_fuelGauge, "updateNVSParam_fuelGauge", 4096,\
														NULL, 7, &updateNVSParam_fuelGauge_taskHandle);
		if (updateNVSParam_fuelGauge_taskRet != pdPASS) {
    		ESP_LOGE(TAG, "Task creation failed!");
		}

		is_fuelGauge_init = true;
	}
	else ESP_LOGE(TAG, "Failed initializing fuel gauge chip. Error: (%s)", esp_err_to_name(err_check));
	*/
	
	/*
	// Init UART over POGO connector
	err_check = init_uart_over_pogo();
	if (err_check != ESP_OK) ESP_LOGE(TAG, "Failed initializing UART over Pogo. Error: (%s)", esp_err_to_name(err_check));
	*/
	
	while(true){
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
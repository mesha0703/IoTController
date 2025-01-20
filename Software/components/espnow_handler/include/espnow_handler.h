#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include <stdlib.h>
#include <stdbool.h>
#include "esp_check.h"

void wifi_init(void);
void espnow_init(void);
esp_err_t espnow_add_peer(uint8_t *mac);

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#define CONFIG_ESPNOW_CHANNEL 1

#endif
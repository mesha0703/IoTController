#ifndef BATTERY_FUELGAUGE_H
#define BATTERY_FUELGAUGE_H

#include "esp_check.h"

esp_err_t init_fuelGauge(void);
void getData_fuelGauge(void *pvParameters);
void updateNVSParam_fuelGauge(void *pvParameters);
esp_err_t readNVSParam_fuelGauge(void);
esp_err_t loadNVSParamAfterPOR_fuelGauge(void);

#endif

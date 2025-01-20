idf_component_register(SRCS "battery_fuelGauge.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c nvs_flash)


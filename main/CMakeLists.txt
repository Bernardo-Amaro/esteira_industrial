idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver freertos esp_driver_gpio esp_timer)
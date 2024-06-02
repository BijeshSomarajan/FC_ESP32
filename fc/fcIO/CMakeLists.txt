FILE(GLOB SRC_FILES spi/*.c pwm/*.c wifi/*c)
idf_component_register( SRCS ${SRC_FILES}
              		    INCLUDE_DIRS "include"
              		    REQUIRES "fcLogger" "driver" "fcUtil" "esp_wifi" "nvs_flash" "esp_http_server"
                       )




FILE(GLOB SRC_FILES *.c)
idf_component_register( SRCS ${SRC_FILES}
                        INCLUDE_DIRS "include"
                         REQUIRES "esp_timer" "freertos"  
                      )




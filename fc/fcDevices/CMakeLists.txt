FILE(GLOB SRC_FILES mpu9250/*.c lsm9ds1/*.c fsia/*.c flash/*c bmp388/*c indicator/*c)
idf_component_register( SRCS ${SRC_FILES}
              		    INCLUDE_DIRS "include"
              		    REQUIRES "fcLogger" "fcTimer" "fcIO" "fcUART" "nvs_flash" "fcDSP"
                      )




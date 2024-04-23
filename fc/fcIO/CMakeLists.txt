FILE(GLOB SRC_FILES spi/*.c pwm/*.c)
idf_component_register( SRCS ${SRC_FILES}
              		    INCLUDE_DIRS "include"
              		    REQUIRES "fcLogger" "driver" "fcUtil"
                       )




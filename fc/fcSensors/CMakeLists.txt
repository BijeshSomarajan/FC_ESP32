FILE(GLOB SRC_FILES attitude/*.c rc/*.c pwm/*.c altitude/*.c config/*.c indicator/*.c noise/*.c )
idf_component_register( SRCS ${SRC_FILES}
              		    INCLUDE_DIRS "include"
              		    REQUIRES "fcLogger" "fcTimer" "fcDevices" "fcCalibration" "fcUtil" "fcDSP" "fcCommon"
                      )
FILE(GLOB SRC_FILES *.c attitudeManager/*.c configManager/*.c rcManager/*.c fcStatusManager/*.c pwmManager/*.c debugManager/*.c altitudeManager/*.c configManager/*.c)
idf_component_register(  SRCS ${SRC_FILES}
                   		 INCLUDE_DIRS "include" 
                   		 REQUIRES "fcTimer" "fcIMU" "fcSensors" "fcCalibration" "fcUtil" "esp_timer" "fcLogger" "fcControl" "fcCommon"
                  	   )



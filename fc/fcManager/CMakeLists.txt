FILE(GLOB SRC_FILES *.c attitudeManager/*.c configManager/*.c rcManager/rcManager.c rcManager/rxRcManager.c rcManager/wifiRcManager.c fcStatusManager/*.c pwmManager/*.c debugManager/*.c altitudeManager/*.c vibrationManager/*.c )
idf_component_register(  SRCS ${SRC_FILES}
                   		 INCLUDE_DIRS "include" 
                   		 REQUIRES "fcTimer" "fcIMU" "fcSensors" "fcCalibration" "fcUtil" "esp_timer" "fcLogger" "fcControl" "fcCommon" "fcDSP"
                  	   )



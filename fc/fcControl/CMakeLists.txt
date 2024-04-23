FILE(GLOB SRC_FILES pid/*.c attitude/*.c common/*.c altitude/*.c)
idf_component_register( SRCS ${SRC_FILES}
              		    INCLUDE_DIRS "include"
              		    REQUIRES "fcLogger" "fcDSP" "fcSensors" "fcIMU" "fcCommon"
                      )




FILE(GLOB SRC_FILES *.c fcStatus/*.c)
idf_component_register(  SRCS ${SRC_FILES}
                   		 INCLUDE_DIRS "include" 
                   		 REQUIRES "fcLogger"
                  	   )



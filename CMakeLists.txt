# ACTUATORS
set( srcs "actuators.c" )
set( include_dirs "." )
set( requires hal_esp32 rtos-support statistics )
set( priv_requires common endpoints pca9555 printfx rules syslog systiming )
idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires} PRIV_REQUIRES ${priv_requires})

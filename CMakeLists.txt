# ACTUATORS

set( srcs "actuators.c" )
set( include_dirs "." )
set( priv_include_dirs )
set( requires rtos-support statistics )
set( priv_requires common driver endpoints hal_esp32 pca9555 printfx rules syslog systiming )

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)

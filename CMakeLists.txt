# ACTUATORS

set( srcs "actuators.c" )
set( include_dirs "." )
set( priv_include_dirs )
set( requires )
set( priv_requires
	pca9555 pcf8574 common main printfx syslog systiming
	endpoints hal_esp32 rules 
)

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)

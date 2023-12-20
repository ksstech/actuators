# ACTUATORS

set( srcs "actuators.c" )
set( include_dirs "." )
set( priv_include_dirs )
idf_build_get_property( cmakeREQUIRES cmakeREQUIRES )
set( requires ${cmakeREQUIRES} )
set( priv_requires common main printfx syslog systiming endpoints hal_esp32 rules )

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)

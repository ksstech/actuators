idf_component_register(
	SRCS "actuators.c"
	INCLUDE_DIRS "."
	REQUIRES statistics
	PRIV_REQUIRES irmacos endpoints rules pca9555 printfx systiming
	)

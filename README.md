# actuators
	ESP32 GPIO on/via SoC/I2C/SPI actuator support library with hardware and software PWM

	Old IDENT module actuator were numbered 0->99.
	In addition mapping provided for 0->99 to also be addressed as 100->199.
	The last 2 digits (00->99) indicate an offset value to be applied (added) to the base/input channel.
	Thus if the input was chan=0 the 108 would translate to actuator chan=8 and 109 to actuator chan=9.
 
	New V2 IDENT module makes provisions for fewer base/input channels (0-31).
	This change allows for more translation logics to be defined.
	To provide for 100-199 compatibility, 32->63 provides same logic just for a smaller base range.
	
	
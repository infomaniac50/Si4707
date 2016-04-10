## Si4707 SAME Weather Radio ##

### Original Description ###
Copied from http://www.raydees.com/Weather_Radio.html and modified
so it works with the SparkFun Si4707 module.  (Requred a change of
the I2C addresss of the chip.)

### Description ###
I was using the Sparkfun version but it was a pain in the \*\*\*\*\*\*!!  
The new radio is from http://www.aiwindustries.com/ and I can tell you it is much easier to work with.
It has a built-in BNC connector for an antenna tuned to the Weather Band and no pesky voltage dividers on the VIO pins (come on Sparkfun :{P).
It needs a level shifter if you intend to use a 5v uC but one of those can be had from https://www.adafruit.com/.

### Suggested Hardware Additions per AIW Industries ###
[I2C Level Shifter](http://www.adafruit.com/products/757) from Adafruit (Enables use with 5V Microcontrollers)  
[Audio Amplifier](http://moderndevice.com/product/audio-amplifier-1x/) module from Modern Devices  

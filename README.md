# arduino-quadcopter
Code to stabilize a quadcopter using an Arduino Uno and an MPU6050 accelerometer/gyroscope

This code uses the [DSSCircuits I2C library](https://github.com/DSSCircuits/I2C-Master-Library) as an alternative to the Wire library.

I previously used [this library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) for doing digital motion processing on the MPU6050.
However, this library was not well optimized for use in a quadcopter, where extended periods of acceleration happen frequently.
The discrepancy between the local acceleration vector and the actual direction of gravity frequently caused stability issues.

In addition, the Wire library does not provice an I2C transmission timeout.  If a failed I2C transmission occurred, then the entire Arduino would lock up and become unresponsive.

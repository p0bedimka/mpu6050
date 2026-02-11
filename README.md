# MPU6050

The MPU6050 sensor driver supports all the functionality described in the dataset (DMP, FIFO, etc.), and is extended as a calibration function.

# Usage example

[Example](https://github.com/p0bedimka/imu-firmware) It shows how to configure this sensor, please note that the device ID should be 0x34. 
If the device ID is 0x38, then this sensor (MPU6500) with this DMP firmware will not work correctly, and accelerometer calibration will not occur.
We need to fix the displacement registers for the accelerometer (0x77, 0x7A, 0x7D).

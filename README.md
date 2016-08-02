Hardware:
Beaglebone + ST Micro IMU sensor with L3GD20 and LSM303DLHC + 4 PWM Electronic Speed Controllers  BLDC Motors 

Software: Enable PWM Pins by using Beaglebone DTS overlays as most of the pins are multiplexed
			sensor.c has code for interfacing with the IMU unit and ESC's.

			 Compile sensor.c over an SSH session.

Uses a Compilmentary filer with (~2-3% of accelerometer data + ~97-98% of gyroscope data to maintain orientation in air )

Issues: Damping of vibration. Vibrations can cause spurious data in gyroscope leading to errors in control signals


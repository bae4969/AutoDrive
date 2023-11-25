# AutoDrive
	This project is for personal research project.
	This project needs Raspberry Pi and PiCar-X (expansion module)
		
# Overviews
	1. Main control has xpub-xsub proxy server and it bind ipc endpoint.
	2. All hardware connect to ipc endpoint and publish its states. (current degree, target degree, ...)
	3. All hardware connect to ipc endpoint and subscribe its command topic. (change target degree, change speed, ...)
	4. Proxy server also bind tcp endpoint, port 45000, 45001
	5. Other pc can subscribe hardware state or publish hardware command by network

# Linearly Control PWM and Servo Motor Method
	In real world, human controls car gently, like accelerate faster and faster and faster or slower and slower and stop.
	However, the pwm and servo motor does not support linear movement.
	So, PWM and servo motor have target value and delta changing value.
	PWM and servo motor have thread to gradually change to the target value during the delta time.
	And user can change target value and delta changing value.
	Strictly speaking, it's not completely linear, but I made it as close as possible.

# Sending Image Data Method
	First of all, when transmitting image data, the most important factors are processing speed.
	When car moving at 100 km/h and image processing speed is 30 FPS, The moving distance between two frames is about 0.9m.
	0.9m makes difficult to find correlations between frames.
	So client(raspi) needs to transmit data as quickly as possible to control server.
	It seems like sending raw data looks good, not encoded data, becuase of poor processor.
	However, larger images and faster FPS seem to make network bandwidth issues.
	
	230828
	- change to encoding jpg


# Libraries
	1. opencv4
		- use for image buffer (not essential)
	2. raspicam_cv
		- use for get image via raspberry camera
	3. wiringPi
		- use for controling gpio pins
	4. wiringPiI2C
		- use for controling I2C board
	5. cppzmq
		- use for controling hardwares
		- use for network controls

# Hardwares
	1. Raspberry Pi 4B
	2. PiCar-X
		1) I2C Protocol Board
		2) Three Servo motors
			- Steer
			- Camera Yaw
			- Camera Pitch
		3) Two PWM motors
			- Left Rear
			- Right Rear
   	3. Lidar
    		1) USB serial protocol
	3. Power
		- USB-C to 9V DC

# TODO
	1. Change camera from raspi cam to stereo depth detection camera
 	2. Need encode data or config auth user
	


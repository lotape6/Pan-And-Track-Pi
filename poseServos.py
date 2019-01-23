from time import sleep
#import RPi.GPIO as GPIO
import PID
import pigpio
#from server_script import obj_center
from server_script import objectSelected

pi = pigpio.pi('192.168.1.15')
# START VARIABLES --------------------------------------------------------------
Pan_gpioPIN = 23
Tilt_gpioPIN = 22
Pan_initPosition = 1500
Tilt_initPosition = 1500
frameSize = (640, 480)
frameCenter = (frameSize[0]/2,frameSize[1]/2)
obj_center=(345,242)
Pan_max_range = 700
Tilt_max_range = 400

# PID'S DEFINITION -------------------------------------------------------------
Pan_pidTarget = 0
Tilt_pidTarget = 0

Pan_Kp = 10
Pan_Ki = 1
Pan_Kd = 1
Tilt_Kp = 10
Tilt_Ki = 1
Tilt_Kd = 1

Pan_pid = PID.PID(Pan_Kp, Pan_Ki, Pan_Kd)
Pan_pid.SetPoint = Pan_pidTarget
Pan_pid.setSampleTime(1)
Tilt_pid = PID.PID(Tilt_Kp, Tilt_Ki, Tilt_Kd)
Tilt_pid.SetPoint = Tilt_pidTarget
Tilt_pid.setSampleTime(1)

# GPIO DEFINITION --------------------------------------------------------------
pi.set_mode(Pan_gpioPIN, pigpio.OUTPUT)
pi.set_mode(Tilt_gpioPIN, pigpio.OUTPUT)

# CALCULATE ANGLE AND MOVE SERVOS FUNCTION -------------------------------------
def setServoAngle(servo, angle):
	#pi.set_PWM_frequency(servo,50)
	if servo == Pan_gpioPIN:
		if angle > (Pan_initPosition + Pan_max_range):
			angle =  Pan_initPosition + Pan_max_range
		elif angle < (Pan_initPosition - Pan_max_range):
			 angle =  Pan_initPosition - Pan_max_range
	if servo == Tilt_gpioPIN:
		if angle > (Tilt_initPosition + Tilt_max_range):
			angle =  Tilt_initPosition + Tilt_max_range
		elif angle < (Tilt_initPosition - Tilt_max_range):
			 angle =  Tilt_initPosition - Tilt_max_range

	pulsewidth = angle
	#dutyCycle = angle / 18. + 3.
	print(pulsewidth)
	pi.set_servo_pulsewidth(servo,pulsewidth)
	#pi.set_PWM_dutycycle(servo,dutyCycle)
	sleep(0.4)
	pi.set_servo_pulsewidth(servo,0)
	#pi.set_PWM_dutycycle(servo,0)

# CALCULATE PID ERROR FUNCTION -------------------------------------------------
def calculatePIDerror(i):
	# The same function for both servos
	# Known: Image frame center and object position in frame (from tracking)
	pid_error = frameCenter[i] - obj_center[i]
	return pid_error

if __name__ == '__main__':
	# INITIAL SETUP ------------------------------------------------------------
	setServoAngle(Pan_gpioPIN, Pan_initPosition)
	setServoAngle(Tilt_gpioPIN, Tilt_initPosition)
	#
	# # START PID LOOP -----------------------------------------------------------
	while True:
		# Calculate error for both servos
		Pan_pidError = calculatePIDerror(0) #"pan" = 0
		Tilt_pidError = calculatePIDerror(1) #"tilt" = 0
		# Update pid signal control value
		Pan_pid.update(Pan_pidError)
		Tilt_pid.update(Tilt_pidError)
		# Calculate value to send to servos: position + pid output
		Pan_Position = Pan_initPosition + Pan_pid.output
		Tilt_Position = Tilt_initPosition + Tilt_pid.output
        	#print("New Pan output:    ", Pan_pid.output)
        	#print("New Tilt output:    ", Tilt_pid.output)
        	print("New Pan Angle:    ", Pan_Position)
       	 	print("New Tilt Angle:    ", Tilt_Position)
		# Move servos to the calculated position
		setServoAngle(Pan_gpioPIN, Pan_Position)
		setServoAngle(Tilt_gpioPIN, Tilt_Position)

	#setServoAngle(servo, int(sys.argv[2]))
#	GPIO.cleanup()

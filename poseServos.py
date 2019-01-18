from time import sleep
import RPi.GPIO as GPIO
import PID
from server_script import obj_center

# START VARIABLES --------------------------------------------------------------
Pan_gpioPIN = 17
Tilt_gpioPIN = 23
Pan_initPosition = 125
Tilt_initPosition = 160
frameSize = (640, 480)
frameCenter = (frameSize[0]/2,frameSize[1]/2)

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
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# CALCULATE ANGLE AND MOVE SERVOS FUNCTION -------------------------------------
def setServoAngle(servo, angle):
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dutyCycle = angle / 18. + 3.
	pwm.ChangeDutyCycle(dutyCycle)
	sleep(0.3)
	pwm.stop()

# CALCULATE PID ERROR FUNCTION -------------------------------------------------
def calculatePIDerror(servo):
	# The same function for both servos
	if axis="pan":
		i = 0
	elif axis="tilt"
		i = 1
	else:
		print("calculatePIDerror failed")
	# Known: Image frame center and object position in frame (from tracking)
	pid_error = frameCenter[i] - obj_center[i]
	return pid_error

if __name__ == '__main__':
	#import sys
	#servo = int(sys.argv[1])

	# INITIAL SETUP ------------------------------------------------------------
	GPIO.setup(Pan_gpioPIN, GPIO.OUT)
	GPIO.setup(Tilt_gpioPIN, GPIO.OUT)
	setServoAngle(Pan_gpioPIN, Pan_initPosition)
	setServoAngle(Tilt_gpioPIN, Tilt_initPosition)

	# START PID LOOP -----------------------------------------------------------
	while True:
		# Calculate error for both servos
		Pan_pidError = calculatePIDerror("pan")
		Tilt_pidError = calculatePIDerror("tilt")
		# Update pid signal control value
		Pan_pid.update(Pan_pidError)
		Tilt_pid.update(Tilt_pidError)
		# Calculate value to send to servos: position + pid output
		Pan_Position = Pan_initPosition + Pan_pid.output
		Tilt_Position = Tilt_initPosition + Tilt_pid.output
		# Move servos to the calculated position
		setServoAngle(Pan_gpioPIN, Pan_Position)
		setServoAngle(Tilt_gpioPIN, Tilt_Position)

	#setServoAngle(servo, int(sys.argv[2]))
	GPIO.cleanup()

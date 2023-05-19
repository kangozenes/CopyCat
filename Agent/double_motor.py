import time
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685
# import Jetson.GPIO as GPIO

# Initialize PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000

# Right motor BOARD = 11,12
r1 = digitalio.DigitalInOut(board.D17)
r2 = digitalio.DigitalInOut(board.D18)
r1.direction = digitalio.Direction.OUTPUT
r2.direction = digitalio.Direction.OUTPUT

# Left motor BOARD = 15,16
l1 = digitalio.DigitalInOut(board.D22)
l2 = digitalio.DigitalInOut(board.D23)
l1.direction = digitalio.Direction.OUTPUT
l2.direction = digitalio.Direction.OUTPUT

print("READY")
pca.channels[0].duty_cycle = 0x8fff  # Channel 0 for ENA
time.sleep(1)
print("OK")
pca.channels[0].duty_cycle = 0x0000  # Channel 0 for ENA

try:
	# Motor direction control
	r1.value = True
	r2.value = False
	l1.value = False
	l2.value = True
	
	# Motor speed control
	for i in range(0x0000, 0x6fff, 0x50):
		pca.channels[0].duty_cycle = i  # Channel 0 for ENA
		pca.channels[4].duty_cycle = i  # Channel 0 for ENA
		pca.channels[5].duty_cycle = i  # Channel 0 for ENA
		time.sleep(0.01)

	for i in range(0x6fff, 0x3fff, -0x50):
		pca.channels[0].duty_cycle = i  # Channel 0 for ENA
		pca.channels[4].duty_cycle = i  # Channel 0 for ENA
		pca.channels[5].duty_cycle = i  # Channel 0 for ENA
		time.sleep(0.01)
	while True:
		pca.channels[0].duty_cycle = 0x3fff  # Channel 0 for ENA
		pca.channels[4].duty_cycle = 0x3fff  # Channel 0 for ENA
		pca.channels[5].duty_cycle = 0x3fff  # Channel 0 for ENA

	#	m1.value = False
	#	m2.value = True
	#	time.sleep(2)  # Run motor in this direction for 2 seconds


except KeyboardInterrupt:  # If CTRL+C is pressed, exit cleanly:
	r1.value=False
	r2.value=False
	l1.value=False
	l2.value=False	
	pca.channels[0].duty_cycle = 0x0000  # Channel 0 for ENA
	GPIO.cleanup()  # cleanup all GPIO


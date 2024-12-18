import RPi.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins connected to the motor driver
IN1 = 13  # IN1 to GPIO 17
IN2 = 12  # IN2 to GPIO 18
ENA = 23  # ENA to GPIO 23 (PWM pin for speed control)

# Set all the pins as outputs
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Set up PWM for speed control
pwm = GPIO.PWM(ENA, 1000)  # 1kHz PWM frequency
pwm.start(0)  # Start with 0% duty cycle (motor off)

# Function to move the motor forward
def motor_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(25)  # Set speed to 75%

# Function to stop the motor
def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)  # Motor off

# Function to move the motor backward
def motor_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(25)  # Set speed to 75%

try:
    motor_forward()
    time.sleep(5)  # Run motor for 5 seconds
    motor_stop()
    time.sleep(1)  # Stop for 1 second
    motor_backward()
    time.sleep(5)  # Run motor backward for 5 seconds
    motor_stop()

finally:
    GPIO.cleanup()  # Clean up the GPIO settings

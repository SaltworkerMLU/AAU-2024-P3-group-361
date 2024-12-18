import gpiozero as gpio
import time

servo = gpio.AngularServo(4)
motor = gpio.Motor(12, 13)



IR = gpio.Button(23)

LED = gpio.LED(24)

IRCount = 1

time.sleep(30)

while True:
    servo.angle = 0
    motor.forward(0.1)
    start_time = time.perf_counter()
    now_time = start_time
    while now_time - start_time < 30:
        now_time = time.perf_counter()
        servo.angle = 0
    motor.backward(0.1)
    time.sleep(0.4)
    motor.stop()
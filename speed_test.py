import gpiozero as gpio
import time

servo = gpio.AngularServo(4)
motor = gpio.Motor(12, 13)



IR = gpio.Button(23)

LED = gpio.LED(24)

IRCount = 1

while True:
    servo.angle = 0
    while IRCount % 2 != 0:
        motor.stop()
        LED.off()
        print("stap")
        print(IRCount)
        #print(IR)
        if IR.is_active == True:
            IRCount += 1
            time.sleep(0.2)
            LED.on()

    while IRCount % 2 == 0:
        print("go")
        print(IRCount)
        LED.on()
        motor.forward(0.09)
        while True:
            if IR.is_active == True:
                motor.stop()
                time.sleep(0.2)
                break
        #print(IR)
        IRCount += 1
        motor.backward(0.1)
        time.sleep(0.4)
        motor.stop()
        if IR.is_active == True:
            IRCount += 1
            time.sleep(0.2)
            LED.off()
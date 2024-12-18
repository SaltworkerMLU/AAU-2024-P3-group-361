import gpiozero as gpio
import time

servo = gpio.AngularServo(4)
motor = gpio.Motor(12, 13)



IR = gpio.Button(23)

LED = gpio.LED(24)

IRCount = 1

while True:

    while IRCount % 2 != 0:
        motor.stop()
        print("stap")
        print(IRCount)
        #print(IR)
        if IR.is_active == True:
            IRCount += 1
            time.sleep(0.2)
            LED.on()

        #time.sleep(0.5)

    while IRCount % 2 == 0:
        print("go")
        print(IRCount)
        servo.angle = -90
        motor.forward(0.06)
        start_time = time.perf_counter()
        now_time = start_time
        while now_time - start_time < 20:
            now_time = time.perf_counter()
            if IR.is_active == True:
                motor.stop()
                time.sleep(0.2)
                #IRCount += 1
                #time.sleep(0.2)
                #LED.on()
                break
        #print(IR)
        IRCount += 1
        motor.stop()
        """if IR.is_active == True:
            IRCount += 1
            time.sleep(0.2)
            LED.off()"""
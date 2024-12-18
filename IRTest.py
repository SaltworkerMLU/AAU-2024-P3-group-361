import gpiozero as gpio
import time

IR = gpio.Button(23)

LED = gpio.LED(24)

IRCount = 1

while True:

    while IRCount % 2 != 0:
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
        #print(IR)
        if IR.is_active == True:
            IRCount += 1
            time.sleep(0.2)
            LED.off()
        
        #time.sleep(0.5)
        

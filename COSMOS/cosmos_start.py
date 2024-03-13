import RPi.GPIO as GPIO
import time
import os
but_pin = 13

def main():

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(but_pin, GPIO.IN)
    curr_value = GPIO.input(but_pin)
    if curr_value == 0:
        tmp = 1
    else :
        tmp = 0

    try:
        while True:
            curr_value = GPIO.input(but_pin)
            print(curr_value)

            if curr_value == tmp:
                os.system("rosrun donkey_control run.py")
                break
            time.sleep(1)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()

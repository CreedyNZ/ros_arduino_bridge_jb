import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
resetpin = 13
GPIO.setup(resetpin, GPIO.OUT)
GPIO.output(resetpin, GPIO.HIGH)
time.sleep(0.1)
GPIO.output(resetpin, GPIO.LOW)
time.sleep(0.1)
GPIO.output(resetpin, GPIO.HIGH)
time.sleep(2)
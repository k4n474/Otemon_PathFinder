"""Provide on/off control and timed patterns for the buzzer on BCM GPIO19."""

import RPi.GPIO as GPIO
from time import sleep

BUZZER = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)

def buzzer_start():
    GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        
    
def buzzer_stop():
    GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
    
def buzzer_sleep(time = 0.1):
    """Sound one beep for the requested duration in seconds."""
    GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
    sleep(time)
    GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.

def hurt_beats():
    """Repeat a heartbeat-like sequence of paired beeps until interrupted."""
    while True:
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.8)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.8)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # Sound the buzzer.
        sleep(8)
        
        GPIO.output(BUZZER, GPIO.LOW)   # Silence the buzzer.
        sleep(0.8)




def main():
    hurt_beats()
    
if __name__ == '__main__':
    main()

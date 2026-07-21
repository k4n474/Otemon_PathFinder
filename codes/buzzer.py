import RPi.GPIO as GPIO
from time import sleep

BUZZER = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)

def buzzer_start():
    GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        
    
def buzzer_stop():
    GPIO.output(BUZZER, GPIO.LOW)   # 止める
    
def buzzer_sleep(time = 0.1):
    GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
    sleep(time)
    GPIO.output(BUZZER, GPIO.LOW)   # 止める

def hurt_beats():
    while True:
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.1)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.7)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.8)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.8)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.05)
        
        GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
        sleep(8)
        
        GPIO.output(BUZZER, GPIO.LOW)   # 止める
        sleep(0.8)




def main():
    hurt_beats()
    
if __name__ == '__main__':
    main()
"""競技開始ボタンの入力処理。"""

import time

import RPi.GPIO as GPIO


BUTTON_PIN = 26
BUTTON_DEBOUNCE_SECONDS = 0.05
BUTTON_POLL_INTERVAL_SECONDS = 0.01


def button_sleep(pin=BUTTON_PIN):
    """指定したGPIOのボタンが押されるまで待機する。

    ボタンは内部プルアップを使用し、押下時にGPIOとGNDが接続される
    アクティブLOW配線を想定する。
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print(f"GPIO{pin}のスタートボタンを押してください")
    while True:
        if GPIO.input(pin) == GPIO.LOW:
            time.sleep(BUTTON_DEBOUNCE_SECONDS)
            if GPIO.input(pin) == GPIO.LOW:
                print("スタートボタンが押されました")
                return

        time.sleep(BUTTON_POLL_INTERVAL_SECONDS)

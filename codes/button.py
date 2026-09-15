"""Handle the button used to start a competition run."""

import time

import RPi.GPIO as GPIO


BUTTON_PIN = 26
BUTTON_DEBOUNCE_SECONDS = 0.05
BUTTON_POLL_INTERVAL_SECONDS = 0.01


def button_sleep(pin=BUTTON_PIN):
    """Wait until the button on the specified GPIO pin is pressed.

    Use the internal pull-up resistor with active-low wiring: pressing the
    button connects the GPIO pin to ground.
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print(f"GPIO{pin}のスタートボタンを押してください")
    while True:
        if GPIO.input(pin) == GPIO.LOW:
            # Confirm the input stays low after contact bounce has settled.
            time.sleep(BUTTON_DEBOUNCE_SECONDS)
            if GPIO.input(pin) == GPIO.LOW:
                print("スタートボタンが押されました")
                return

        time.sleep(BUTTON_POLL_INTERVAL_SECONDS)

import time

from tt_motor import tt_motor


def main():
    motor = tt_motor()
    duty_cycles = [20, 40, 60, 80]
    try:
        for duty_cycle in duty_cycles:
            actual = motor.forward(duty_cycle)
            print(f"後輪モーター単体テスト: forward 指定{duty_cycle}% -> 出力{actual}%")
            time.sleep(1.5)

            print("stop")
            motor.stop()
            time.sleep(0.8)

        for duty_cycle in duty_cycles:
            actual = motor.backward(duty_cycle)
            print(f"後輪モーター単体テスト: backward 指定{duty_cycle}% -> 出力{actual}%")
            time.sleep(1.5)

            print("stop")
            motor.stop()
            time.sleep(0.8)
    finally:
        motor.cleanup()


if __name__ == "__main__":
    main()

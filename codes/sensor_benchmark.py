import time

from gyro import close_gyro, get_angle
from ultrasound import cleanup_sensors, dis_get, us_get


def bench_ultrasound_single(count=30):
    print("ultrasound us_get()")
    total = 0.0
    misses = 0
    for index in range(count):
        start = time.perf_counter()
        distance = us_get()
        elapsed = time.perf_counter() - start
        total += elapsed
        if distance is None:
            misses += 1
            text = "None"
        else:
            text = f"{distance:.2f} cm"
        print(f"{index + 1:02d}: {elapsed * 1000:7.2f} ms  {text}")
    print(f"avg: {total / count * 1000:.2f} ms, misses: {misses}/{count}")


def bench_ultrasound_median(samples=10, count=5):
    print(f"ultrasound dis_get(samples={samples})")
    for index in range(count):
        start = time.perf_counter()
        distance = dis_get(samples)
        elapsed = time.perf_counter() - start
        text = "None" if distance is None else f"{distance:.2f} cm"
        print(f"{index + 1:02d}: {elapsed * 1000:7.2f} ms  {text}")


def bench_gyro(count=100):
    print("gyro get_angle('z')")
    start = time.perf_counter()
    for _ in range(count):
        angle = get_angle("z")
    elapsed = time.perf_counter() - start
    print(f"last angle: {angle:.2f} deg")
    print(f"total: {elapsed * 1000:.2f} ms, avg: {elapsed / count * 1000:.2f} ms")


def main():
    try:
        bench_ultrasound_single()
        print()
        bench_ultrasound_median()
        print()
        bench_gyro()
    finally:
        cleanup_sensors()
        close_gyro()


if __name__ == "__main__":
    main()

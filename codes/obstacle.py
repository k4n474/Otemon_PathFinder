"""Control the robot during the obstacle challenge.

The run has three main stages:
1. Find a target object with the camera.
2. Avoid objects and black walls while counting blue-line crossings.
3. Stop after the required crossings and challenge-specific finish conditions.
"""

import math
import time

import cv2
import numpy as np
import RPi.GPIO as GPIO

from camera_detector import (
    BOTTOM_EXCLUSION_SIZE,
    PiColorDetector,
    build_primary_target_line,
    object_front_priority,
)
from gyro import get_angle, reset_angle, close_gyro
# from ultrasound import us_get, dis_get, us_back_get, dis_back_get

from buzzer import buzzer_start, buzzer_stop, buzzer_sleep, hurt_beats
from button import button_sleep
from lidar_read import LidarReader
from lidar_wall_follow import follow_wall_until_front_distance
from park import (
    start_parking_0, start_parking_1, start_parking_2,
    start_parking_3, start_parking_4, start_parking_5, stop_parking,
    start_parking_6, start_parking_7,
)
# Initialize hardware interfaces.
# ---------------------------------------------------------------------------

lidar = LidarReader(scan_frequency_increase_hz=6)

# ---------------------------------------------------------------------------
# Driving settings.
# ---------------------------------------------------------------------------


# Gyro-controlled turns.
GYRO_TURN_TIMEOUT_SECONDS = 20.0  # Maximum duration of a gyro turn, in seconds.


# PD control for object and wall avoidance.
AVOID_GREEN_TARGET_ANGLE = -40  # Target line angle when avoiding a green object, in degrees.
AVOID_RED_TARGET_ANGLE = 45  # Target line angle when avoiding a red object, in degrees.
AVOID_GREEN_TARGET_ANGLE_BELOW_150 = -35  # Green target angle when the object center has Y > 150 pixels.
AVOID_RED_TARGET_ANGLE_BELOW_150 = 40  # Red target angle when the object center has Y > 150 pixels.
AVOID_STEERING_MAX = 30  # Maximum steering angle during object avoidance, in degrees.
AVOID_WALL_STEERING_ANGLE = 30.0  # Fixed steering angle for black-wall avoidance, in degrees.
AVOID_KP = 1.5  # Proportional gain for target-line angle error.
AVOID_KD = 0.1# Derivative gain for changes in target-line angle.
AVOID_HOLD_STEERING_BELOW_Y = 200  # Below this image Y coordinate, drive straight or hold the steering angle.
AVOID_STRAIGHT_ANGLE_TOLERANCE = 5.0  # Drive straight when the target-line angle is within this tolerance, in degrees.
AVOID_WALL_STEERING_UPDATE_MIN = 0.5  # Minimum angle change needed to update wall-avoidance steering, in degrees.
AVOID_POWER_BOOST_STEERING_THRESHOLD = 30  # Steering-angle threshold for increasing motor power, in degrees.
AVOID_POWER_BOOST = 0  # Extra motor power during sharp steering.

# Initial backup check.
BACK_CHECK_AREA_THRESHOLD = 2000  # Minimum object area in pixels required to trigger a backup.
BACK_CHECK_SECONDS = 1.25  # Duration of the backup maneuver, in seconds.


# Blue-line crossing detection.
BLUE_LINE_COOLDOWN_SECONDS = 2.5  # Ignore interval after a crossing to prevent duplicate counts, in seconds.
BLUE_LINE_CROSSING_TARGET = 12  # Required crossing count before checking finish conditions.
BLUE_LINE_CROSSING_TARGET_P = 13  # Required crossing count for obstacle_challenge_p only.
BLUE_LINE_LOST_CONFIRM_SECONDS = 1.5  # Required blue-line absence for direction=0, in seconds.
BLUE_LINE_LOST_CONFIRM_SECONDS_DIRECTION_ONE = 2  # Required blue-line absence for direction=1, in seconds.

# Keep the rear light on throughout the obstacle challenge.
REAR_LIGHT_PIN = 21  # BCM GPIO pin connected to the rear light.

detector = PiColorDetector(
    enable_recording=True,
    detect_boundary_enabled=False,
    detect_court_enabled=True,
    bottom_exclusion_size=BOTTOM_EXCLUSION_SIZE,
)
from newobot import dc_motor, set_angle, stop, brake, cleanup

# ---------------------------------------------------------------------------
# State shared across driving phases.
# ---------------------------------------------------------------------------

blue_line_crossing_count = 0
blue_line_was_detected = False
blue_line_ignore_until = 0.0


def drive_along_wall_until_front_distance(
    stop_distance,
    *,
    target_side_distance=250.0,
    duty_cycle=42.0,
    trace_side=None,
    timeout=30.0,
):
    """Keep the side-wall distance until the front wall reaches the specified distance."""
    started_lidar_here = not lidar.running

    try:
        if started_lidar_here:
            lidar.start()

        return follow_wall_until_front_distance(
            lidar,
            front_stop_distance=stop_distance,
            target_side_distance=target_side_distance,
            motor_speed=duty_cycle,
            trace_side=trace_side,
            timeout=timeout,
            drive_motor=dc_motor,
            set_steering=set_angle,
            stop_motor=stop,
        )
    finally:
        if started_lidar_here:
            lidar.stop()


def gyro_turn(
    target_angle,
    steering_angle,
    duty_cycle,
    timeout=GYRO_TURN_TIMEOUT_SECONDS,
):
    """Drive at a fixed steering angle until yaw reaches the target, then stop.

    The sign of target_angle determines the gyro turn direction; the sign
    of steering_angle is not used for the completion check. Compare yaw
    against target_angle relative to the last gyro reset; do not reset here.
    """
    if timeout <= 0:
        raise ValueError("timeout は0より大きくしてください。")

    started_at = time.monotonic()
    turned_angle = get_angle("z")
    first_angle = turned_angle

    def target_reached():
        if target_angle >= first_angle:
            return turned_angle >= target_angle
        return turned_angle <= target_angle

    try:
        set_angle(steering_angle)
        dc_motor(duty_cycle)

        while not target_reached():
            turned_angle = get_angle("z")
            print(
                f"ジャイロ旋回: {turned_angle:.1f}°"
                f" / 目標: {target_angle:.1f}°",
                end="\r",
                flush=True,
            )

            if time.monotonic() - started_at >= timeout:
                raise RuntimeError(
                    f"{target_angle:.1f}度のジャイロ旋回がタイムアウトしました"
                )

            time.sleep(0.01)
    finally:
        stop()
        set_angle(0)

    print(f"\nジャイロ旋回完了: {turned_angle:.1f}°")
    return turned_angle


def gyro_pd(
    power,
    duration,
    target_angle=None,
    *,
    kp=1.0,
    kd=0.1,
    max_steering=30.0,
    interval=0.01,
):
    """Hold a yaw angle with PD steering for duration seconds, then stop.

    Positive power drives forward; negative power drives backward. If omitted,
    target_angle is the starting yaw. Explicit targets use get_angle("z")'s
    reference (the last gyro reset); this function does not reset the gyro.
    Return the last measured yaw. Gains and steering angles are in degrees.
    """
    values = (power, duration, kp, kd, max_steering, interval)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("gyro_pd の引数は有限の数値にしてください。")
    if not 0 < abs(power) <= 100:
        raise ValueError("power は -100〜100 の範囲で、0以外にしてください。")
    if duration <= 0 or interval <= 0:
        raise ValueError("duration と interval は0より大きくしてください。")
    if kp < 0 or kd < 0 or not 0 < max_steering <= 50:
        raise ValueError("kp・kd は0以上、max_steering は0より大きく50以下にしてください。")
    if target_angle is not None and not math.isfinite(target_angle):
        raise ValueError("target_angle は有限の数値にしてください。")

    travel_sign = 1.0 if power > 0 else -1.0
    try:
        # Initialize/calibrate the gyro while stationary, before starting the timer.
        stop()
        current_angle = get_angle("z")
        if not math.isfinite(current_angle):
            raise RuntimeError("ジャイロの角度が不正です。")
        if target_angle is None:
            target_angle = current_angle
        previous_error = None
        previous_time = None
        deadline = time.monotonic() + duration
        motor_started = False

        while True:
            current_angle = get_angle("z")
            if not math.isfinite(current_angle):
                raise RuntimeError("ジャイロの角度が不正です。")
            now = time.monotonic()
            if now >= deadline:
                break
            error = target_angle - current_angle
            derivative = 0.0
            if previous_time is not None and now > previous_time:
                derivative = (error - previous_error) / (now - previous_time)
            steering = travel_sign * (kp * error + kd * derivative)
            set_angle(max(-max_steering, min(max_steering, steering)))
            if not motor_started:
                dc_motor(power)
                motor_started = True
            previous_error = error
            previous_time = now
            time.sleep(min(interval, max(0.0, deadline - time.monotonic())))
        return current_angle
    finally:
        stop()
        set_angle(0)


def update_blue_line_crossing(result, finish_state=None):
    """Count a crossing when a blue line changes from invisible to visible.

    Ignore blue lines briefly after each count to avoid counting the same
    line twice. The counter persists across avoid_obj calls for the full run.
    """
    global blue_line_crossing_count
    global blue_line_was_detected
    global blue_line_ignore_until

    crossing_target = (finish_state or {}).get("crossing_target", BLUE_LINE_CROSSING_TARGET)
    if blue_line_crossing_count >= crossing_target:
        return blue_line_crossing_count

    current_time = time.monotonic()
    if current_time < blue_line_ignore_until:
        return blue_line_crossing_count

    blue_line_is_detected = result.get("blue_line") is not None
    if blue_line_is_detected and not blue_line_was_detected:
        blue_line_crossing_count += 1
        blue_line_was_detected = True
        blue_line_ignore_until = current_time + BLUE_LINE_COOLDOWN_SECONDS
        detector.set_blue_line_crossing_count(blue_line_crossing_count)
        print(f"\nBLUE LINE DETECTED: {blue_line_crossing_count}")
    elif not blue_line_is_detected:
        blue_line_was_detected = False

    return blue_line_crossing_count


def blue_line_finish_reached(
    finish_state=None,
    finish_delay_seconds=0.0,
    result=None,
    require_magenta_absent=False,
):
    """Check the blue-line crossing target and any additional finish conditions.

    If require_magenta_absent is enabled, wait until the last blue line has
    stayed invisible for the direction-specific duration, then check the
    number of magenta objects.
    """
    crossing_target = (finish_state or {}).get("crossing_target", BLUE_LINE_CROSSING_TARGET)
    if blue_line_crossing_count < crossing_target:
        return False

    if require_magenta_absent:
        if finish_state is None:
            return False

        if result is not None:
            current_time = time.monotonic()
            blue_line_is_detected = result.get("blue_line") is not None
            direction = finish_state.get("direction", 0)
            blue_line_lost_confirm_seconds = (
                BLUE_LINE_LOST_CONFIRM_SECONDS_DIRECTION_ONE
                if direction == 1
                else BLUE_LINE_LOST_CONFIRM_SECONDS
            )

            if not finish_state.get("blue_line_cleared", False):
                if blue_line_is_detected:
                    finish_state["blue_line_absent_since"] = None
                elif finish_state.get("blue_line_absent_since") is None:
                    finish_state["blue_line_absent_since"] = current_time
                elif (
                    current_time - finish_state["blue_line_absent_since"]
                    >= blue_line_lost_confirm_seconds
                ):
                    finish_state["blue_line_cleared"] = True
                    print(
                        "\n青線が"
                        f"{blue_line_lost_confirm_seconds:.1f}秒以上"
                        "見えなくなりました"
                    )

            if finish_state.get("blue_line_cleared", False):
                magenta_count = len(result.get("magenta_objects", []))
                magenta_limit = 1 if direction == 1 else 0
                finish_state["magenta_condition_met"] = (
                    magenta_count <= magenta_limit
                )

        if not finish_state.get("blue_line_cleared", False):
            return False
        if not finish_state.get("magenta_condition_met", False):
            return False

    if finish_state is None or finish_delay_seconds <= 0.0:
        return True

    current_time = time.monotonic()
    if finish_state["finish_at"] is None:
        finish_state["finish_at"] = current_time + finish_delay_seconds
        print(
            f"\n青線を目標回数検出: あと{finish_delay_seconds:.1f}秒走行します"
        )

    return current_time >= finish_state["finish_at"]


def select_front_object(result):
    """Select the red or green object that appears lowest in the image."""
    candidates = []
    for red_object in result["red_objects"]:
        candidates.append(("red", red_object))

    for green_object in result["green_objects"]:
        candidates.append(("green", green_object))

    if not candidates:
        return None, None

    return max(candidates, key=lambda item: object_front_priority(item[1]))


def back_check(power, keep_camera_running=False):
    """Check one frame, back up if needed, and return the magenta object's side.

    Returns:
        int: 0 if magenta is on the left or absent; 1 if it is on the right.
    """
    started_detector_here = detector.camera is None
    try:
        if started_detector_here:
            detector.start()

        result = detector.process_once()
        _color_name, obj = select_front_object(result)
        magenta_obj = select_front_magenta_object(result)

        magenta_direction = 0
        if magenta_obj is not None:
            frame_center_x = result["frame"].shape[1] / 2.0
            magenta_direction = 0 if magenta_obj["center"][0] < frame_center_x else 1

        if obj is not None and obj["area"] >= BACK_CHECK_AREA_THRESHOLD:
            print("Need to back")
            set_angle(0)
            dc_motor(power)
            time.sleep(BACK_CHECK_SECONDS)

        return magenta_direction
    finally:
        stop()
        set_angle(0)
        if started_detector_here and not keep_camera_running:
            detector.stop()


def select_front_magenta_object(result):
    """Select the detected magenta object that appears closest in the image."""
    objects = result.get("magenta_objects", [])
    return max(objects, key=object_front_priority) if objects else None


def find_obj(
    duty_cycle,
    rd,
    finish_state=None,
    finish_delay_seconds=0.0,
    require_magenta_absent=False,
):
    """Check the camera while stopped, then steer and drive to search if needed.

    Returns:
        str | None: The detected color, or None when the finish conditions
        are met. KeyboardInterrupt stops the robot and is re-raised.
    """
    if rd not in (0, 1):
        raise ValueError("find_obj の rd は0または1にしてください。")

    steering_angle = AVOID_STEERING_MAX
    search_started = False
    wall_avoiding = False
    detector.set_black_wall_probe_direction(rd)
    detector.set_search_black_wall_probe_enabled(True)
    try:
        while True:
            result = detector.process_once()
            crossing_count = update_blue_line_crossing(result, finish_state)
            if blue_line_finish_reached(
                finish_state,
                finish_delay_seconds,
                result,
                require_magenta_absent,
            ):
                stop()
                set_angle(0)
                return None

            # Use a fixed lower-right probe for rd=0 and lower-left probe for rd=1.
            # Steer away from the wall while the black-pixel fraction meets the detector threshold.
            if result.get("black_wall_on_probe", False):
                wall_steering = (
                    -AVOID_WALL_STEERING_ANGLE
                    if rd == 0
                    else AVOID_WALL_STEERING_ANGLE
                )
                if not wall_avoiding:
                    set_angle(wall_steering)
                    wall_avoiding = True
                if not search_started:
                    dc_motor(duty_cycle)
                    search_started = True
                continue

            # Resume the normal search direction once the wall leaves the probe.
            if wall_avoiding:
                normal_steering = steering_angle if rd == 0 else -steering_angle
                set_angle(normal_steering)
                wall_avoiding = False

            color_name, obj = select_front_object(result)

            if obj is not None:
                center_x, center_y = obj["center"]
                width, height = obj["size"]
                stop()
                set_angle(0)
                return color_name

            if not search_started:
                if rd == 0:
                    set_angle(steering_angle)
                else:
                    set_angle(steering_angle * -1)
                dc_motor(duty_cycle)
                search_started = True

    except KeyboardInterrupt:
        stop()
        set_angle(0)
        raise
    finally:
        detector.set_search_black_wall_probe_enabled(False)


def avoid_obj(
    duty_cycle,
    direction,
    finish_state=None,
    finish_delay_seconds=0.0,
    require_magenta_absent=False,
):
    """Handle object PD avoidance and black-wall avoidance in one camera loop.

    direction=1 uses the default probe position and right steering;
    direction=0 mirrors the probe and steers left. Resume object PD control
    once the black wall is no longer detected.
    """
    if direction not in (0, 1):
        raise ValueError("avoid_obj の direction は0または1にしてください。")

    detector.set_black_wall_probe_direction(direction)
    wall_steering_sign = 1.0 if direction == 1 else -1.0
    previous_error = None
    previous_time = None
    previous_color = None
    previous_wall_steering = None
    previous_target_angle = None
    # The search ends at zero steering; keep this saved angle separate from wall avoidance.
    object_steering = 0.0
    current_duty_cycle = duty_cycle
    buzzer_active = False
    buzzer_stop()
    dc_motor(duty_cycle)

    while True:
        result = detector.process_once()
        crossing_count = update_blue_line_crossing(result, finish_state)
        primary = result.get("primary_detection")

        if blue_line_finish_reached(
            finish_state,
            finish_delay_seconds,
            result,
            require_magenta_absent,
        ):
            buzzer_stop()
            stop()
            set_angle(0)
            return

        # Give black-wall avoidance priority over object avoidance while the wall is visible.
        if result.get("black_wall_on_probe", False):
            if buzzer_active:
                buzzer_stop()
                buzzer_active = False
            if current_duty_cycle != duty_cycle:
                dc_motor(duty_cycle)
                current_duty_cycle = duty_cycle
            wall_steering = wall_steering_sign * AVOID_WALL_STEERING_ANGLE
            if (
                previous_wall_steering is None
                or abs(wall_steering - previous_wall_steering)
                >= AVOID_WALL_STEERING_UPDATE_MIN
            ):
                set_angle(wall_steering)
                previous_wall_steering = wall_steering

            # Reset PD history to prevent a derivative spike after wall avoidance.
            previous_error = None
            previous_time = None
            previous_color = None
            continue

        wall_just_cleared = previous_wall_steering is not None
        previous_wall_steering = None

        control_target = primary
        target_line = build_primary_target_line(control_target, result["frame"])
        line_angle = target_line["angle_deg"] if target_line is not None else None
        if control_target is None or line_angle is None:
            buzzer_stop()
            stop()
            set_angle(0)
            return

        color_name, obj = control_target
        object_y = obj["center"][1]
        target_angle = (
            AVOID_GREEN_TARGET_ANGLE
            if color_name == "green"
            else AVOID_RED_TARGET_ANGLE
        )
        if object_y > 150:
            target_angle = (
                AVOID_GREEN_TARGET_ANGLE_BELOW_150
                if color_name == "green"
                else AVOID_RED_TARGET_ANGLE_BELOW_150
            )
        error = line_angle - target_angle
        if object_y > AVOID_HOLD_STEERING_BELOW_Y:
            if buzzer_active:
                buzzer_stop()
                buzzer_active = False
            if current_duty_cycle != duty_cycle:
                dc_motor(duty_cycle)
                current_duty_cycle = duty_cycle
            if abs(error) <= AVOID_STRAIGHT_ANGLE_TOLERANCE:
                set_angle(0)
                object_steering = 0.0
            # Outside the tolerance, hold the saved angle; restore it after wall avoidance.
            elif wall_just_cleared:
                set_angle(object_steering)
            previous_error = None
            previous_time = None
            previous_color = None
            continue
            

        current_time = time.monotonic()

        if (
            previous_error is None
            or previous_time is None
            or previous_color != color_name
            or previous_target_angle != target_angle
        ):
            derivative = 0.0
        else:
            elapsed = current_time - previous_time
            derivative = (
                (error - previous_error) / elapsed
                if elapsed > 0.0
                else 0.0
            )

        steering = AVOID_KP * error + AVOID_KD * derivative
        steering = max(-AVOID_STEERING_MAX, min(AVOID_STEERING_MAX, steering))
        steering_over_threshold = (
            abs(steering) > AVOID_POWER_BOOST_STEERING_THRESHOLD
        )
        if steering_over_threshold and not buzzer_active:
            buzzer_start()
            buzzer_active = True
        elif not steering_over_threshold and buzzer_active:
            buzzer_stop()
            buzzer_active = False

        boosted_duty_cycle = min(100.0, duty_cycle + AVOID_POWER_BOOST)
        desired_duty_cycle = (
            boosted_duty_cycle
            if steering_over_threshold
            else duty_cycle
        )
        if current_duty_cycle != desired_duty_cycle:
            dc_motor(desired_duty_cycle)
            current_duty_cycle = desired_duty_cycle
            if desired_duty_cycle == boosted_duty_cycle:
                print(
                    "[avoid_obj] ステアリング角が"
                    f"±{AVOID_POWER_BOOST_STEERING_THRESHOLD:g}°を超えました: "
                    f"steering={steering:+.1f}°, power={desired_duty_cycle:.1f}"
                )
            else:
                print(
                    "[avoid_obj] ステアリング角が"
                    f"±{AVOID_POWER_BOOST_STEERING_THRESHOLD:g}°以内に戻りました: "
                    f"steering={steering:+.1f}°, power={desired_duty_cycle:.1f}"
                )
        set_angle(steering)
        object_steering = steering
        previous_error = error
        previous_time = current_time
        previous_color = color_name
        previous_target_angle = target_angle


def _run_obstacle_challenge(
    power,
    direction,
    finish_delay_seconds=0.0,
    require_magenta_absent=False,
    crossing_target=BLUE_LINE_CROSSING_TARGET,
    brake_on_finish=False,
):
    """Run the obstacle challenge; return True only on normal completion."""
    finish_state = {
        "crossing_target": crossing_target,
        "finish_at": None,
        "direction": direction,
        "blue_line_absent_since": None,
        "blue_line_cleared": False,
        "magenta_condition_met": False,
    }
    try:
        # Reuse the running camera passed on by back_check or out_park.
        if detector.camera is None:
            detector.start()
        # lidar.start()

        while True:
            # Search until a target object is found.
            find_obj(
                power ,
                direction,
                finish_state,
                finish_delay_seconds,
                require_magenta_absent,
            )

            if blue_line_finish_reached(
                finish_state,
                finish_delay_seconds,
                require_magenta_absent=require_magenta_absent,
            ):
                break

            # Avoid the detected target and nearby walls.
            avoid_obj(
                power,
                direction,
                finish_state,
                finish_delay_seconds,
                require_magenta_absent,
            )

            if blue_line_finish_reached(
                finish_state,
                finish_delay_seconds,
                require_magenta_absent=require_magenta_absent,
            ):
                break

        if brake_on_finish:
            brake()
        else:
            stop()
        set_angle(0)
        return True
    except KeyboardInterrupt:
        print("\nCtrl+C を受け付けたため終了します")
        stop()
        set_angle(0)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"エラー: {exc}")
    finally:
        buzzer_stop()
        detector.stop()
        # close_gyro()
        if detector.recording_path is not None:
            print(
                f"録画保存: scp otm@192.168.137.213:{detector.recording_path} "
                "~/workspace/pivideos"
            )
        # cleanup()


def obstacle_challenge(power, direction):
    """Apply electrical braking after the normal finish conditions are met."""
    completed = _run_obstacle_with_rear_light(
        power,
        direction,
        require_magenta_absent=True,
    )
    if completed:
        brake()
    return completed


def obstacle_challenge_p(power, direction, po=None):
    """Finish with direction- and parking-position-specific braking behavior."""
    brake_on_finish = direction == 0
    finish_delay_seconds = 0.5 if direction != 0 or po == 1 else 0.0
    return _run_obstacle_with_rear_light(
        power,
        direction,
        finish_delay_seconds=finish_delay_seconds,
        require_magenta_absent=False,
        crossing_target=BLUE_LINE_CROSSING_TARGET_P,
        brake_on_finish=brake_on_finish,
    )


def _run_obstacle_with_rear_light(
    power,
    direction,
    finish_delay_seconds=0.0,
    require_magenta_absent=False,
    crossing_target=BLUE_LINE_CROSSING_TARGET,
    brake_on_finish=False,
):
    """Keep the rear light on during the challenge and always switch it off afterward."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(REAR_LIGHT_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(REAR_LIGHT_PIN, GPIO.HIGH)
    try:
        return _run_obstacle_challenge(
            power,
            direction,
            finish_delay_seconds=finish_delay_seconds,
            require_magenta_absent=require_magenta_absent,
            crossing_target=crossing_target,
            brake_on_finish=brake_on_finish,
        )
    finally:
        GPIO.output(REAR_LIGHT_PIN, GPIO.LOW)


def obstacle_challenge_np(power):
    """Stop using direction-specific blue-line absence times and magenta counts."""
    # button_sleep()
    try:
        direction = back_check(
            (power + 10) * -1,
            keep_camera_running=True,
        )
        _run_obstacle_challenge(
            power,
            direction,
            require_magenta_absent=True,
        )
    finally:
        GPIO.output(REAR_LIGHT_PIN, GPIO.LOW)

def out_park(keep_camera_running=False):
    """Return (direction, po) using the initial and post-move object checks.

    Direction 0: initially nearby red=6, green=7. Otherwise, after moving,
    front object area >= 500: red=1, green=2; absent=0.
    Direction 1: 3 if no nearby object, 4 for red, or 5 for green.
    """
    reset_angle()
    started_detector_here = detector.camera is None
    try:
        if started_detector_here:
            detector.start()
        result = detector.process_once()
        height, width = result["frame"].shape[:2]
        court = result.get("court")
        court_mask = court["mask"] if court is not None else np.zeros((height, width), dtype=np.uint8)

        center_x = width // 2
        left_mask = court_mask[:, :center_x]
        right_mask = court_mask[:, center_x:]
        left_ratio = cv2.countNonZero(left_mask) / left_mask.size if left_mask.size else 0.0
        right_ratio = cv2.countNonZero(right_mask) / right_mask.size if right_mask.size else 0.0
        # Left (1) is also the fallback when both sides are equal or absent.
        direction = 0 if right_ratio > left_ratio else 1

        set_angle(0)
        dc_motor(-30)
        time.sleep(0.5)
        stop()

        if direction == 0:
            set_angle(35)
        else:
            set_angle(-40)
        dc_motor(30)
        time.sleep(1.1)

        set_angle(0)
        time.sleep(0.5)
        brake()
        time.sleep(1)

        result = detector.process_once()
        nearby_objects = [
            (color, obj)
            for color in ("red_objects", "green_objects")
            for obj in result.get(color, [])
            if obj["area"] >= 1000
        ]
        object_detected = bool(nearby_objects)
        po = 0 if direction == 0 else 3
        if nearby_objects:
            color, _obj = max(
                nearby_objects, key=lambda item: object_front_priority(item[1])
            )
            if direction == 0:
                po = 6 if color == "red_objects" else 7
            else:
                po = 4 if color == "red_objects" else 5
        print(object_detected)

        if object_detected == True:
            if direction == 0:
                gyro_turn(0,-25,20)
                gyro_pd(-30,3,target_angle=-10, kp=2.0, kd=0.1)
            else:
                dc_motor(30)
                time.sleep(0.6)
                stop()
                gyro_turn(0,25,20)
                gyro_pd(-30,3.4,target_angle=10, kp=2.0, kd=0.1)

        if object_detected == False:
            if direction == 0:
                dc_motor(30)
                time.sleep(0.8)
                brake()
                time.sleep(0.2)
                result = detector.process_once()
                color_name, front_obj = select_front_object(result)
                if front_obj is not None and front_obj["area"] >= 500:
                    po = 1 if color_name == "red" else 2
                    print(f"一番手前のオブジェクト: 色={color_name}, 面積={front_obj['area']}")



            

        return direction, po
    finally:
        stop()
        if started_detector_here and not keep_camera_running:
            detector.stop()


def main():
    parking_thread = None
    # button_sleep()
    try:
        direction, po = out_park(keep_camera_running=True)
        if po == 0:
            # direction=0: 最初に近接物体なし、追加判定でも対象なし
            if obstacle_challenge_p(30, direction, po):
                parking_thread = start_parking_0(direction)
                parking_thread.join()

        elif po == 1:
            # direction=0: 追加判定で赤
            if obstacle_challenge_p(30, direction, po):
                parking_thread = start_parking_1(direction)
                parking_thread.join()

        elif po == 2:
            # direction=0: 追加判定で緑
            if obstacle_challenge_p(30, direction, po):
                parking_thread = start_parking_2(direction)
                parking_thread.join()

        elif po == 3:
            # direction=1: 近接物体なし
            if obstacle_challenge_p(30, direction, po):
                parking_thread = start_parking_3(direction)
                parking_thread.join()

        elif po == 4:
            # direction=1: 近接物体が赤
            if obstacle_challenge_p(30, direction, po):
                parking_thread = start_parking_4(direction)
                parking_thread.join()

        elif po == 5:
            # direction=1: 近接物体が緑
            if obstacle_challenge_p(30, direction, po):
                parking_thread = start_parking_5(direction)
                parking_thread.join()

        elif po == 6:
            # direction=0: 最初の近接物体が赤
            if obstacle_challenge_p(30, direction, po):
                parking_thread = start_parking_6(direction)
                parking_thread.join()

        elif po == 7:
            # direction=0: 最初の近接物体が緑
            print("7777777777777777777777777")
            if obstacle_challenge_p(30, direction, po):
                parking_thread = start_parking_7(direction)
                parking_thread.join()
    finally:
        if parking_thread is not None:
            stop_parking()
        stop()
        detector.stop()

if __name__ == "__main__":
    main()

"""Control the robot during the obstacle challenge.

The run has three main stages:
1. Find a target object with the camera.
2. Avoid objects and black walls while counting blue-line crossings.
3. Stop after the required crossings and challenge-specific finish conditions.
"""

import time

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
from park import start_parking
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
from newobot import dc_motor, set_angle, stop, cleanup

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


def update_blue_line_crossing(result):
    """Count a crossing when a blue line changes from invisible to visible.

    Ignore blue lines briefly after each count to avoid counting the same
    line twice. The counter persists across avoid_obj calls for the full run.
    """
    global blue_line_crossing_count
    global blue_line_was_detected
    global blue_line_ignore_until

    if blue_line_crossing_count >= BLUE_LINE_CROSSING_TARGET:
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
    if blue_line_crossing_count < BLUE_LINE_CROSSING_TARGET:
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
            crossing_count = update_blue_line_crossing(result)
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
        crossing_count = update_blue_line_crossing(result)
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
):
    """Run the obstacle challenge with the specified finish delay."""
    finish_state = {
        "finish_at": None,
        "direction": direction,
        "blue_line_absent_since": None,
        "blue_line_cleared": False,
        "magenta_condition_met": False,
    }
    try:
        # Reuse the running camera when it was passed on by back_check.
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

        stop()
        set_angle(0)
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
    """Stop using the same blue-line absence and magenta-count rules as the np variant."""
    _run_obstacle_with_rear_light(
        power,
        direction,
        require_magenta_absent=True,
    )


def _run_obstacle_with_rear_light(
    power,
    direction,
    finish_delay_seconds=0.0,
    require_magenta_absent=False,
):
    """Keep the rear light on during the challenge and always switch it off afterward."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(REAR_LIGHT_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(REAR_LIGHT_PIN, GPIO.HIGH)
    try:
        _run_obstacle_challenge(
            power,
            direction,
            finish_delay_seconds=finish_delay_seconds,
            require_magenta_absent=require_magenta_absent,
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

def out_park(direction):
    set_angle(0)
    dc_motor(-30)
    time.sleep(0.5)
    stop()

    if direction == 0:
        set_angle(40)
    else:
        set_angle(-40)
    dc_motor(30)
    time.sleep(1)

    set_angle(0)
    time.sleep(1)
    stop()

def main():
    
    button_sleep()
    obstacle_challenge_np(30)
    stop()

if __name__ == "__main__":
    main()

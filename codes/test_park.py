"""実機を動かさず、駐車部品の到達判定・停止と手順を確認する。"""
import unittest
from unittest.mock import Mock, patch

import park
import park_control as control


class MovementTest(unittest.TestCase):
    def setUp(self):
        control.prepare()
        gains = patch.object(control, "SIDE_KP", 0.1)
        gains.start()
        self.addCleanup(gains.stop)
        front_gain = patch.object(control, "FRONT_KP", 1.0)
        front_gain.start()
        self.addCleanup(front_gain.stop)
        self.motor = Mock()
        self.hardware = patch.object(control, "_hardware", self.motor)
        self.hardware.start()
        self.wait = patch.object(control, "wait")
        self.wait.start()

    def tearDown(self):
        self.wait.stop()
        self.hardware.stop()
        control.prepare()

    def test_turn_reaches_absolute_angle_from_nonzero_start(self):
        with patch.object(control, "_yaw", side_effect=[100, 100, 140, 180, 180]):
            self.assertEqual(control.gyro_turn(180, 40, 35), 180)
        self.assertTrue(all(0 < c.args[0] <= 35 for c in self.motor.dc_motor.call_args_list))
        self.motor.brake.assert_called_once()
        self.motor.stop.assert_called_once()
        self.motor.set_angle.assert_called_with(0)

    def test_default_turn_uses_last_reset_reference(self):
        with patch.object(control, "_yaw", side_effect=[60, 60, 80, 90, 90]):
            self.assertEqual(control.gyro_turn(90, -40, 35), 90)
        self.assertEqual(self.motor.dc_motor.call_count, 2)

    def test_turn_accelerates_decelerates_and_brakes_before_stop(self):
        for sign in (1, -1):
            self.motor.reset_mock()
            # 発進→通常速度→残り10°→残り2°→目標到達→停止後の実測。
            with patch.object(control, "_yaw", side_effect=[
                sign * v for v in [0, 0, 20, 80, 88, 90, 91]
            ]), patch.object(control.time, "monotonic", side_effect=[0, 0, 0.4, 0.8, 0.9]):
                self.assertEqual(control.gyro_turn(sign * 90, -40, sign * 35), sign * 91)
            powers = [abs(c.args[0]) for c in self.motor.dc_motor.call_args_list]
            self.assertLess(powers[0], powers[1])
            self.assertGreater(powers[1], powers[2])
            self.assertGreater(powers[2], powers[3])
            self.assertTrue(all(c.args[0] * sign > 0 for c in self.motor.dc_motor.call_args_list))
            names = [c[0] for c in self.motor.mock_calls]
            self.assertLess(names.index("brake"), names.index("stop"))

    def test_backward_angle_is_not_added_to_current_yaw(self):
        with patch.object(control, "_read_walls", side_effect=[self.frame(600), self.frame(1200)]), \
             patch.object(control, "_yaw", return_value=60):
            control.gyro_backward(90, 30, 1200)
        self.assertEqual(self.motor.set_angle.call_args_list[0].args[0], -30)

    def test_absolute_turn_can_cross_negative_target(self):
        with patch.object(control, "_yaw", side_effect=[10, 10, -40, -81, -81]):
            self.assertEqual(control.gyro_turn(-80, -40, -30), -81)

    def test_sensor_failure_stops_motor(self):
        with patch.object(control, "_yaw", side_effect=[0, 0, RuntimeError("sensor")]):
            with self.assertRaises(RuntimeError):
                control.gyro_turn(80, 40, 35)
        self.motor.stop.assert_called_once()
        self.motor.set_angle.assert_called_with(0)

    def test_timeout_stops_motor(self):
        with patch.object(control, "_yaw", return_value=0), patch.object(
            control.time, "monotonic", side_effect=[0, 11]
        ):
            with self.assertRaises(TimeoutError):
                control.gyro_turn(80, 40, 35)
        self.motor.stop.assert_called_once()

    def test_all_four_distance_thresholds(self):
        for forward, backward, side_mode in [
            (control.lidar_forward, control.lidar_backward, True),
            (control.gyro_forward, control.gyro_backward, False),
        ]:
            for fn, distances, power in [(forward, [300, 200, 150], 30),
                                         (backward, [150, 300, 450], -30)]:
                self.motor.reset_mock()
                frames = [self.frame(d, 250) for d in distances]
                with patch.object(control, "_read_walls", side_effect=frames), \
                     patch.object(control, "_yaw", return_value=0):
                    result = fn(distances[-1]) if side_mode else fn(0, 30, distances[-1])
                    self.assertEqual(result, distances[-1])
                self.assertEqual(self.motor.dc_motor.call_count, 2)
                self.motor.dc_motor.assert_called_with(power)
                self.motor.stop.assert_called_once()

    @staticmethod
    def frame(front_distance, side_distance=None, side="left"):
        walls = []
        if front_distance is not None:
            walls.append({"wall_distance": front_distance, "front_distance": front_distance, "is_front_wall": True, "role": "front"})
        if side_distance is not None:
            walls.append({"wall_distance": side_distance, "is_side_wall": True,
                          "side": side, "role": side, "length": 600})
        return walls

    def test_missing_front_resets_confirmation(self):
        frames = [self.frame(d) for d in [140, None, 140, 130]]
        with patch.object(control, "_read_walls", side_effect=frames), \
             patch.object(control, "_yaw", return_value=0):
            self.assertEqual(control._lidar_drive(150, 30, steering="gyro", kp=1, kd=0.1, max_steering=30, confirm_samples=2), 130)
        self.assertEqual(self.motor.stop.call_count, 2)

    def test_side_pd_mirrors_left_right_and_reverse(self):
        for side, forward_angle in [("left", -10), ("right", 10)]:
            for fn, target, final, expected in [
                (control.lidar_forward, 150, 150, forward_angle),
                (control.lidar_backward, 800, 800, -forward_angle),
            ]:
                self.motor.reset_mock()
                with patch.object(control, "_read_walls", side_effect=[
                    self.frame(600, 350, side), self.frame(final, 350, side)
                ]):
                    fn(target, 30, side, 250)
                self.assertEqual(self.motor.set_angle.call_args_list[0].args[0], expected)

    def test_missing_side_stops_and_initial_distance_is_held(self):
        with patch.object(control, "SIDE_KD", 0), patch.object(control, "_read_walls", side_effect=[
            self.frame(600), self.frame(600, 300),
            self.frame(500, 350), self.frame(150, 350)
        ]):
            control.lidar_forward(150)
        self.assertEqual(self.motor.stop.call_count, 2)
        self.assertEqual([c.args[0] for c in self.motor.set_angle.call_args_list],
                         [0, 0, -5, 0])

    def test_gyro_pd_reverses_steering(self):
        for fn, target, end, expected in [(control.gyro_forward, 150, 150, 10),
                                          (control.gyro_backward, 800, 800, -10)]:
            self.motor.reset_mock()
            with patch.object(control, "_read_walls", side_effect=[self.frame(600), self.frame(end)]), \
                 patch.object(control, "_yaw", return_value=0):
                fn(10, 30, target)
            self.assertEqual(self.motor.set_angle.call_args_list[0].args[0], expected)

    def test_pd_derivative_and_limit(self):
        pd = control._PD()
        with patch.object(control.time, "monotonic", side_effect=[1, 2, 3]):
            self.assertEqual(pd.update(10, 0.1, 0.2, 5), 1)
            self.assertEqual(pd.update(20, 0.1, 0.2, 5), 4)
            self.assertEqual(pd.update(100, 0.1, 0.2, 5), 5)

    def test_negative_public_power_is_rejected(self):
        for fn in [control.lidar_forward, control.lidar_backward,
                   control.gyro_forward, control.gyro_backward]:
            with self.assertRaises(ValueError):
                if fn in (control.gyro_forward, control.gyro_backward):
                    fn(90, -30, 150)
                else:
                    fn(150, power=-30)
        self.motor.dc_motor.assert_not_called()

    def test_lidar_timeout_stops_motor(self):
        with patch.object(control.time, "monotonic", side_effect=[0, 31]):
            with self.assertRaises(TimeoutError):
                control.lidar_forward(150)
        self.motor.stop.assert_called_once()

    def test_algorithm_front_and_side_result_is_used(self):
        walls = self.frame(600, 250, "left")
        walls[0]["normal_angle"] = 100
        with patch.object(control, "_read_walls", return_value=walls):
            front, sides = control.read_front_and_side_walls()
        self.assertIs(front, walls[0])
        self.assertEqual(sides["left"]["wall_distance"], 250)
        self.assertIsNone(sides["right"])
        reached_walls = self.frame(150)
        reached_walls[0]["normal_angle"] = 100
        with patch.object(control, "_read_walls", side_effect=[walls, reached_walls]):
            control.lidar_front(150, 30)
        self.assertEqual(self.motor.set_angle.call_args_list[0].args[0], -15)
        self.motor.dc_motor.assert_called_once_with(30)

    def test_front_tilt_control_without_side_wall(self):
        for normal, expected in [(100, -15), (80, 5), (91, -6), (140, -30)]:
            self.motor.reset_mock()
            with patch.object(control, "read_front_wall", side_effect=[
                {"wall_distance": 600, "normal_angle": normal},
                {"wall_distance": 150, "normal_angle": normal},
            ]), patch.object(control, "_yaw") as yaw:
                self.assertEqual(control.lidar_front(150, 30, "right", 250), 150)
            self.assertEqual(self.motor.set_angle.call_args_list[0].args[0], expected)
            self.motor.dc_motor.assert_called_once_with(30)
            self.motor.stop.assert_called_once()
            yaw.assert_not_called()

    def test_front_reverse_reverses_steering_and_distance_condition(self):
        with patch.object(control, "read_front_wall", side_effect=[
            {"wall_distance": 200, "normal_angle": 100},
            {"wall_distance": 450, "normal_angle": 100},
        ]):
            self.assertEqual(control.lidar_front(450, -30, "left", 250), 450)
        self.motor.dc_motor.assert_called_once_with(-30)
        self.assertEqual(self.motor.set_angle.call_args_list[0].args[0], 15)
        self.motor.stop.assert_called_once()
        self.motor.set_angle.assert_called_with(0)

    def test_front_missing_wall_stops_until_reacquired(self):
        with patch.object(control, "read_front_wall", side_effect=[
            None, {"wall_distance": 600, "normal_angle": 90},
            {"wall_distance": 150, "normal_angle": 90},
        ]):
            control.lidar_front(150, 30, "left", 250)
        self.assertEqual(self.motor.stop.call_count, 2)
        self.motor.dc_motor.assert_called_once_with(30)

    def test_cancel_prevents_motor_start(self):
        control.cancel()
        with self.assertRaises(control.ParkingCancelled):
            control.dc_motor(30)
        self.motor.dc_motor.assert_not_called()


class SequenceTest(unittest.TestCase):
    def test_sequence_calls_four_argument_wall_functions(self):
        with patch.multiple(park, reset_angle=Mock(), pause=Mock(),
                            read_front_wall_angle=Mock(return_value=90.0),
                            gyro_turn=Mock(), lidar_forward=Mock(), lidar_front=Mock(),
                            lidar_backward=Mock(), gyro_forward=Mock(), gyro_backward=Mock()):
            park.parking_sequence(park.RUN_DIRECTION)
            for fn in (park.lidar_forward, park.lidar_backward):
                for call in fn.call_args_list:
                    self.assertEqual(len(call.args), 4)
                    self.assertEqual(call.kwargs, {})

    def test_shutdown_on_sequence_error(self):
        with patch.object(park, "parking_sequence", side_effect=RuntimeError("failure")), \
             patch.object(park, "shutdown") as shutdown, \
             patch.object(park, "start_lidar_viewer"):
            park.control_running = True
            with self.assertRaises(RuntimeError):
                park._run(1)
            shutdown.assert_called_once()
            self.assertFalse(park.control_running)


class FrontWallAngleTest(unittest.TestCase):
    def setUp(self):
        control.prepare()

    def test_waits_for_scan_and_front_wall(self):
        with patch.object(control, "read_front_wall", side_effect=[
            None, None, {"normal_angle": 94.5},
        ]) as read, patch.object(control, "wait") as wait, \
             patch.object(control.time, "monotonic", return_value=0):
            self.assertEqual(control.read_front_wall_angle(detection_range=3000), 94.5)
        self.assertEqual(read.call_count, 3)
        read.assert_called_with(final_wall=False, detection_range=3000)
        self.assertEqual(wait.call_count, 2)

    def test_missing_wall_times_out(self):
        with patch.object(control, "read_front_wall", return_value=None), \
             patch.object(control.time, "monotonic", side_effect=[0, 0.1, 2.0]), \
             patch.object(control, "wait") as wait:
            self.assertIsNone(control.read_front_wall_angle())
        wait.assert_called_once()

    def test_zero_timeout_reads_once(self):
        with patch.object(control, "read_front_wall", return_value=None) as read, \
             patch.object(control, "wait") as wait:
            self.assertIsNone(control.read_front_wall_angle(timeout=0))
        read.assert_called_once()
        wait.assert_not_called()

    def test_cancel_interrupts_retry(self):
        with patch.object(control, "read_front_wall", return_value=None), \
             patch.object(control, "wait", side_effect=lambda _: control.cancel()), \
             patch.object(control.time, "monotonic", return_value=0):
            try:
                with self.assertRaises(control.ParkingCancelled):
                    control.read_front_wall_angle()
            finally:
                control.prepare()

    def test_invalid_timeout(self):
        for timeout in (-1, float("nan"), float("inf")):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                control.read_front_wall_angle(timeout=timeout)


if __name__ == "__main__":
    unittest.main()

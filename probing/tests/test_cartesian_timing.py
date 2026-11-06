import math
import unittest

from probing.backends.ros2_moveit_backend import ROS2MoveItBackend


class CartesianTimingTest(unittest.TestCase):
    def test_trapezoidal_profile_duration_and_midpoint(self):
        time_at = ROS2MoveItBackend._trapezoidal_time_at_distance
        self.assertAlmostEqual(time_at(0.0, 0.03, 0.01, 0.05), 0.0)
        self.assertAlmostEqual(time_at(0.015, 0.03, 0.01, 0.05), 1.6)
        self.assertAlmostEqual(time_at(0.03, 0.03, 0.01, 0.05), 3.2)

    def test_short_path_uses_triangular_profile(self):
        time_at = ROS2MoveItBackend._trapezoidal_time_at_distance
        total_distance = 0.001
        acceleration = 0.05
        expected_duration = 2.0 * math.sqrt(total_distance / acceleration)
        self.assertAlmostEqual(
            time_at(total_distance, total_distance, 0.01, acceleration),
            expected_duration,
        )

    def test_profile_times_are_monotonic(self):
        time_at = ROS2MoveItBackend._trapezoidal_time_at_distance
        times = [
            time_at(distance, 0.03, 0.01, 0.05)
            for distance in (0.0, 0.001, 0.005, 0.015, 0.025, 0.029, 0.03)
        ]
        self.assertEqual(times, sorted(times))

    def test_quaternion_sign_does_not_change_angle(self):
        angle = ROS2MoveItBackend._quaternion_distance(
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0, -1.0),
        )
        self.assertAlmostEqual(angle, 0.0)

    def test_pilz_linear_scaling_matches_configured_limits(self):
        backend = ROS2MoveItBackend.__new__(ROS2MoveItBackend)
        backend.cartesian_speed_mps = 0.01
        backend.cartesian_acceleration_mps2 = 0.05
        backend.cartesian_angular_speed_radps = 0.2
        backend.cartesian_angular_acceleration_radps2 = 0.5

        velocity, acceleration = backend._pilz_scaling_factors("linear")

        self.assertAlmostEqual(velocity, 0.1)
        self.assertAlmostEqual(acceleration, 0.1)

    def test_pilz_angular_scaling_matches_derived_acceleration(self):
        backend = ROS2MoveItBackend.__new__(ROS2MoveItBackend)
        backend.cartesian_speed_mps = 0.01
        backend.cartesian_acceleration_mps2 = 0.05
        backend.cartesian_angular_speed_radps = 0.2
        backend.cartesian_angular_acceleration_radps2 = 0.5

        velocity, acceleration = backend._pilz_scaling_factors("angular")

        self.assertAlmostEqual(velocity, 0.1)
        self.assertAlmostEqual(acceleration, 0.05)

    def test_pilz_rejects_speed_above_cartesian_limit(self):
        backend = ROS2MoveItBackend.__new__(ROS2MoveItBackend)
        backend.cartesian_speed_mps = 0.11
        backend.cartesian_acceleration_mps2 = 0.05
        backend.cartesian_angular_speed_radps = 0.2
        backend.cartesian_angular_acceleration_radps2 = 0.5

        with self.assertRaises(ValueError):
            backend._pilz_scaling_factors("linear")

    def test_point_line_distance_detects_cartesian_deviation(self):
        error = ROS2MoveItBackend._point_line_distance(
            (0.5, 0.002, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(error, 0.002)


if __name__ == "__main__":
    unittest.main()
